"""
ros_sensor.py

This custom sensor supports:
- dynamic loading of ROS Messages from both ROS and custom workspace libraries
  (assuming ros environment scripts properly exported in module config)
- Capture of all ROS data in message as python dictionary
- upload of messages based on events

{
 ...
 events: [
   {
     name: 'EVENT_NAME',
     eval_start: 'event_start_eval',
     eval_stop:  'event_stop_eval',
 ]
}
"""
import importlib
import logging

import numpy as np
import rospy
import threading

from array import array
from datetime import datetime as dt
from typing import Any, ClassVar, List, Mapping, Optional, Sequence, Union
from typing_extensions import Self

from viam.components.sensor import Sensor
from viam.errors import NoCaptureToStoreError
from viam.logging import getLogger
from viam.module.types import Reconfigurable
from viam.resource.types import Model, ModelFamily
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import ResourceName
from viam.resource.base import ResourceBase
from viam.resource.registry import Registry, ResourceCreatorRegistration

from filtering.cache import global_event_table
from filtering.cache.component_cache import ComponentCache


class RosSensor(Sensor, Reconfigurable):
    MODEL: ClassVar[Model] = Model(ModelFamily('viam-soleng', 'noetic'), 'sensor')
    ros_topic: Union[str, None]
    ros_msg_pkg: Union[str, None]
    ros_msg_type: Union[str, None]
    dm_present: bool
    use_cache: bool
    events: List
    msg_cache: ComponentCache
    msg: Any
    prev_msg: Any
    lock: threading.Lock
    logger: logging.Logger

    @classmethod
    def new(cls, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]) -> Self:
        """

        :param config:
        :param dependencies:
        :return:
        """
        sensor = cls(config.name)
        sensor.reconfigure(config, dependencies)
        return sensor

    @classmethod
    def validate_config(cls, config: ComponentConfig) -> Sequence[str]:
        """

        :param config:
        :return:
        """
        ros_topic = config.attributes.fields['ros_topic'].string_value
        ros_msg_pkg = config.attributes.fields['ros_msg_package'].string_value
        ros_msg_type = config.attributes.fields['ros_msg_type'].string_value

        if ros_topic is None or ros_topic == '':
            raise Exception('ros_topic is a required attribute')

        if ros_msg_pkg is None or ros_msg_pkg == '':
            raise Exception('ros_msg_pkg is a require attribute')

        if ros_msg_type is None or ros_msg_type == '':
            raise Exception('ros_msg_type is a required attribute')
        return []

    def __init__(self, name: str) -> None:
        """
        create sensor object

        :param name:
        """
        super().__init__(name)
        self.logger = getLogger(self.__class__.__name__)    # logger only needs to be setup once
        self.lock = threading.Lock()                        # lock only needs to be setup once
        self.logger.info(f'constructing object: {name}')

        # set all default values (these will be the initial values to compare to new config, etc.)
        self.ros_topic = None
        self.ros_msg_pkg = None
        self.ros_msg_type = None
        self.msg = None
        self.msg_cache = None
        self.prev_msg = None
        self.dm_present = False
        self.use_cache = False
        self.events = []

    def reconfigure(self, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]) -> None:
        """
        reconfigure is called for all components when a configuration update occurs
        we need to detect if the component has changed to decide on building a new cache

        If the topic or data type changes we will clear the prior cache

        :param config:
        :param dependencies:
        :return:
        """
        self.logger.info('reconfigure sensor')
        ros_topic = config.attributes.fields['ros_topic'].string_value
        ros_msg_pkg = config.attributes.fields['ros_msg_package'].string_value
        ros_msg_type = config.attributes.fields['ros_msg_type'].string_value
        events = config.attributes.fields['events'].list_value

        dm_present = False

        for sc in config.service_configs:
            if sc.type == 'rdk:service:data_manager':
                if 'capture_methods' in sc.attributes and len(sc.attributes['capture_methods']) > 0:
                    self.logger.info(f'found {len(sc.attributes["capture_methods"])} capture method(s) for sensor')
                    dm_present = True

        # if any of these three items are changed this sensor needs to be modified
        # TODO: if the message type changes, clear the queue
        if (
            ros_topic != self.ros_topic or              # message topic was changed
            ros_msg_pkg != self.ros_msg_pkg or          # message package was changed
            ros_msg_type != self.ros_msg_type or        # message type was changed
            dm_present != self.dm_present               # data manager was added or removed

        ):
            self.logger.info(f'reconfigure of {config.name} is required, cache will be reset')

            # setup ros attributes (that might have changed)
            self.ros_topic = ros_topic
            self.ros_msg_pkg = ros_msg_pkg
            self.ros_msg_type = ros_msg_type
            self.dm_present = dm_present

            # get ros message type for callback
            lib = importlib.import_module(self.ros_msg_pkg)
            ros_sensor_cls = getattr(lib, self.ros_msg_type)

            # should we set up cache?
            if dm_present:
                # data management is present and collecting we need to use cache
                self.logger.info('setting up cache')

                # now enable cache
                self.events = events
                self.use_cache = True

                if self.msg_cache is None:
                    self.logger.info('creating cache')
                    self.msg_cache = ComponentCache(component_name=config.name)
                else:
                    self.logger.info('updating cache')
                    # change cache window if different from current cache window

            else:
                # no cache - if it was alive before now it is not
                self.logger.info('no cache needed, disabling anything that was previously created')
                self.use_cache = False

            # setup subscriber
            rospy.Subscriber(self.ros_topic, ros_sensor_cls, self.subscriber_callback)

    def subscriber_callback(self, msg):
        """

        :param msg:
        :return:
        """
        with self.lock:
            self.prev_msg = self.msg
            self.msg = build_msg(msg)
            if len(self.events) > 0:
                for event in self.events:
                    start_eval = event['eval_start']
                    stop_eval = event['eval_stop']

                    if eval(start_eval):
                        self.logger.info(f'starting event: {event["name"]}')
                        global_event_table.start_event({'name': event['name'], 'start': dt.now()})

                    if eval(stop_eval):
                        self.logger.info(f'stopping event: {event["name"]}')
                        global_event_table.stop_event({'name': event['name'], 'end': dt.now()})

            if self.use_cache:
                self.msg_cache.add_data(self.msg)

    async def get_readings(
        self,
        *,
        extra: Optional[Mapping[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs
    ) -> Mapping[str, Any]:
        """

        :param extra:
        :param timeout:
        :param kwargs:
        :return:
        """
        if 'fromDataManagement' in extra and extra['fromDataManagement'] is True:
            if self.use_cache:
                data = self.msg_cache.get_data()
                self.logger.debug(f'data management retrieved data: {data}')
                if data is not None:
                    return data
            raise NoCaptureToStoreError()
        if self.msg is not None:
            return self.msg
        return {'value': 'NOT READY'}


def build_msg(msg):
    """
    build the python dictionary from the ros message

    :param msg:
    :return:
    """
    r_data = {}
    if hasattr(msg, '__slots__'):
        for key in msg.__slots__:
            r_data[key] = build_msg(getattr(msg, key))
    else:
        msg_type = type(msg)
        if (
                msg_type is list                    # list []
                or msg_type is tuple                # tuple ()
                or msg_type is set                  # set ()
                or msg_type is array                # array []
                or msg_type is np.ndarray           # numpy array []
                or msg_type is bytes                # bytes 0b...
        ):
            l_data = []
            for value in msg:
                l_data.append(build_msg(value))
            return l_data
        elif msg_type is dict:
            d_data = {}
            for key in msg.keys():
                d_data[key] = build_msg(msg[key])
            return d_data
        else:
            return msg
    return r_data


Registry.register_resource_creator(
    Sensor.SUBTYPE,
    RosSensor.MODEL,
    ResourceCreatorRegistration(RosSensor.new, RosSensor.validate_config)
)
