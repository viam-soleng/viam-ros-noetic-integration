"""
ros_sensor.py

This custom sensor supports:
- dynamic loading of ROS Messages from both ROS and custom workspace libraries
  (assuming ros environment scripts properly exported in module config)
- Capture of all ROS data in message as python dictionary
- upload of messages based on events
"""
import importlib
import logging

import numpy as np
import rospy
import threading

from array import array
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

from filtering.filter_caching.component_cache import ComponentCache

class RosSensor(Sensor, Reconfigurable):
    MODEL: ClassVar[Model] = Model(ModelFamily('viam-soleng', 'noetic'), 'sensor')
    ros_topic: Union[str, None]
    ros_msg_pkg: Union[str, None]
    ros_msg_type: Union[str, None]
    dm_present: bool
    use_cache: bool
    cache_window: int
    events: List
    msg_cache: ComponentCache
    msg: Any
    prev_msg: Any
    filter_condition: str
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
        super().__init__(name)
        self.logger = getLogger(self.__class__.__name__)    # logger only needs to be setup once
        self.lock = threading.Lock()                        # lock only needs to be setup once
        self.logger.info(f'constructing object: {name}')

        # set all default values (these will be the initial values to compare to new config, etc.)
        self.ros_topic = None
        self.ros_msg_pkg = None
        self.ros_msg_type = None
        self.msg = None
        self.prev_msg = None
        self.dm_present = False
        self.use_cache = False

    def reconfigure(self, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase])-> None:
        """
        reconfigure is called for all components when a configuration update occurs
        we need to detect if the component has changed to decide on building a new cache

        If the topic or data type changes we will clear the prior cache

        :param config:
        :param dependencies:
        :return:
        """

        ros_topic = config.attributes.fields['ros_topic'].string_value
        ros_msg_pkg = config.attributes.fields['ros_msg_package'].string_value
        ros_msg_type = config.attributes.fields['ros_msg_type'].string_value
        cache_window = config.attributes.fields['cache_window'].number_value
        events = config.attributes.fields['events_info'].list_value

        dm_present = False

        for sc in config.service_configs:
            # TODO: RDK/SDK bug that does not pass attributes of service config
            #
            # work-around: if the data_manager service is present we will assume the
            #              readings method for the sensor is turned on
            #              this will be documented and shared, once this bug is
            #              addressed we will update the code
            if sc.type == 'rdk:service:data_manager':
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
                # validate events
                if events is None or len(events) == 0:
                    raise Exception('cannot configure a cache without setting up at least one event trigger')

                # validate cache window
                if cache_window is None or cache_window == 0:
                    raise Exception('cache_window mus be a valid integer greater than 0, if we are using caches')

                # now enable cache
                self.cache_window = cache_window
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
            # evaluate filter
            # cache?
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
                if data is not None:
                    return data
            raise NoCaptureToStoreError()
        if self.msg is not None:
            return self.msg
        return {'value': 'NOT READY'}


def build_msg(msg):
    """

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