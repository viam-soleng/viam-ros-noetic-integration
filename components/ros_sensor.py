"""

"""
import importlib
import logging

import numpy as np
import rospy
import threading

from array import array
from typing import Any, ClassVar, List, Mapping, Optional, Sequence
from typing_extensions import Self

from google.protobuf.json_format import MessageToDict
from viam.components.sensor import Sensor
from viam.logging import getLogger
from viam.module.types import Reconfigurable
from viam.resource.types import Model, ModelFamily
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import ResourceName
from viam.resource.base import ResourceBase
from viam.resource.registry import Registry, ResourceCreatorRegistration

from .ros_timed_cache import RosTimedCache


class RosSensor(Sensor, Reconfigurable):
    MODEL: ClassVar[Model] = Model(ModelFamily('viam-soleng', 'noetic'), 'sensor')
    ros_topic: str
    ros_msg_pkg: str
    ros_msg_type: str
    dm_present: bool
    use_cache: bool
    cache_window: int
    events: List
    msg_cache: RosTimedCache
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
        #sensor.logger = getLogger(config.name)  # logger only needs to be setup once
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

        cache_window = config.attributes.fields['cache_window'].number_value

        # if any of these three items are changed this sensor needs to be modified
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

                # validate cache window

                # now enable cache
            else:
                # no cache - if it was alive before now it is not
                self.logger.info('no cache needed, disabling anything that was previously created')

            # event data type
            # { ...
            #     events: [ {event_name: '..', event_threshold: '', event_key: '..'}, ...],
            #     cache_window: Xseconds,
            #
            self.events = config.attributes.fields['events'].list_value
            if self.events is not None and len(self.events) > 0:
                # TODO: review with team to validate this configuration
                self.logger.info('processing events configuration')
            else:
                self.logger.warning('no events list to process, disabling cache if it already exists')
                self.use_cache = False

            # create cache & lock
            if cache_window is None or cache_window == 0:
                self.logger.warning('cache_window must be a configured for 1 second or more, will not be configure')
                self.use_cache = False
            else:
                self.msg_cache = RosTimedCache()
                self.use_cache = True

            # finally if we should not use cache
            #if not self.use_cache:


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
                self.msg_cache.add_item(self.msg)

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
                return self.msg_cache.get_item()
            else:
                return self.msg

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