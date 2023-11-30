import importlib
import logging

import numpy as np
import rospy
import threading

from array import array
from typing import Any, ClassVar, Mapping, Optional, Sequence
from typing_extensions import Self

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
    ros_msg_type: str
    msg_cache: RosTimedCache
    msg: Any
    lock: threading.Lock
    logger: logging.Logger

    @classmethod
    def new(cls, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]) -> Self:
        sensor = cls(config.name)
        sensor.reconfigure(config, dependencies)
        return sensor

    @classmethod
    def validate_config(cls, config: ComponentConfig) -> Sequence[str]:
        return []

    def reconfigure(self, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase])-> None:
        self.logger = getLogger('RosSensor')
        self.ros_topic = config.attributes.fields['ros_topic'].string_value
        ros_msg_pkg = config.attributes.fields['ros_msg_package'].string_value
        self.ros_msg_type = config.attributes.fields['ros_msg_type'].string_value
        self.msg_cache = RosTimedCache()

        lib = importlib.import_module(ros_msg_pkg)
        ros_sensor_cls = getattr(lib, self.ros_msg_type)
        self.lock = threading.Lock()
        rospy.Subscriber(self.ros_topic, ros_sensor_cls, self.subscriber_callback)

    def subscriber_callback(self, msg):
        with self.lock:
            self.msg = build_msg(msg)
            self.msg_cache.add_item(self.msg)

    async def get_readings(
        self,
        *,
        extra: Optional[Mapping[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs
    ) -> Mapping[str, Any]:
        if 'fromDataManagement' in extra and extra['fromDataManagement'] is True:
            self.logger.info('process data manager call')
            return self.msg_cache.get_item()

        if self.msg is not None:
            return self.msg
        return {'value': 'NOT READY'}


def build_msg(msg):
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