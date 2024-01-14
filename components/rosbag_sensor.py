"""
the rosbag sensor will be used to copy data from a rosbag location to another file location

TODO: validate if this is needed
"""
import numpy as np

from array import array
from datetime import datetime as dt
from logging import Logger
from typing import Any, ClassVar, Mapping, Optional, Sequence, Union
from typing_extensions import Self

from viam.components.sensor import Sensor
from viam.module.types import Reconfigurable
from viam.resource.types import Model, ModelFamily
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import ResourceName
from viam.resource.base import ResourceBase
from viam.resource.registry import Registry, ResourceCreatorRegistration


class RosbagSensor(Sensor, Reconfigurable):
    MODEL: ClassVar[Model] = Model(ModelFamily('viam-soleng', 'noetic'), 'rosbag-sensor')
    logger: Logger
    event_start_time: Union[dt, None]
    event_end_time: Union[dt, None]
    rosbag_directory: str

    @classmethod
    def new(cls, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]) -> Self:
        sensor = cls(config.name)
        sensor.reconfigure(config, dependencies)
        return sensor

    @classmethod
    def validate_config(cls, config: ComponentConfig) -> Sequence[str]:
        return []

    def __init__(self, name: str) -> None:
        super().__init__(name)
        self.event_end_time = None
        self.event_start_time = None
        self.rosbag_directory = ''

    def reconfigure(self, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase])-> None:
        return None

    async def get_readings(
        self,
        *,
        extra: Optional[Mapping[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs
    ) -> Mapping[str, Any]:
        if 'fromDataManagement' in extra and extra['fromDataManagement'] is True:
            # inside data management
            return {'value': 'data management data'}
        return {'value': 'NOT READY'}


Registry.register_resource_creator(
    Sensor.SUBTYPE,
    RosbagSensor.MODEL,
    ResourceCreatorRegistration(RosbagSensor.new, RosbagSensor.validate_config)
)