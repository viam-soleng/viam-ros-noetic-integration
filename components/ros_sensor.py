from typing import Any, ClassVar, Mapping, Optional, Sequence
from typing_extensions import Self

from viam.components.sensor import Sensor
from viam.module.types import Reconfigurable
from viam.resource.types import Model, ModelFamily
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import ResourceName
from viam.resource.base import ResourceBase
from viam.resource.registry import Registry, ResourceCreatorRegistration


class RosSensor(Sensor, Reconfigurable):
    MODEL: ClassVar[Model] = Model(ModelFamily('viam-soleng', 'noetic'), 'sensor')
    ros_topic: str
    ros_msg_pkg: str
    ros_msg_type: str
    msg: Any

    @classmethod
    def new(cls, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]) -> Self:
        pass

    @classmethod
    def validate_config(cls, config: ComponentConfig) -> Sequence[str]:
        return []

    def reconfigure(self, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase])-> None:
        pass

    async def get_readings(
        self,
        *,
        extra: Optional[Mapping[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs
    ) -> Mapping[str, Any]:
        return {}


Registry.register_resource_creator(
    Sensor.SUBTYPE,
    RosSensor.MODEL,
    ResourceCreatorRegistration(RosSensor.new, RosSensor.validate_config)
)