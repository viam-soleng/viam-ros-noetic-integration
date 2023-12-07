import rospy
import viam
from threading import Lock
from utils import quaternion_to_orientation
from typing import Any, ClassVar, Dict, Mapping, Optional, Sequence, Tuple
from typing_extensions import Self
from viam.components.movement_sensor import MovementSensor, Orientation, Vector3
from viam.module.types import Reconfigurable
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import ResourceName
from viam.resource.base import ResourceBase
from viam.resource.registry import Registry, ResourceCreatorRegistration
from viam.resource.types import Model, ModelFamily
from viam.utils import ValueTypes
from sensor_msgs.msg import Imu


class RosImuProperties(MovementSensor.Properties):
    def __init__(self):
        super().__init__(
            linear_acceleration_supported=True,
            angular_velocity_supported=True,
            orientation_supported=True,
            position_supported=False,
            compass_heading_supported=False,
            linear_velocity_supported=False
        )


class RosImu(MovementSensor, Reconfigurable):
    MODEL: ClassVar[Model] = Model(ModelFamily('viam-soleng', 'noetic'), 'imu')
    ros_topic: str
    msg: Imu
    lock: Lock
    props: RosImuProperties

    @classmethod
    def new(cls, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]) -> Self:
        imu = cls(config.name)
        imu.reconfigure(config, dependencies)
        return imu

    @classmethod
    def validate_config(cls, config: ComponentConfig) -> Sequence[str]:
        topic = config.attributes.fields['ros_topic'].string_value
        if topic == '':
            raise Exception('ros_topic required')
        return []

    def __init__(self) -> None:
        self.msg = None
        self.props = RosImuProperties()
        self.lock = Lock()

    def reconfigure(self, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]) -> None:
        self.ros_topic = config.attributes.fields['ros_topic'].string_value
        rospy.Subscriber(self.ros_topic, Imu, self.subscriber_callback)

    def subscriber_callback(self, msg: Imu) -> None:
        with self.lock:
            self.msg = msg

    async def get_position(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs
    ) -> Tuple[viam.components.movement_sensor.GeoPoint, float]:
        raise NotImplementedError()

    async def get_linear_velocity(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs
    ) -> viam.components.movement_sensor.Vector3:
        raise NotImplementedError()

    async def get_angular_velocity(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs
    ) -> viam.components.movement_sensor.Vector3:
        if self.msg is None:
            raise Exception("ros imu message not ready")
        av = self.msg.angular_velocity
        return Vector3(x=av.x, y=av.y, z=av.z)

    async def get_linear_acceleration(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs
    ) -> viam.components.movement_sensor.Vector3:
        if self.msg is None:
            raise Exception("ros imu message not ready")
        la = self.msg.linear_acceleration
        return Vector3(x=la.x, y=la.y, z=la.z)

    async def get_compass_heading(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs
    ) -> float:
        raise NotImplementedError()

    async def get_orientation(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs
    ) -> viam.components.movement_sensor.Orientation:
        if self.msg is None:
            raise Exception("ros imu message not ready")
        o = self.msg.orientation
        return quaternion_to_orientation(o.w, o.x, o.y, o.z)

    async def get_accuracy(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs
    ) -> Mapping[str, float]:
        raise NotImplementedError()

    async def get_properties(self, *, timeout: Optional[float] = None, **kwargs) -> MovementSensor.Properties:
        return self.props

    async def do_command(
        self,
        command: Mapping[str, ValueTypes],
        *,
        timeout: Optional[float] = None,
        **kwargs
    ):
        raise NotImplementedError()


Registry.register_resource_creator(
    MovementSensor.SUBTYPE,
    RosImu.MODEL,
    ResourceCreatorRegistration(RosImu.new, RosImu.validate_config)
)