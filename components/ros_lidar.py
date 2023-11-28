from PIL import Image
import rospy
import viam
from threading import Lock
from typing import ClassVar, Mapping, Optional, Sequence, Tuple, List
from typing_extensions import Self
from viam.components.camera import Camera
from viam.module.types import Reconfigurable
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import ResourceName
from viam.resource.base import ResourceBase
from viam.resource.registry import Registry, ResourceCreatorRegistration
from viam.resource.types import Model, ModelFamily
from viam.utils import ValueTypes
from sensor_msgs.msg import LaserScan


class RosLidar(Camera, Reconfigurable):
    MODEL: ClassVar[Model] = Model(ModelFamily('viam-soleng', 'noetic'), 'lidar')

    ros_topic: str
    lock: Lock
    scan: LaserScan

    @classmethod
    def new(cls, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]) -> Self:
        camera = cls(config.name)
        camera.reconfigure(config, dependencies)
        return camera

    @classmethod
    def validate_config(cls, config: ComponentConfig) -> Sequence[str]:
        topic = config.attributes.fields['ros_topic'].string_value
        if topic == '':
            raise Exception('ros_topic required')
        return []

    def reconfigure(self, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]) -> None:
        self.ros_topic = config.attributes.fields['ros_topic'].string_value
        rospy.Subscriber(self.ros_topic, LaserScan, self.subscriber_callback)
        self.lock = Lock()

    def subscriber_callback(self, image: ROSImage) -> None:
        self.image = image

    async def get_image(self, mime_type: str = '', timeout: Optional[float] = None, **kwargs) -> Image:
        raise NotImplementedError()

    async def get_images(
            self,
            *,
            timeout: Optional[float] = None,
            **kwargs
    ) -> Tuple[List[viam.media.video.NamedImage], viam.proto.common.ResponseMetadata]:
        raise NotImplementedError()

    async def get_point_cloud(self, *, timeout: Optional[float] = None, **kwargs) -> Tuple[bytes, str]:
        with self.lock:
            scan = self.scan

        if scan is None:
            return None
        else:
            # TODO: convert to pcd
            return ''

    async def get_properties(self, *, timeout: Optional[float] = None, **kwargs) -> Camera.Properties:
        return self.props

    async def do_commands(
            self,
            command: Mapping[str, ValueTypes],
            *,
            timeout: Optional[float] = None,
            **kwargs
    ) -> Mapping[str, ValueTypes]:
        raise NotImplementedError()


Registry.register_resource_creator(
    Camera.SUBTYPE,
    RosLidar.MODEL,
    ResourceCreatorRegistration(RosLidar.new, RosLidar.reconfigure)
)
