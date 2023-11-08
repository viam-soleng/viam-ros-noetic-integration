from PIL import Image
import rospy
import viam
from threading import Lock
from typing import ClassVar, Mapping, Optional, Sequence, Tuple, List
from typing_extensions import Self
from viam.components.camera import Camera, IntrinsicParameters, DistortionParameters
from viam.module.types import Reconfigurable
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import ResourceName
from viam.resource.base import ResourceBase
from viam.resource.registry import Registry, ResourceCreatorRegistration
from viam.resource.types import Model, ModelFamily
from viam.utils import ValueTypes
from sensor_msgs.msg import Image as ROSImage
from cv_bridge import CvBridge

class RosCamera(Camera, Reconfigurable):
    MODEL: ClassVar[Model] = Model(ModelFamily('viam-soleng', 'noetic'), 'camera')

    ros_topic: str
    props: Camera.Properties
    lock: Lock
    image: ROSImage
    bridge: CvBridge

    @classmethod
    def new(cls, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]) -> Self:
        camera = cls(config.name)
        camera.props = Camera.Properties(
            supports_pcd=False,
            distortion_parameters=DistortionParameters(),
            intrinsic_parameters=IntrinsicParameters()
        )
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
        rospy.Subscriber(self.ros_topic, ROSImage, self.subscriber_callback)
        self.bridge = CvBridge()
        self.lock = Lock()

    def subscriber_callback(self, image: ROSImage) -> None:
        self.image = image

    async def get_image(self, mime_type: str = '', timeout: Optional[float] = None, **kwargs) -> Image:
        with self.lock:
            img = self.image

        if img is None:
            return Image.new(mode='RGB', size=(250, 250))
        else:
            return Image.fromarray(self.bridge.imgmsg_to_cv2(img, desired_encoding='passthrough'))

    async def get_images(
            self,
            *,
            timeout: Optional[float] = None,
            **kwargs
    ) -> Tuple[List[viam.media.video.NamedImage], viam.proto.common.ResponseMetadata]:
        raise NotImplementedError()

    async def get_point_cloud(self, *, timeout: Optional[float] = None, **kwargs) -> Tuple[bytes, str]:
        raise NotImplementedError()

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
    RosCamera.MODEL,
    ResourceCreatorRegistration(RosCamera.new, RosCamera.reconfigure)
)


