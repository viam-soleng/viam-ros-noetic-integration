"""
ros_camera.py

The ros camera supports
- sensor_msgs.msg.ROSImage
- sensor_msgs.msg.CompressedImage
"""
from PIL import Image
import cv2
import numpy as np
import rospy
import viam
from logging import Logger
from threading import Lock
from typing import ClassVar, List, Mapping, Optional, Sequence, Tuple, Union
from typing_extensions import Self
from viam.components.camera import Camera, IntrinsicParameters, DistortionParameters
from viam.logging import getLogger
from viam.module.types import Reconfigurable
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import ResourceName
from viam.resource.base import ResourceBase
from viam.resource.registry import Registry, ResourceCreatorRegistration
from viam.resource.types import Model, ModelFamily
from viam.utils import ValueTypes
from sensor_msgs.msg import Image as ROSImage, CompressedImage
from cv_bridge import CvBridge


class RosCamera(Camera, Reconfigurable):
    MODEL: ClassVar[Model] = Model(ModelFamily('viam-soleng', 'noetic'), 'camera')
    ros_topic: str
    props: Camera.Properties
    lock: Lock
    image: Union[ROSImage, CompressedImage]
    bridge: CvBridge
    is_compressed: bool
    image: Optional[ROSImage]
    logger: Logger

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

    def __init__(self, name: str) -> None:
        super().__init__(name)
        self.lock = Lock()
        self.bridge = CvBridge()
        self.image = None
        self.logger = getLogger(name)

    def reconfigure(self, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]) -> None:
        """
        reconfigure ros camera

        TODO: add camera info/description topics as needed
        """
        self.logger.info('reconfigure ros camera')
        self.props = Camera.Properties(
            supports_pcd=False,
            distortion_parameters=DistortionParameters(),
            intrinsic_parameters=IntrinsicParameters()
        )
        self.ros_topic = config.attributes.fields['ros_topic'].string_value
        use_compressed = config.attributes.fields['compressed'].string_value

        dtype = ROSImage
        self.is_compressed = False
        if use_compressed.lower() == 'true':
            self.is_compressed = True
            dtype = CompressedImage

        rospy.Subscriber(self.ros_topic, dtype, self.subscriber_callback)

    def subscriber_callback(self, image: Union[ROSImage, CompressedImage]) -> None:
        """
        set image to ROS image
        """
        self.image = image

    async def get_image(self, mime_type: str = '', timeout: Optional[float] = None, **kwargs) -> Image:
        """
        convert to viam image
        """
        with self.lock:
            img = self.image

        if img is None:
            return Image.new(mode='RGB', size=(250, 250))
        else:
            if self.is_compressed:
                return Image.fromarray(cv2.imdecode(np.fromstring(img.data, np.uint8), cv2.IMREAD_COLOR))
            else:
                return Image.fromarray(self.bridge.imgmsg_to_cv2(img, desired_encoding='passthrough'))

    async def get_images(
            self,
            *,
            timeout: Optional[float] = None,
            **kwargs
    ) -> Tuple[List[viam.media.video.NamedImage], viam.proto.common.ResponseMetadata]:
        """

        """
        self.logger.warning('not implemented')
        raise NotImplementedError()

    async def get_point_cloud(self, *, timeout: Optional[float] = None, **kwargs) -> Tuple[bytes, str]:
        self.logger.warning('not implemented')
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
        self.logger.warning('not implemented')
        raise NotImplementedError()


Registry.register_resource_creator(
    Camera.SUBTYPE,
    RosCamera.MODEL,
    ResourceCreatorRegistration(RosCamera.new, RosCamera.validate_config)
)
