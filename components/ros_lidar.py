import math
import numpy as np
import rospy
import viam

from PIL import Image
from threading import Lock
from typing import ClassVar, Mapping, Optional, Sequence, Tuple, List
from typing_extensions import Self
from viam.components.camera import Camera, DistortionParameters, IntrinsicParameters
from viam.media.video import CameraMimeType
from viam.module.types import Reconfigurable
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import ResourceName
from viam.resource.base import ResourceBase
from viam.resource.registry import Registry, ResourceCreatorRegistration
from viam.resource.types import Model, ModelFamily
from viam.utils import ValueTypes
from sensor_msgs.msg import LaserScan

# PCD required header information
VERSION = 'VERSION .7\n'
FIELDS = 'FIELDS x y z\n'
SIZE = 'SIZE 4 4 4\n'
TYPE_OF = 'TYPE F F F\n'
COUNT = 'COUNT 1 1 1\n'
HEIGHT = 'HEIGHT 1\n'
VIEWPOINT = 'VIEWPOINT 0 0 0 1 0 0 0\n'
DATA = 'DATA binary\n'


class RosLidar(Camera, Reconfigurable):
    MODEL: ClassVar[Model] = Model(ModelFamily('viam-soleng', 'noetic'), 'lidar')

    ros_topic: Optional[str]
    ros_lidar_props: Optional[Camera.Properties]
    lock: Lock
    msg: Optional[LaserScan]

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
        self.msg = None
        self.ros_lidar_props = None
        self.ros_topic = None

    def reconfigure(self, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]) -> None:
        self.ros_topic = config.attributes.fields['ros_topic'].string_value
        self.ros_lidar_props = Camera.Properties(
            supports_pcd=True,
            intrinsic_parameters=IntrinsicParameters(
                width_px=0, height_px=0, focal_x_px=0.0, focal_y_px=0.0, center_x_px=0.0
            ),
            distortion_parameters=DistortionParameters(model='')
        )
        rospy.Subscriber(self.ros_topic, LaserScan, self.subscriber_callback)
        self.lock = Lock()

    def subscriber_callback(self, msg: LaserScan) -> None:
        self.msg = msg

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
            msg = self.msg

        if msg is None:
            raise Exception('LaserScan msg not ready')
        pdata = []
        for i, r in enumerate(msg.ranges):
            if r < msg.range_min or r > msg.range_max:
                continue
            ang = msg.angle_min + (float(i) * msg.angle_increment)
            x = math.cos(ang) * r
            y = math.sin(ang) * r
            pdata.append(x)
            pdata.append(y)
            pdata.append(float(0))

        width = f'WIDTH {len(pdata)}\n'
        points = f'POINTS {len(pdata)}\n'
        header = f'{VERSION}{FIELDS}{SIZE}{TYPE_OF}{COUNT}{width}{HEIGHT}{VIEWPOINT}{points}{DATA}'
        a = np.array(pdata, dtype='f')
        h = bytes(header, 'UTF-8')
        return h + a.tobytes(), CameraMimeType.PCD

    async def get_properties(self, *, timeout: Optional[float] = None, **kwargs) -> Camera.Properties:
        return self.ros_lidar_props

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
    ResourceCreatorRegistration(RosLidar.new, RosLidar.validate_config)
)
