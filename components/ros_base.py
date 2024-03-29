"""
In ROS a base is typically represented as the /cmd_vel topic in the graph and uses
a twist message variant, while viam uses a Base object which supports methods that
take a linear and angular vectors.

These vectors passed from Viam code are converted to twist messages, with one caveat:
the RDK linear y is passed to the ROS Twist message as the x component in the vector

In viam only the Y component in the linear vector is used for wheeled bases, with
positive implying forward and only the Z component in the angular component is used
for wheeled bases, with positive turning left.

TODO: add attributes to support different base types, currently our publisher will
      continue to publish messages even if the twist message vectors are all 0, this
      can cause issues when we attempt to call base actions like "dock" etc.
      More testing will be needed to ensure the base works as expected
TODO: Need to test with noetic
"""
from threading import Lock
import rospy
import viam
from logging import Logger
from typing import Any, ClassVar, Dict, Mapping, Optional, Sequence, Tuple
from typing_extensions import Self
from viam.components.base import Base
from viam.logging import getLogger
from viam.module.types import Reconfigurable
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import ResourceName
from viam.resource.base import ResourceBase
from viam.resource.registry import Registry, ResourceCreatorRegistration
from viam.resource.types import Model, ModelFamily
from viam.utils import ValueTypes
from geometry_msgs.msg import Twist


class RosBase(Base, Reconfigurable):
    """
    RosBase represents a wheeled base that supports Twist Messages
    """
    MODEL: ClassVar[Model] = Model(ModelFamily('viam-soleng', 'noetic'), 'base')
    is_base_moving: bool
    lock: Lock
    publish_time: int
    ros_topic: str
    twist_msg: Twist
    publisher: rospy.Publisher
    timer: rospy.Timer
    logger: Logger

    @classmethod
    def new(cls, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]) -> Self:
        """
        new() creates a new wheeled base class
        """
        base = cls(config.name)
        base.reconfigure(config, dependencies)
        return base

    @classmethod
    def validate_config(cls, config: ComponentConfig) -> Sequence[str]:
        """
        validate_config requires:
        ros_topic: the topic to subscribe to
        publish_time: the rate at which to publish messages in seconds
        """
        topic = config.attributes.fields['ros_topic'].string_value
        publish_time = config.attributes.fields['publish_time'].string_value

        if publish_time == '':
            raise Exception('time (in seconds) required as float')
        else:
            try:
                publish_time = int(publish_time)
                if publish_time == 0:
                    raise Exception('time (in seconds) required as float')
            except ValueError as ve:
                raise Exception(f'invalid value for time (in seconds): {ve}')

        if topic == '':
            raise Exception('ros_topic required')

        if publish_time == 0.0:
            raise Exception('time (in seconds) required')

        return []

    def __init__(self, name: str) -> None:
        super().__init__(name)
        self.is_base_moving = False
        self.lock = Lock
        self.publish_time = 0
        self.ros_topic = ''
        self.twist_msg = Twist()
        self.publisher = None
        self.timer = None
        self.logger = getLogger(name)

    def ros_publisher_cb(self) -> None:
        """
        ros_publisher_cb will be called by the ros node to publish twist messages
        TODO: we will have to introduce logic here to stop publishing if
              required
        """
        self.publisher.publish(self.twist_msg)

    def reconfigure(self, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]) -> None:
        """
        reconfigure will reconfigure the ros component when the rdk configuration is updated
        this will require shutting down the publisher if it exists as well as the timer
        and resetting on possible new configurations

        :param config:
        :param dependencies:
        :return:
        """
        self.logger.info('attempting ros_base reconfigure')
        self.ros_topic = config.attributes.fields['ros_topic'].string_value
        self.publish_time = int(config.attributes.fields['publish_time'].string_value)
        self.publisher = rospy.Publisher(self.ros_topic, Twist, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(self.publish_time), self.ros_publisher_cb)
        self.is_base_moving = False

    async def move_straight(
            self,
            distance: int,
            velocity: float,
            *,
            extra: Optional[Dict[str, Any]] = None,
            timeout: Optional[float] = None,
            **kwargs
    ) -> None:
        """
        move_straight currently not implemented
        """
        raise NotImplementedError()

    async def spin(
            self,
            angle: float,
            velocity: float,
            *,
            extra: Optional[Dict[str, Any]] = None,
            timeout: Optional[float] = None,
            **kwargs
    ) -> None:
        """
        spin: currently not implemented
        """
        raise NotImplementedError()

    async def set_power(
            self,
            linear: viam.components.base.Vector3,
            angular: viam.components.base.Vector3,
            *,
            extra: Optional[Dict[str, Any]] = None,
            timeout: Optional[float] = None, **kwargs
    ) -> None:
        """
        set_power convert Viam linear and angular velocity to twist message to be published
        """
        with self.lock:
            self.twist_msg.linear.x = linear.y
            self.twist_msg.angular.z = angular.z
            self.is_base_moving = True

    async def set_velocity(
            self,
            linear: viam.components.base.Vector3,
            angular: viam.components.base.Vector3,
            *,
            extra: Optional[Dict[str, Any]] = None,
            timeout: Optional[float] = None,
            **kwargs
    ) -> None:
        """
        set_velocity convert Viam linear and angular velocity to twist message to be published
        """
        with self.lock:
            self.twist_msg.linear.x = linear.y
            self.twist_msg.angular.z = angular.z
            self.is_base_moving = True

    async def stop(
            self,
            extra: Optional[Dict[str, Any]] = None,
            timeout: Optional[float] = None,
            **kwargs
    ) -> None:
        """
        stop() will stop the base by publishing a twist message with 0's as the components for
        the linear and angular vectors
        """
        with self.lock:
            self.twist_msg.linear.x = 0.0
            self.twist_msg.angular.z = 0.0
            self.is_base_moving = False

    async def is_moving(self) -> bool:
        """
        Return if the base is moving or not
        """
        return self.is_base_moving

    async def get_properties(self, *, timeout: Optional[float] = None, **kwargs) -> Base.Properties:
        """
        return the base width and turning radius

        Currently not implemented
        """
        self.logger.warning('not implemented')
        raise NotImplementedError()

    async def do_command(
            self,
            command: Mapping[str, ValueTypes],
            *,
            timeout: Optional[float] = None,
            **kwargs
    ) -> Mapping[str, ValueTypes]:
        """
        Currently not supported
        """
        self.logger.warning('not implemented')
        raise NotImplementedError()

    async def close(self):
        """

        :return:
        """
        self.timer.shutdown()


"""
Register the new MODEL as well as define how the object is validated 
and created
"""
Registry.register_resource_creator(
    Base.SUBTYPE,
    RosBase.MODEL,
    ResourceCreatorRegistration(RosBase.new, RosBase.validate_config)
)