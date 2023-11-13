import logging
from threading import Lock
from typing import Any, ClassVar, Mapping, Sequence

import rospy
from rosgraph_msgs.msg import Log
from typing_extensions import Self
from viam.logging import getLogger
from viam.module.types import Reconfigurable
from viam.proto.app.robot import ServiceConfig
from viam.resource.base import ResourceBase, ResourceName
from viam.resource.types import Model, ModelFamily

from .api import NoeticLoggerService

LEVELS_P_R = {
    logging.DEBUG: rospy.DEBUG,
    logging.INFO: rospy.INFO,
    logging.WARNING: rospy.WARN,
    logging.ERROR: rospy.ERROR,
    logging.FATAL: rospy.FATAL
}

LEVELS_R_P = {
    rospy.DEBUG: logging.DEBUG,
    rospy.INFO: logging.INFO,
    rospy.WARN: logging.WARNING,
    rospy.ERROR: logging.ERROR,
    rospy.FATAL: logging.FATAL
}

class MyNoeticLoggerService(NoeticLoggerService, Reconfigurable):
    """

    """
    MODEL: ClassVar[Model] = Model(ModelFamily('viam-soleng', 'noetic'), 'logger')

    ros_topic: str
    log_level: int
    lock: Lock
    logger: logging.Logger

    def __init__(self, name: str):
        """

        :param name:
        """
        super().__init__(name)

    @classmethod
    def new(
        cls, config: ServiceConfig, dependencies: Mapping[ResourceName, ResourceBase]
    ) -> Self:
        """

        :param config:
        :param dependencies:
        :return:
        """
        service = cls(config.name)
        service.reconfigure(config, dependencies)
        return service

    @classmethod
    def validate_config(cls, config: ServiceConfig) -> Sequence[str]:
        """

        :param config:
        :return:
        """
        ros_topic = config.attributes.fields['ros_topic'].string_value
        if ros_topic == '':
            raise Exception('ros_topic attribute required')
        return []

    def reconfigure(
        self, config: ServiceConfig, dependencies: Mapping[ResourceName, ResourceBase]
    ) -> None:
        """

        :param config:
        :param dependencies:
        :return:
        """
        self.ros_topic = config.attributes.fields['ros_topic'].string_value
        self.log_level = config.attributes.fields['log_level'].string_value.upper()

        try:
            self.log_level = logging.__getattribute__(self.log_level)
        except AttributeError:
            self.log_level = logging.__getattribute__('INFO')

        self.logger = getLogger(f'{__name__}.{self.__class__.__name__}')
        self.logger.setLevel(self.log_level)

        rospy.Subscriber(self.ros_topic, Log, self.subscriber_callback)
        self.lock = Lock()

    def subscriber_callback(self, ros_log: Log) -> None:
        """

        :param ros_log:
        :return:
        """
        with self.lock:
            message = f'node name: {ros_log.name}, message: {ros_log.msg}, severity level: {str(ros_log.level)}'
            self.logger.log(
                LEVELS_R_P[str(ros_log.level)], message
            ) if ros_log.level >= LEVELS_P_R[self.log_level] else None

    async def status(self) -> Mapping[str, Any]:
        """

        :return:
        """
        return {
            'ros_topic': self.ros_topic,
            'log_level': self.log_level
        }
