import importlib
from threading import Lock
from typing import ClassVar, Mapping, Sequence

import rospy
from rosgraph_msgs.msg import Log
from typing_extensions import Self
from viam.module.types import Reconfigurable
from viam.proto.app.robot import ServiceConfig
from viam.resource.base import ResourceBase, ResourceName
from viam.resource.types import Model, ModelFamily
from components.ros_sensor import RosSensor

from .api import NoeticFilterService

class MyNoeticFilterService(NoeticFilterService, Reconfigurable):
    MODEL: ClassVar[Model] = Model(ModelFamily('viam-soleng', 'noetic'), 'filter')

    event_sensor: RosSensor
    event_type: str
    event_trigger_type: str
    event_trigger_obj: object
    lock: Lock

    def __init__(self, name: str):
        super().__init__(name)

    @classmethod
    def new(
        cls, config: ServiceConfig, dependencies: Mapping[ResourceName, ResourceBase]
    ) -> Self:
        service = cls(config.name)
        service.reconfigure(config, dependencies)
        return service

    @classmethod
    def validate_config(cls, config: ServiceConfig) -> Sequence[str]:
        event_type = config.attributes.fields['event_type'].string_value
        event_trigger_type = config.attributes.fields['event_trigger_type'].string_value
        event_trigger_package = config.attributes.fields['event_trigger_package'].string_value
        event_trigger_cls = config.attributes.fields['event_trigger_cls'].string_value
        event_sensor = config.attributes.fields['sensor'].string_value

        if event_type == '':
            raise Exception('event_type attribute required')

        if event_trigger_type == '':
            raise Exception('event_trigger_type attribute required, supported options: custom')

        if event_sensor == '':
            raise Exception('sensor attribute required')

        # TODO: document this
        if event_trigger_type == 'custom':
            if event_trigger_package == '':
                raise Exception('event_trigger_package attribute required with custom trigger type')
            if event_trigger_cls == '':
                raise Exception('event_trigger_cls attribute required with custom trigger type')

            try:
                tmp = importlib.import_module(event_trigger_package)
                if not hasattr(tmp, event_trigger_cls):
                    raise Exception(
                        f'invalid event trigger class, {event_trigger_package} does not have {event_trigger_cls}'
                    )
            except ModuleNotFoundError as mnfe:
                raise Exception(f'invalid event package: {mnfe}')
        else:
            # TODO: required support for other event types -> time, etc.
            raise Exception('event_trigger_type required to be set to "custom"')


        cam_name = struct_to_dict(config.attributes).get("source")
        actual_cam = dependencies[Camera.get_resource_name(cam_name)]
        self.underlying = cast(Camera, actual_cam)

        return []

    def reconfigure(
        self, config: ServiceConfig, dependencies: Mapping[ResourceName, ResourceBase]
    ):
        self.event_type = config.attributes.fields['event_type'].string_value
        self.event_trigger_type = config.attributes.fields['event_trigger_type'].string_value
        self.event_sensor = config.attributes.fields['sensor'].string_value
        event_trigger_package = config.attributes.fields['event_trigger_package'].string_value
        event_trigger_cls = config.attributes.fields['event_trigger_cls'].string_value

        try:
            p = importlib.import_module(event_trigger_package)
            if not hasattr(p, event_trigger_cls):
                raise Exception(f'invalid, {event_trigger_package} cannot load {event_trigger_cls}')
            self.event_trigger_obj = p.__getattribute__(event_trigger_cls)
        except ModuleNotFoundError as mnfe:
            raise Exception(f'module {event_trigger_package} not found: {mnfe}')
        except AttributeError as ae:
            raise Exception(f'problem loading attribute: {ae}')

        rospy.Subscriber(self.ros_topic, Log, self.subscriber_callback)
        self.lock = Lock()

    async def status(self) -> dict:
        return {
            'event': self.event_name
        }
