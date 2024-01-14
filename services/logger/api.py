import abc
from typing import Final

from grpclib.client import Channel
from grpclib.server import Stream

from viam.resource.rpc_service_base import ResourceRPCServiceBase
from viam.resource.types import RESOURCE_TYPE_SERVICE, Subtype
from viam.services.service_base import ServiceBase

from proto.noetic_logger_grpc import NoeticLoggerServiceBase, NoeticLoggerServiceStub
from proto.noetic_logger_pb2 import LoggerRequest, LoggerResponse


class NoeticLoggerService(ServiceBase):
    SUBTYPE: Final = Subtype("viam-soleng", RESOURCE_TYPE_SERVICE, "logger")

    @abc.abstractmethod
    async def status(self) -> LoggerResponse:
        ...


class NoeticLoggerRPCService(NoeticLoggerServiceBase, ResourceRPCServiceBase):
    RESOURCE_TYPE = NoeticLoggerService

    async def Status(self, stream: Stream[LoggerRequest, LoggerResponse]) -> None:
        request = await stream.recv_message()
        assert request is not None
        name = request.name
        service = self.get_resource(name)
        resp = await service.status()
        await stream.send_message(resp)


class NoeticLoggerClient(NoeticLoggerService):
    def __init__(self, name: str, channel: Channel) -> None:
        self.channel = channel
        self.client = NoeticLoggerServiceStub(channel)
        super().__init__(name)
