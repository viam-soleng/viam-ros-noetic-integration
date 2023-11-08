import abc
from typing import Final

from grpclib.client import Channel
from grpclib.server import Stream

from viam.resource.rpc_service_base import ResourceRPCServiceBase
from viam.resource.types import RESOURCE_TYPE_SERVICE, Subtype
from viam.services.service_base import ServiceBase

from proto.noetic_filter_grpc import NoeticFilterServiceBase, NoeticFilterServiceStub
from proto.noetic_filter_pb2 import StatusRequest, StatusResponse, FilterRequest, FilterResponse


class NoeticFilterService(ServiceBase):
    SUBTYPE: Final = Subtype("viam-soleng", RESOURCE_TYPE_SERVICE, "filter")

    @abc.abstractmethod
    async def status(self) -> FilterResponse:
        ...


class NoeticFilterRPCService(NoeticFilterServiceBase, ResourceRPCServiceBase):
    RESOURCE_TYPE = NoeticFilterService

    async def Status(self, stream: Stream[StatusRequest, StatusResponse]) -> None:
        request = await stream.recv_message()
        assert request is not None
        name = request.name
        service = self.get_resource(name)
        resp = await service.status()
        await stream.send_message(resp)

    async def ShouldFilter(self, stream: Stream[FilterRequest, FilterResponse]) -> None:
        request = await stream.recv_message()
        assert request is not None
        name = request.name
        service = self.get_resource(name)
        resp = await service.status()
        await stream.send_message(resp)


class NoeticFilterClient(NoeticFilterService):
    def __init__(self, name: str, channel: Channel) -> None:
        self.channel = channel
        self.client = NoeticFilterServiceStub(channel)
        super().__init__(name)
