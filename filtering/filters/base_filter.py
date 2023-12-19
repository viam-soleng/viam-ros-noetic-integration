from abc import ABC, abstractmethod
from viam.components.sensor import Sensor
class BaseFilter(ABC):
    @abstractmethod
    def should_filter(self, sensor: Sensor) -> bool:
        ...
