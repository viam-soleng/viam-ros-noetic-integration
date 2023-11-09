from .base_filter import BaseFilter
from viam.components.sensor import Sensor

class AlwaysTrue(BaseFilter):
    def should_filter(self, sensor: Sensor) -> bool:
        return True

class AlwaysFalse(BaseFilter):
    def should_filter(self, sensor: Sensor) -> bool:
        return False
