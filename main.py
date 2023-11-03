import asyncio
import sys

from viam.components.sensor import Sensor
from viam.logging import getLogger
from viam.module.module import Module

from components import RosSensor
from utils import RospyManager

logger = getLogger(__name__)
async def main(addr: str) -> None:
    try:
        logger.info('starting module')
        rclpy_mgr = RospyManager.get_mgr()
        rclpy_mgr.spin_node()

        m = Module(addr)
        m.add_model_from_registry(Sensor.SUBTYPE, RosSensor.MODEL)
        await m.start()
    except Exception as e:
        raise Exception(f'error occurred starting module: {e}')
    finally:
        rclpy_mgr.shutdown()


if __name__ == '__main__':
    if len(sys.argv) != 2:
        raise Exception('need socket path as cmd line arg')
    asyncio.run(main(sys.argv[1]))
