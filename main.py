"""
main entry point

"""
import asyncio
import sys

from viam.components.camera import Camera
from viam.components.movement_sensor import MovementSensor
from viam.components.sensor import Sensor
from viam.logging import getLogger
from viam.module.module import Module

from components import RosCamera, RosImu, RosLidar, RosSensor
from utils import RospyManager

logger = getLogger(__name__)

OK = 0
SOCKET_NEEDED_ERR = 1
MODULE_START_ERR = 2

async def main(addr: str) -> None:
    failed = OK
    logger.info('starting module')
    rclpy_mgr = RospyManager.get_mgr()
    rclpy_mgr.spin_node()

    try:
        m = Module(addr)
        m.add_model_from_registry(Camera.SUBTYPE, RosCamera.MODEL)
        m.add_model_from_registry(MovementSensor.SUBTYPE, RosImu.MODEL)
        m.add_model_from_registry(Camera.SUBTYPE, RosLidar.MODEL)
        m.add_model_from_registry(Sensor.SUBTYPE, RosSensor.MODEL)
        await m.start()
    except Exception as e:
        logger.fatal(f'error occurred starting module: {e}, exiting ({MODULE_START_ERR})')
        failed = MODULE_START_ERR
    finally:
        rclpy_mgr.shutdown()
        sys.exit(failed)


if __name__ == '__main__':
    if len(sys.argv) != 2:
        logger.fatal(f'need socket path as cmd line arg, exiting ({SOCKET_NEEDED_ERR})')
        sys.exit(SOCKET_NEEDED_ERR)
    asyncio.run(main(sys.argv[1]))
    sys.exit(OK)
