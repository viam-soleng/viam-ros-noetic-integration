import asyncio
import sys


async def main(addr: str) -> None:
    pass


if __name__ == '__main__':
    if len(sys.argv) != 2:
        raise Exception('need socket path as cmd line arg')
    asyncio.run(main(sys.argv[1]))
