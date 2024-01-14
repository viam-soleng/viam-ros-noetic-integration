"""
This utility can be used stand-along to inspect a diskcache file
"""
import argparse
import os
import sys
from diskcache import Deque
from diskcache import Cache


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='inspect a diskcache file', usage='must pass -e <dir> or -s <dir>')
    parser.add_argument(
        '-e',
        '--event-cache',
        type=str,
        help='path to event cache directory',
        required=False,
        default=None,
        dest='event'
    )
    parser.add_argument(
        '-d'
        '--cache-dir',
        type=str,
        help='path to cache directory for events and components',
        required=False,
        default='/opt/viam-cache',
        dest='cache_dir'
    )
    parser.add_argument(
        '-s',
        '--sensor-cache',
        type=str,
        help='path to sensor cache directory',
        required=False,
        default=None,
        dest='sensor'
    )
    args = parser.parse_args()
    print(f'args => {args}')
    return args


def process_sensor_cache(loc: str) -> None:
    q = Deque(directory=loc)
    print(f'len of sensor q: {len(q)}')
    sample = q.peekleft()
    print(f'sample: {sample.get_item()}')


def process_event_cache(loc: str) -> None:
    ec = Cache(directory=loc)
    print(f'length of event cache {len(ec)}')
    for event in ec:
        print(f'event: {event}')
        print(f'\tinfo: {ec["event"]}')


def main(args: argparse.Namespace) -> None:
    """

    :return:
    """
    os.environ['CACHE_DIR'] = args.cache_dir

    if args.sensor:
        process_sensor_cache(args.sensor)
    elif args.event:
        process_event_cache(args.event)
    else:
        print('invalid')


if __name__ == "__main__":
    main(parse_args())
