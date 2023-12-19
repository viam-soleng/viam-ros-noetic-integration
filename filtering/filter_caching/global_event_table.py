"""
module: Global Event Table

The global event table is responsible for storing all active events in a persistent cache
"""
import os
from datetime import datetime as dt, timedelta
from diskcache import Cache
from logging import Logger
from threading import Lock
from typing import Dict, List, Union
from viam.logging import getLogger

# global variables for name of cache
__CACHE_NAME__: str = 'global_event_table'
__CACHE_BASE_DIR__: str = os.environ['VIAM_MODULE_DIR']

# cache data
__cache: Cache = Cache(f'{__CACHE_BASE_DIR__}/{__CACHE_NAME__}')        # cache of events
__cache_window: int = 20                                                # window in seconds
__lock: Lock = Lock()                                                   # lock for updating private variables

# date/time data
__start_time: Union[dt, None] = None
__end_time: Union[dt, None] = None

# logger
__logger: Logger = getLogger(__name__)


def get_event_table() -> Cache:
    """
    return the event table as needed

    :return:
    """
    return __cache

def get_active_events() -> List[str]:
    """
    return all active events in the cache as a set of keys

    :return:
    """
    keys = []

    __lock.acquire()
    for key in __cache:
        e = __cache[key]['end']
        if e is None or e < __end_time:
            keys.append(key)
        else:
            __logger.info(f'removing old stopped event: {key}')
            del(__cache[key])

    __lock.release()
    return keys

def start_event(event: Dict) -> bool:
    """

    :param event:
    :return:
    """
    global __start_time
    __lock.acquire()

    if __start_time is None or event['start'] < __start_time:
        __logger.debug(f'{event["name"]} starting earlier than earliest event, updating time')
        __start_time = event['start'] - timedelta(seconds=__cache_window)

    __lock.release()
    return __cache.add(event['name'], event)


def stop_event(event: Dict) -> None:
    """

    :param event:
    :return:
    """
    global __end_time
    __lock.acquire()
    if event in __cache:
        if __end_time is None or event['end'] > __end_time:
            __logger.debug(f'{event["name"]} stopped later than last event, updating time')
            __end_time = event['end'] + timedelta(seconds=__cache_window)
        __cache[event]['end'] = event['end']
    else:
        __logger.info(f'{event} was not found in cache(sz:{len(__cache)})')
    __lock.release()

def get_start_time() -> dt:
    """

    :return:
    """
    return __start_time

def get_end_time() -> dt:
    """

    :return:
    """
    return __end_time
