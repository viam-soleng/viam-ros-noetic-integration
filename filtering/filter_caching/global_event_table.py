"""
module: Global Event Table

The global event table is responsible for storing all active events in a persistent cache

Every event will be stored in the table using:

[key:event_name] -> {
 name: 'event_name',
 start: start_time - cache_window
 end: stop_time + cache_window
}

The start time must be set before the event is added to the cache table, and the stop time
must exist when stop_event is called

Though diskcache gives the feels like a python dictionary, we cannot update the disk-backed
dictionaries using d[event_name][k] = v
We will retrieve the event, update it in memory and set the new event

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
__CACHE_BASE_DIR__: str = os.environ['CACHE_DIR']
__DEFAULT_CACHE_WINDOW__: int = 20
# cache data
__cache: Cache = Cache(f'{__CACHE_BASE_DIR__}/{__CACHE_NAME__}')        # cache of events
__cache_window: int = __DEFAULT_CACHE_WINDOW__                          # window in seconds
__lock: Lock = Lock()                                                   # lock for updating private variables

# date/time data
__start_time: Union[dt, None] = None
__end_time: Union[dt, None] = None

# logger
__logger: Logger = getLogger(__name__)

# try to set up new cache window size
try:
    __cache_window = int(os.environ['CACHE_WINDOW'])
except KeyError:
    __logger.warning(f'no CACHE_WINDOW environment value, falling back to default({__DEFAULT_CACHE_WINDOW__})')
    __cache_window = __DEFAULT_CACHE_WINDOW__
except ValueError:
    __logger.warning(f'CACHE_WINDOW is not a valid int, falling back to default({__DEFAULT_CACHE_WINDOW__})')
    __cache_window = __DEFAULT_CACHE_WINDOW__


def load_initial_dates() -> None:
    """
    attempt to load dates from cache
    # TODO: need to test this logic out
    :return:
    """
    global __start_time
    global __end_time

    if __start_time is None:
        try:
            __start_time = __cache['__start_time']
        except KeyError:
            __logger.warning(f'start time not found in cache')
            __start_time = None

    if __end_time is None:
        try:
            __end_time = __cache['__end_time']
        except KeyError:
            __logger.warning(f'end time not found in cache')
            __end_time = None


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
        if key == '__start_time' or key == '__end_time':
            continue
        __logger.info(f'get_active_events() -> {key}: {__cache[key]}')
        e = None
        if 'end' in __cache[key]:
            e = __cache[key]['end']

        if e is None or e < __end_time:
            keys.append(key)
        else:
            __logger.info(f'removing old stopped event: {key}')
            del(__cache[key])

    __lock.release()
    __logger.info(f'returning active events: {keys}')
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
        # every time we update start time, load it in the cache as well
        __cache['__start_time'] = __start_time

    __lock.release()
    __logger.info(f'added event {event["name"]}')
    return __cache.add(event['name'], event)


def stop_event(event: Dict) -> None:
    """

    :param event:
    :return:
    """
    global __end_time
    __lock.acquire()
    if event['name'] in __cache:
        name = event['name']
        # get existing event
        existing_event = __cache[name]
        existing_event['end'] = event['end']

        # do we update event
        if __end_time is None or event['end'] > __end_time:
            __logger.debug(f'{event["name"]} stopped later than last event, updating time')
            __end_time = event['end'] + timedelta(seconds=__cache_window)
            # every time we update start time, load it in the cache as well
            __cache['__end_time'] = __end_time
        __cache[name] = existing_event
    else:
        # this is not ideal as the ros_sensor will call stop_event often right now
        __logger.debug(f'{event["name"]} was not found in cache(sz:{len(__cache)})')
    __logger.info(f'processed stop event for: {event["name"]}')
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
