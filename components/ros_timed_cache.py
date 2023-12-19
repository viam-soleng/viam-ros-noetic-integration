from datetime import datetime as dt, timedelta
from logging import Logger
from queue import Empty, Queue
from threading import Lock, Thread, Timer
from typing import Any, Dict, List, Union
from typing_extensions import Self
from viam.logging import getLogger
from diskcache import Cache, Deque

logger: Logger = getLogger(__name__)


class RosGlobalEventTable(object):
    """

    """
    def __new__(cls):
        if not hasattr(cls, 'instance'):
            cls.instance = super(RosGlobalEventTable, cls).__new__(cls)
        return cls.instance

    def get_start_time(self):
        return self._start_time

    def get_end_time(self):
        return self._end_time



class RosTimedCachedItem(object):
    """
    A cached item will contain the component data as well as the
    time the data was inserted as well as any event tags
    """
    def __init__(self, to_cache: Dict) -> None:
        """

        :param to_cache:
        """
        logger.debug('creating cache data')
        self._data_item = to_cache
        self._data_item['v_metadata'] = {
            'inserted_into_cache_at': dt.now().timestamp()
        }

    def get_data(self):
        return self._data_item

    def get_insert_time(self):
        return self._data_item['v_metadata']['inserted_into_cache_at']

class RosFilterEvents:
    """
    static event table used by all caches to determine current events and timings

    Need to clean this up as this is not a true singleton pattern since __init__ can be
    called as well

    """
    _rfe_obj: Any = None
    _events: List
    _current_start_time: Union[dt, None]
    _current_end_time: Union[dt, None]

    @classmethod
    def new(cls) -> Self:
        """

        :return:
        """
        if cls._rfe_obj is not None:
            return cls._rfe_obj
        else:
            cls._rfe_obj = cls()
            return cls._rfe_obj

    def __init__(self) -> None:
        """

        """
        self._events = []
        self._current_start_time = None
        self._current_end_time = None

    def add_event(self, event: str, start_time: dt) -> None:
        """
        add event to list

        :param event:
        :param start_time:
        :return:
        """
        for e in self._events:
            if e['name'] == event:
                logger.info(f'{event} is already processing ({e["s_time"]}')
                return

        # add event validate times
        self._events.append({
            'name': event,
            's_time': start_time,
            'e_time': None
        })

    def remove_event(self, event: str) -> None:
        """
        TODO: who called this (threads must be managed for this)
        :param event:
        :return:
        """
        for e in self._events:
            if e['name'] == event:
                self._events.remove(e)

    def stop_event(self, event: str) -> None:
        """
        Event has left the building do we add time to the cache
        :param event:
        :return:
        """
        for e in self._event:
            if e['name'] == event:
                e['e_time'] = dt.now()

    def get_events(self) -> List:
        return list(self._events)

    def clear_events(self) -> None:
        pass

    def __len__(self):
        return len(self._events)



class RosTimedCache(object):
    """
    This cache will be created for each sensor that is being collected
    """
    def __init__(self, seconds : int =20) -> None :
        """
        Create queue of infinite size for right now
        """
        self._queue = Queue(maxsize=0)                  # infinite queue size
        self._insert_times = []                         # insert times for items (might not be needed)
        self._start_time = None                         # start time of event
        self._finish_time = None                        # finish time of event
        self._in_event = False                          # are we in an event
        self._time_to_cache = seconds                   # how long to cache item for
        self._rfe = RosFilterEvents.new()

    def add_item(self, item: dict):
        """

        :param item:
        :return:
        """
        cacheable_item = RosTimedCachedItem(item)
        self._insert_times.append(cacheable_item.get_insert_time())
        self._queue.put(cacheable_item)

    def get_item(self) -> Union[Dict, None]:

        if self._queue.empty():
            return None

        insert_time = self._insert_times[-1]
        cacheable_item = self._queue.get()

        # not sure about this just yet
        if cacheable_item.get_insert_time() != insert_time:
            logger.warning('times not equal for cached item')

        data = None
        if len(self._rfe) != 0:
            data = cacheable_item.get_data()
            data['v_metadata']['events'] = self._rfe.get_events()
            return data
        return data

    def tag_events(self, event: str):
        """

        :param event:
        :return:
        """
        pass

    def empty_cache(self):
        with self._queue.mutex:
            self._queue.get_nowait()