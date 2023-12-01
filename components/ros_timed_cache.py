from datetime import datetime as dt, timedelta
from logging import Logger
from queue import Queue
from threading import Lock, Thread, Timer
from typing import Dict, List, Union
from viam.logging import getLogger

logger: Logger = getLogger(__name__)

class RosTimedCachedItem(object):
    """
    A cached item will contain the component data as well as the
    time the data was inserted as well as any event tags
    """
    def __init__(self, to_cache: Dict) -> None:
        """

        :param to_cache:
        """
        logger.debug('inserting data')
        self._data_item = to_cache
        self._data_item['v_metadata'] = {
            'inserted_into_cache_at': dt.now()
        }

    def get_data(self):
        return self._data_item

    def get_insert_time(self):
        return self._data_item['v_metadata']['inserted_into_cache_at']

class RosFilterEvents:
    """
    static instance used by all caches

    """
    _events: List = []

    @classmethod
    def add_event(cls, event: str, start_time: dt) -> None:
        for e in cls._events:
            if e['name'] == event:
                logger.info(f'{event} is already processing ({e["s_time"]}')
                return

        cls._events.append({
            'name': event,
            's_time': start_time,
            'e_time': None
        })

    @classmethod
    def remove_event(cls, event: str) -> None:
        for e in cls._events:
            if e['name'] == event:
                cls._events.remove(e)
    @classmethod
    def stop_event(cls, event: str) -> None:
        for e in cls._event:
            if e['name'] == event:
                e['e_time'] = dt.now()

    @classmethod
    def get_events(cls) -> List:
        return list(cls._events)

    @classmethod
    def __len__(cls):
        return len(cls._events)




class RosTimedCache(object):
    """

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

    def add_item(self, item: dict):
        """

        :param item:
        :return:
        """
        cacheable_item = RosTimedCachedItem(item)
        self._insert_times.append(cacheable_item.get_insert_time())
        self._queue.put(cacheable_item)

    def get_item(self) -> Union[Dict, None]:
        insert_time = self._insert_times[-1]
        cacheable_item = self._queue.get()

        # not sure about this just yet
        if cacheable_item.get_insert_time() != insert_time:
            logger.warning('times not equal for cached item')

        data = None
        if len(RosFilterEvents) != 0:
            data = cacheable_item.get_data()
            data['v_metadata']['events'] = RosFilterEvents.get_events()
            return data
        return data

    def tag_events(self, event: str):
        """

        :param event:
        :return:
        """
        pass

    def clean_cache(self):
        pass