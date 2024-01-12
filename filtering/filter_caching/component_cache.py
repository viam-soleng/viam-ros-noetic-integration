"""
module: component cache

each component has the ability to create a disk based cache

TODO: make ViAM_MODULE_DATA variable
"""
import os
from .global_event_table import get_active_events, get_cache_window
from datetime import datetime as dt, timedelta
from diskcache import Deque
from logging import Logger
from typing import Dict, Union
from viam.logging import getLogger

class CachedItem(object):
    """

    """
    def __init__(self, to_cache: Dict) -> None:
        """

        :param to_cache:
        """
        self._item = to_cache
        self._item['v_metadata'] = {
            'cache_insert': dt.now().timestamp()
        }

    def get_data(self) -> Dict:
        """

        :return:
        """
        return self._item

    def get_cache_time(self) -> dt:
        """

        :return:
        """
        return dt.fromtimestamp(self._item['v_metadata']['cache_insert'])


class ComponentCache(object):
    """

    """
    def __init__(self, component_name: str) -> None:
        """

        :param component_name:
        """
        cache_dir = os.environ['CACHE_DIR']
        self.__logger: Logger = getLogger(f'{component_name}_cache')
        self.__queue = Deque(directory=f'{cache_dir}/{component_name}')

    def get_data(self) -> Union[Dict, None]:
        """
        Attempt to return data from the queue, if data does not exist return None

        :return:
        """
        try:
            peek = self.__queue.peekleft()
            ae = get_active_events(peek.get_cache_time())
            if len(ae) != 0:
                ci = self.__queue.popleft()
                # fix this make cache item subscriptable
                data_item = ci.get_data()
                data_item['v_metadata']['events'] = ae
                return data_item
            # if we are here there is nothing to return
            if peek.get_cache_time() < (dt.now() - timedelta(seconds=get_cache_window())):
                i = self.__queue.popleft()
                self.__logger.info(f'removing element from cache: {i}')
            return None
        except IndexError as ie:
            self.__logger.debug(f'get_data(): cache is empty: {ie}')
            return None


    def add_data(self, item) -> None:
        """
        Attempt to add the data item to the queue
        Nothing is returned at this time

        :param item:
        :return:
        """
        ci = CachedItem(item)
        self.__queue.append(ci)

