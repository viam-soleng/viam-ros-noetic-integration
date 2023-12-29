"""
module: component cache

each component has the ability to create a disk based cache

TODO: make ViAM_MODULE_DATA variable
"""
import os
from .global_event_table import get_active_events
from datetime import datetime as dt
from diskcache import Deque
from logging import Logger
from typing import Dict, Union
from viam.logging import getLogger

__logger: Logger = getLogger(__name__)


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
        # todo: validate with team that this is a valid variable
        cache_dir = os.environ['CACHE_DIR']
        self.__queue = Deque(directory=f'{cache_dir}/{component_name}')

    def get_data(self) -> Union[Dict, None]:
        """

        :return:
        """
        ci = self.__queue.popleft()
        ae = get_active_events()
        if len(ae) != 0:
            ci['v_metadata']['events'] = ae
            return ci
        # event analysis
        return None

    def add_data(self, item) -> bool:
        """

        :param item:
        :return:
        """
        ci = CachedItem(item)
        self.__queue.append(ci)

