from queue import LifoQueue
from threading import Lock, Thread, Timer


class RosTimedCachedItem(object):
    """
    A cached item will contain the component data as well as the
    time the data was inserted as well as any event tags
    """
    def __init__(self, to_cache: dict):
        """

        :param to_cache:
        """
        pass

    def update_tag(self, tag):
        """

        :param tag:
        :return:
        """
        pass

class RosTimedCache(object):
    """

    """
    lock: Lock
    cache: list = []

    def __init__(self):
        """

        """
        pass

    def add_item(self, item: dict):
        """

        :param item:
        :return:
        """
        pass

    def tag_events(self, event: str):
        """

        :param event:
        :return:
        """
        pass

    def clean_cache(self):
        pass