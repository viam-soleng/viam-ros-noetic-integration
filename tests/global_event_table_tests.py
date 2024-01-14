import os
import unittest

from datetime import datetime as dt
from filtering.cache.global_event_table import (
    get_active_events,
    get_event_table,
    get_end_time,
    get_start_time,
    start_event,
    stop_event,
    __CACHE_NAME__,
    __CACHE_BASE_DIR__
)


class TestEventTable(unittest.TestCase):
    def setUp(self):
        self.table = get_event_table()
    def test_env_variable(self):
        self.assertEqual(os.environ['VIAM_MODULE_DIR'], __CACHE_BASE_DIR__)

    def test_table_directory(self):
        self.assertEqual(f'{os.environ["VIAM_MODULE_DIR"]}/{__CACHE_NAME__}', self.table.directory)

    def test_add_some_event(self):
        start_event({'name': 'demo', 'start': dt.now()})
        #self.assertEqual(len(self.table), 1)
        self.assertLessEqual(get_start_time(), dt.now())

    def test_stop_some_event(self):
        stop_event({'name': 'demo', 'end': dt.now()})
        self.assertGreaterEqual(get_end_time(), dt.now())


if __name__ == '__main__':
    unittest.main()
