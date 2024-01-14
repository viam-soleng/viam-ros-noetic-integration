import os

# set environment variable before accessing event table
# ensures system starts correctly
os.environ['VIAM_MODULE_DIR'] = './tmp'

from .global_event_table_tests import TestEventTable