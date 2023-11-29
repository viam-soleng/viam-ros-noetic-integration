import rospy
from threading import Thread
from viam.logging import getLogger

class RospyManager:
    """

    """
    __initialized = False
    mgr = None
    @classmethod
    def get_mgr(cls):
        """

        :return:
        """
        if not cls.__initialized:
            rospy.init_node('viamnode')
            cls.__initialized = True
            mgr = RospyManager()

        return mgr


    def __init__(self):
        self.logger = getLogger(__name__)
        self.logger.info('starting manager')
        self.executor_thread = None
        self.is_spinning = False

    def spin_node(self):
        if not self.is_spinning:
            self.logger.debug('starting rospy spin thread')
            self.executor_thread = Thread(target=rospy.spin, daemon=True)
            self.executor_thread.start()

    def shutdown(self):
        self.logger.info('attempting to shutdown rospy spin thread')
        rospy.signal_shutdown()
        self.executor_thread.join()
