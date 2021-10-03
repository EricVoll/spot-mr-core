import rospy
import sys
# sys.path.insert(0, '/home/eric/catkin_ws/src/spot-mr-core/spot_tools_core_tests')

from spot_tools_tests_common.test_class import TestClass
from asa_ros_msgs.msg import FoundAnchor
import tf2_ros as tf
from geometry_msgs.msg import TransformStamped

class CommandRepublisherTests(TestClass):
    def __init__(self):
        super(CommandRepublisherTests, self).__init__('command_republisher_tests', True)

        self.register_test_handle("RepublishCmdRelative", "Command was not republished correctly", self.test_republish_cmd)

    def test_republish_cmd(self):
        pass

if __name__ == "__main__":
    tester = CommandRepublisherTests()
    tester.run()