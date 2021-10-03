#!/usr/bin/env python3

import rospy
import sys

from spot_tools_tests_common.test_class import TestClass
from asa_ros_msgs.msg import FoundAnchor
import tf2_ros as tf
from geometry_msgs.msg import TransformStamped, PoseStamped
from spot_tools_tests_common.asa_mock_utils import get_mock_tf_frame
from std_msgs.msg import String
class CommandRepublisherTests(TestClass):
    def __init__(self):
        super(CommandRepublisherTests, self).__init__('command_republisher_tests', True)

        self.command_pub = rospy.Publisher('/spot/command/pose', PoseStamped, queue_size=10)
        rospy.Subscriber('/spot/command/processed', PoseStamped, self.received_processed_cmd)
        rospy.Subscriber('/spot/command/feedback', String, self.command_feedback_received)
        self.register_test_handle("MissingAnchorCmd", "Should receive failed cmd feedback", self.test_missin_anchor_cmd)
        self.register_test_handle("RepublishCmdRelative", "Command was not republished correctly", self.test_republish_cmd)

        rospy.sleep(1)

##########
# TEST 0 #
##########

    def test_missin_anchor_cmd(self):

        # publish a command
        cmd = PoseStamped()
        cmd.header.frame_id = "non_existing_anchor_id_rot"
        cmd.pose.position.x = 0
        cmd.pose.position.y = 1
        cmd.pose.orientation.w = 1
        self.command_pub.publish(cmd)

    def command_feedback_received(self, data):
        if self.test_idx == 0:
            if data.data == "Fail; Could not find anchor":
                self.advance_test(0, True)

##########
# TEST 1 #
##########

    # validates that the command republisher node correctly processes commands
    # it should take the command from anchor -> target pose, and re-calculate the command from 
    # robot base -> target
    def test_republish_cmd(self):
        # publish demo command


        ''' (up = x, right = y)

        |__ (1,0,0)       |__ (1,1,0)
        test_anchor_rot   command
          
        |__ (0,0,0)       |__ (0,1,0)
        odom              body
        
        
        
        '''

        # mock asa anchor
        t = get_mock_tf_frame()
        t.header.frame_id = "odom"
        t.child_frame_id = "test_anchor_rot"
        self.tf_static_broadcaster.sendTransform(t)

        # mock robot body base
        body = get_mock_tf_frame()
        body.header.frame_id = "odom"
        body.child_frame_id = "body"
        body.transform.translation.x = 0
        body.transform.translation.y = 1
        self.tf_static_broadcaster.sendTransform(body)

        self.test_data[0]['expected_delta'] = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
        
        # publish a command
        cmd = PoseStamped()
        cmd.header.frame_id = "test_anchor_rot"
        cmd.pose.position.x = 0
        cmd.pose.position.y = 1
        cmd.pose.orientation.w = 1
        self.command_pub.publish(cmd)

        rospy.loginfo("Published command!")

    def received_processed_cmd(self, data):
        success = data.pose.position.x == 1 and data.pose.position.y == 0
        self.advance_test(1, success)

if __name__ == "__main__":
    tester = CommandRepublisherTests()
    tester.run()