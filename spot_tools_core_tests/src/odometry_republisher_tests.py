
import rospy
import sys

from spot_tools_tests_common.test_class import TestClass
from asa_ros_msgs.msg import FoundAnchor
import tf2_ros as tf
from geometry_msgs.msg import TransformStamped, PoseStamped
from spot_tools_tests_common.asa_mock_utils import get_mock_tf_frame
from nav_msgs.msg import Odometry

class OdometryRepublisherTests(TestClass):
    def __init__(self):
        super(OdometryRepublisherTests, self).__init__('command_republisher_tests', True)

        rospy.Subscriber('spot/odometry/asa_relative', Odometry, self.received_relative_odom)
        self.spot_odom_pub = rospy.Publisher('spot/odometry', Odometry, queue_size=1)
        self.asa_found_pub = rospy.Publisher('asa_ros/found_anchor', FoundAnchor, queue_size=1)

        self.register_test_handle("AnchorIdCollection", "should not republish since no anchor exists", self.test_missing_anchor_odom)
        self.register_test_handle("RelativeOdomSingleAnchor", "Should have published odom relative to the only anchor", self.test_single_anchor_republish)
        self.register_test_handle("RelativeOdomClosestAnchor", "Should have published odom relative to the closest anchor", self.test_closest_anchor_republish)
        rospy.sleep(1)

    def test_missing_anchor_odom(self):
        self.spot_odom_pub.publish(Odometry())
        rospy.sleep(2)
        self.advance_test(0, True)

    def test_single_anchor_republish(self):
        # Create a single rotated anchor

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
        found_anchor_msg = FoundAnchor()
        found_anchor_msg.anchor_id = t.child_frame_id
        found_anchor_msg.anchor_in_world_frame = t
        self.asa_found_pub.publish(found_anchor_msg)

        # mock robot body base
        body = get_mock_tf_frame()
        body.header.frame_id = "odom"
        body.child_frame_id = "body"
        body.transform.translation.x = 0
        body.transform.translation.y = 1
        self.tf_static_broadcaster.sendTransform(body)
        
        rospy.sleep(0.5)

        # send robot odometry
        # The actual position does not matter at all - it looks up the tf 
        odom = Odometry()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "body"
        self.spot_odom_pub.publish(odom)

    def test_closest_anchor_republish(self):
        # mock asa anchor at the robot's body position
        t = get_mock_tf_frame()
        t.header.frame_id = "odom"
        t.child_frame_id = "test_anchor_2_rot"
        t.transform.translation.x = 0
        t.transform.translation.y = 1
        self.tf_static_broadcaster.sendTransform(t)
        found_anchor_msg = FoundAnchor()
        found_anchor_msg.anchor_id = t.child_frame_id
        found_anchor_msg.anchor_in_world_frame = t
        
        self.asa_found_pub.publish(found_anchor_msg)

        rospy.sleep(0.5)

        odom = Odometry()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "body"
        self.spot_odom_pub.publish(odom)



    def received_relative_odom(self, odom):
        if self.test_idx == 0:
            # Should not receive anything here in this test case
            self.advance_test(0, False)
            return

        if self.test_idx == 1:
            success = odom.pose.pose.position.x == -1 and odom.pose.pose.position.y == 1
            self.advance_test(1, success)
            return

        if self.test_idx == 2:
            success = odom.pose.pose.position.x == 0 and odom.pose.pose.position.y == 0
            self.advance_test(2, success)
            return


if __name__ == "__main__":
    tester = OdometryRepublisherTests()
    tester.run()