#!/usr/bin/env python3

import rospy

from spot_tools_tests_common.test_class import TestClass
from spot_tools_tests_common.asa_mock_utils import get_mock_tf_frame

from asa_ros_msgs.msg import FoundAnchor
import tf2_ros as tf
from geometry_msgs.msg import TransformStamped

class AnchorRotatorTests(TestClass):
    def __init__(self):
        super(AnchorRotatorTests, self).__init__('anchor_rotator_tests', True)

        self.pub = rospy.Publisher('/asa_ros/found_anchor', FoundAnchor, queue_size=1)
        rospy.Subscriber('/anchor_rotated_created', FoundAnchor, self.should_receive_new_frame)
        
        self.register_test_handle("AnchorRotation_ShouldBroadcastNewAnchor", "anchor_rotator_node did not boradcast a new anchor in time!", self.should_broadcast_new_frame)
        
        # Give the target node some time to spawn
        rospy.sleep(1)

    # Test 1
    def should_broadcast_new_frame(self):

        # Create a fake anchor
        t = get_mock_tf_frame()

        self.test_data[0]['expected_anchor_id'] = t.child_frame_id + "_rot"

        self.tf_static_broadcaster.sendTransform(t)

        # Inform target node of the anchor creation
        msg = FoundAnchor()
        msg.anchor_in_world_frame = t
        msg.anchor_id = t.child_frame_id
        self.pub.publish(msg)
        rospy.loginfo("published anchor")

    def should_receive_new_frame(self, anchor):
        self.advance_test(0, self.test_data[0]['expected_anchor_id'] == anchor.anchor_id)
    

if __name__ == "__main__":
    tester = AnchorRotatorTests()
    tester.run()