import rospy
import sys
sys.path.insert(0, '/home/eric/catkin_ws/src/spot-mr-core/spot_tools_core_tests')

from src.test_class import TestClass
from asa_ros_msgs.msg import FoundAnchor
import tf2_ros as tf
from geometry_msgs.msg import TransformStamped

class AnchorRotatorTests(TestClass):
    def __init__(self):
        rospy.init_node('anchor_rotator_tests')
        super(AnchorRotatorTests, self).__init__()


        #tf setup
        self.tf_Buffer = tf.Buffer(rospy.Duration(10))
        self.tf_listener = tf.TransformListener(self.tf_Buffer)
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.tf_static_broadcaster = tf.StaticTransformBroadcaster()

        self.pub = rospy.Publisher('/asa_ros/found_anchor', FoundAnchor, queue_size=1)
        rospy.Subscriber('/anchor_rotated_created', FoundAnchor, self.should_receive_new_frame)
        
        self.register_test_handle("AnchorRotation_ShouldBroadcastNewAnchor", "anchor_rotator_node did not boradcast a new anchor in time!", self.should_broadcast_new_frame)
        # Give the target node some time to spawn
        rospy.sleep(1)

    # Test 1
    def should_broadcast_new_frame(self):

        # Create a fake anchor
        t = TransformStamped()
        t.header.stamp.secs = rospy.Time.now().secs
        t.header.frame_id = "odom"
        t.child_frame_id = "test_anchor_" + str(t.header.stamp.secs)
        t.transform.translation.x = 1
        t.transform.rotation.w = 1

        self.test_data[0]['expected_anchor_id'] = t.child_frame_id + "_rot"

        self.tf_static_broadcaster.sendTransform(t)

        # Inform target node of the anchor creation
        msg = FoundAnchor()
        msg.anchor_in_world_frame = t
        msg.anchor_id = t.child_frame_id
        self.pub.publish(msg)
        rospy.loginfo("published anchor")

        # Fail the test if the subscriber callback was not quick enough
        rospy.sleep(5)
        self.advance_test(0, False)

    def should_receive_new_frame(self, anchor):
        self.advance_test(0, self.test_data[0]['expected_anchor_id'] == anchor.anchor_id)
    

if __name__ == "__main__":
    tester = AnchorRotatorTests()
    tester.run()