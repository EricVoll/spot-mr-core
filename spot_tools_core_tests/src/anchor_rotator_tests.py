from .test_class import TestClass
import rospy
from asa_ros_msgs.msg import FoundAnchor
import tf2_ros as tf
from geometry_msgs.msg import TransformStamped

class AnchorRotatorTests(TestClass):
    def __init__(self):
        rospy.init_node('anchor_rotator_tests')
        super.__init__()


        #tf setup
        self.tf_Buffer = tf.Buffer(rospy.Duration(10))
        self.tf_listener = tf.TransformListener(self.tf_Buffer)
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.tf_static_broadcaster = tf.StaticTransformBroadcaster()

        self.register_test_handle("AnchorRotation_ShouldBroadcastNewAnchor", "anchor_rotator_node did not boradcast a new anchor in time!", self.should_broadcast_new_frame)

        # Give the target node some time to spawn
        rospy.sleep(5)

        self.run()

    # Test 1
    def should_broadcast_new_frame(self):
        pub = rospy.Publisher('asa_ros/found_anchor', FoundAnchor, queue_size=1)
        rospy.Subscriber('anchor_rotated_created', FoundAnchor, self.should_receive_new_frame)

        # Create a fake anchor
        t = TransformStamped()
        t.header.stamp.secs = rospy.Time.now().secs
        t.header.frame_id = "odom"
        t.child_frame_id = "test_anchor_1"
        t.transform.translation.x = 1
        t.transform.rotation.w = 1

        self.tf_static_broadcaster.sendTransform(t)

        # Inform target node of the anchor creation
        msg = FoundAnchor()
        msg.anchor_in_world_frame = t
        msg.anchor_id = "test_anchor_1"
        pub.publish(msg)

        # Fail the test if the subscriber callback was not quick enough
        rospy.sleep(5)
        self.advance_test(0, False)

    def should_receive_new_frame(self):
        self.advance_test(0, True)
    

if __name__ == "__main__":
    tester = AnchorRotatorTests()
    tester.run()