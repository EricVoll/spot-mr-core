import rospy
from geometry_msgs.msg import TransformStamped

def get_mock_tf_frame():
    # Create a fake anchor
    t = TransformStamped()
    t.header.stamp.secs = rospy.Time.now().secs
    t.header.frame_id = "odom"
    t.child_frame_id = "test_anchor_" + str(t.header.stamp.secs)
    t.transform.translation.x = 1
    t.transform.rotation.w = 1
    return t
