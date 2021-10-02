
import rospy
import tf
from asa_ros_msgs.msg import FoundAnchor

import rospy
import tf2_ros as tf
from tf.transformations import *

class AnchorRotatorNode:
    def __init__(self):
        rospy.init_node('anchor_rotator')
        rospy.Subscriber('/asa_ros/found_anchor', FoundAnchor, self.asa_found_anchor_callback)
        self.pub = rospy.Publisher("/anchor_rotated_created", FoundAnchor, queue_size=10)

        #tf setup
        self.tf_Buffer = tf.Buffer(rospy.Duration(10))
        self.tf_listener = tf.TransformListener(self.tf_Buffer)
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.tf_static_broadcaster = tf.StaticTransformBroadcaster()

        self.drift_free_frame = "odom"


    def spin(self):
        rospy.spin()

    def asa_found_anchor_callback(self, data):
        rospy.loginfo(f"Found anchor {data.anchor_id} and will rotate it")
        frame_name = data.anchor_id
        self.republish_frame_rotated(frame_name)


    
    # applies a constant rotation to the frame relative to odom
    def republish_frame_rotated(self, frame_name):

        rotatedFrameName = frame_name + "_rot"
        rotatedtrans = self.tf_Buffer.lookup_transform(self.drift_free_frame, frame_name, rospy.Time(0), rospy.Duration(0.1))
        rotation = rotatedtrans.transform.rotation

        quat1 = [-0.5, 0.5, 0.5, 0.5]
        quat2 = [rotation.x,rotation.y,rotation.z,rotation.w]
        quat3 = [ 0, 0, 0.7071068, 0.7071068 ]

        nr = quaternion_multiply(quat2, quat1)
        nr = quaternion_multiply(quat3, nr)

        rotatedtrans.transform.rotation.x = nr[0]
        rotatedtrans.transform.rotation.y = nr[1]
        rotatedtrans.transform.rotation.z = nr[2]
        rotatedtrans.transform.rotation.w = nr[3]
        rotatedtrans.child_frame_id = rotatedFrameName
        self.tf_static_broadcaster.sendTransform(rotatedtrans)

        anchor = FoundAnchor()
        anchor.anchor_id = rotatedFrameName
        anchor.anchor_in_world_frame = rotatedtrans
        self.pub.publish(anchor)



if __name__ == "__main__":
    node = AnchorRotatorNode()
    node.spin()