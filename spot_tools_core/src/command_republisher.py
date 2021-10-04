#!/usr/bin/env python3

'''
Type: Ros-Node
Description: Listens to a topic publishing commands relative to some frame and republishes them relative to spot's body frame
Input: Commands on the topic "/command/raw"
Output: Commands on the topic "/command/processed"

'''
import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped
import tf2_ros as tf
from std_msgs.msg import String
class CommandRepublisher():
    def __init__(self):
        rospy.init_node("command_republisher")

        rospy.Subscriber("/command/raw", PoseStamped, self.received_cmd)
        self.cmd_pub = rospy.Publisher("/command/processed", PoseStamped, queue_size=1)
        self.cmd_feedback_pub = rospy.Publisher("/command/feedback", String, queue_size=10)

        #tf setup
        self.tf_Buffer = tf.Buffer(rospy.Duration(10))
        self.tf_listener = tf.TransformListener(self.tf_Buffer)
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.tf_static_broadcaster = tf.StaticTransformBroadcaster()

        self.robot_frame = "body"
        self.goal_frame_id = "current_goal"



    def received_cmd(self, cmd):
        rospy.loginfo("received command!")
        self.cmd_feedback_pub.publish("Received command")

        # Lookup cmd relative to body frame
        rospy.loginfo(cmd.header.frame_id)
        if not cmd.header.frame_id.endswith("_rot"):
            rospy.loginfo("did append")
            cmd.header.frame_id = cmd.header.frame_id + "_rot"

        _tf = self.pose_to_tf(cmd.pose, self.goal_frame_id, cmd.header.frame_id)
        self.tf_broadcaster.sendTransform(_tf)

        rospy.sleep(0.1)

        # lookup transform to odom
        try:
            trans = self.tf_Buffer.lookup_transform(self.robot_frame, self.goal_frame_id, rospy.Time(0), rospy.Duration(1))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("Failed to lookup anchor for anchored goal once.")
            self.cmd_feedback_pub.publish("Fail; Could not find anchor")
            return

        
        target_pose = PoseStamped()
        target_pose.header.frame_id = self.robot_frame
        target_pose.pose.position.x = trans.transform.translation.x
        target_pose.pose.position.y = trans.transform.translation.y
        target_pose.pose.position.z = trans.transform.translation.z
        target_pose.pose.orientation.x = trans.transform.rotation.x
        target_pose.pose.orientation.y = trans.transform.rotation.y
        target_pose.pose.orientation.z = trans.transform.rotation.z
        target_pose.pose.orientation.w = trans.transform.rotation.w

        self.cmd_pub.publish(target_pose)
        self.cmd_feedback_pub.publish("Success; Republished command")



    # transforms a post to a transform object
    def pose_to_tf(self, pose, frame_name, parent_frame, time=None):
        """
        Generate a TF from a given pose, frame, and parent.
        """
        assert pose is not None, 'Cannot have None for pose.'
        _tf = TransformStamped()
        
        _tf.child_frame_id = frame_name
        if time is None:
            time = rospy.Time.now()
        _tf.header.stamp = time
        _tf.header.frame_id = parent_frame

        _tf.transform.translation = pose.position
        _tf.transform.rotation = pose.orientation

        return _tf

    def spin(self):
        rospy.spin()

if __name__ == "__main__":
    c = CommandRepublisher()
    c.spin()