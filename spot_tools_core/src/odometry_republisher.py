import rospy
from nav_msgs.msg import Odometry
from asa_ros_msgs.msg import FoundAnchor, CreatedAnchor
import tf2_ros as tf

class OdometryRepublisher:
    def __init__(self):

        self.anchor_ids = []
        self.robot_frame = "body"


        #tf setup
        self.tf_Buffer = tf.Buffer(rospy.Duration(10))
        self.tf_listener = tf.TransformListener(self.tf_Buffer)
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.tf_static_broadcaster = tf.StaticTransformBroadcaster()

        rospy.Subscriber('odometry', Odometry, self.republish_robot_odoom)
        rospy.Subscriber('found_anchor', FoundAnchor, self.found_anchor_callback)
        rospy.Subscriber('created_anchor', CreatedAnchor, self.created_anchor_callback)


        self.odom_publisher = rospy.Publisher('/odometry/filtered/asa_relative', Odometry, queue_size=10)

    def republish_robot_odom(self, data):
        current_parent_frame_id = data.header.frame_id
        child_frame_id = data.child_frame_id

        if(current_parent_frame_id == self.current_anchor_id):
            #rospy.loginfo("Skipped republishing since current anchor id "+ self.current_anchor_id + " is same as sent parent")
            return

        current_anchor_id = self.find_nearest_anchor()

        #Lookup the transform relative to the 
        trans = self.tf_Buffer.lookup_transform(current_anchor_id, child_frame_id, rospy.Time(0), rospy.Duration(1.0))
        new_odom = Odometry()
        new_odom.header = data.header
        new_odom.header.frame_id = self.current_anchor_id

        #Todo: couldn't find the data type of position. Will make nicer in future.
        new_odom.pose.pose.position.x = trans.transform.translation.x
        new_odom.pose.pose.position.y = trans.transform.translation.y
        new_odom.pose.pose.position.z = trans.transform.translation.z
        new_odom.pose.pose.orientation.x = trans.transform.rotation.x
        new_odom.pose.pose.orientation.y = trans.transform.rotation.y
        new_odom.pose.pose.orientation.z = trans.transform.rotation.z
        new_odom.pose.pose.orientation.w = trans.transform.rotation.w
        new_odom.pose.covariance = data.pose.covariance #We are no using covariance at all. Maybe do not set it?

        self.odom_publisher.publish(new_odom)




    #############################
    # ASA Anchor id maintenance # 
    #############################

    def found_anchor_callback(self, anchor):
        self.handle_anchor_id_received(anchor.anchor_id)

    def created_anchor_callback(self, anchor):
        self.handle_anchor_id_received(anchor.anchor_id)

    def handle_anchor_id_received(self, anchor_id):

        if anchor_id in self.anchor_ids:
            return
        
        if anchor_id.endswith("_rot"):
            id = anchor_id[:-4]
            if id in self.anchor_ids:
                self.anchor_ids.remove(id)

        self.anchor_ids.append(anchor_id)

        # Queries all registered anchors and returns the id of the nearest one to the robot frame
    def find_nearest_anchor(self):

        if(len(self.anchor_ids) == 1):
            return self.anchor_ids[0]

        smallestDistance = 1000000
        smallestDistanceAnchorId = None
        for anchor in self.anchor_ids:
            distance = self.getDistanceToAnchor(anchor, self.robot_frame)
            if(distance < smallestDistance):
                smallestDistance = distance
                smallestDistanceAnchorId = anchor

        # Check if the closest anchor changed and notify terminal about it.
        if(self.last_closest_anchor != smallestDistanceAnchorId):
            rospy.loginfo("changed anchor to " + smallestDistanceAnchorId)
            self.last_closest_anchor = smallestDistanceAnchorId

        return smallestDistanceAnchorId

    # returns the distance between the two frames
    def getDistanceToAnchor(self, anchor_id, target_frame):
        trans = self.tf_Buffer.lookup_transform(target_frame, anchor_id, rospy.Time(0))
        tr = trans.transform.translation
        return (tr.x**2 + tr.y**2 + tr.z**2)**0.5