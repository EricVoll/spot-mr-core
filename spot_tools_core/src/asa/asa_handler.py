from asa_ros_msgs.msg import CreatedAnchor
from asa_ros_msgs.msg import FoundAnchor
from asa_ros_msgs.srv import FindAnchor, CreateAnchor
from threading import Thread

import rospy
import tf2_ros as tf
from tf.transformations import *
from std_msgs.msg import String
from std_srvs.srv import Empty as EmptySrv
from std_msgs.msg import Empty as EmptyMsg
from geometry_msgs.msg import TransformStamped, Quaternion, PoseStamped
from nav_msgs.msg import Odometry

import roslaunch

class asa_handler:

    def __init__(self, anchor_id, found_anchor_callback, is_hololens_anchor = False):
        # self.world_drift_observer = WorldDriftObserver(0.5, self.rovio_drifted)
        rospy.loginfo("ASA Handler start " + 40*"=")
        self.anchor_id_preset = anchor_id
        self.found_anchor_callback = found_anchor_callback
        self.is_hololens_anchor = is_hololens_anchor

        self.drift_free_frame = "odom"
        self.asa_node_name = "asa_ros"
        self.is_asa_running = False
        self.asa_find_anchor_service_name = self.asa_node_name +"/find_anchor"
        self.asa_reset_service_name = self.asa_node_name +"/reset"

        # Subscribers listening to the asa ros wrapper
        rospy.Subscriber('/asa_anchor_rotator/anchor_rotated_created', FoundAnchor, self.asa_found_anchor_callback)

        self.start_asa_node()


    def start_asa_node(self):

        if self.is_asa_running:
            return #we don't want to launch it twice.

        rospy.loginfo("------------------ launching asa_ros ------------------")

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        
        arg = ['spot_tools_core', 'asa_ros.launch']
        if self.anchor_id_preset != "":
            arg.append('anchor_id:=' + self.anchor_id_preset)

        roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(arg)[0]
        roslaunch_args = arg[2:]
        
        self.asa_parent = roslaunch.parent.ROSLaunchParent(uuid, [(roslaunch_file, roslaunch_args)], 
         verbose=False,
         force_screen=False,
         process_listeners = [self.process_listener])
        self.asa_parent.start()
        self.is_asa_running = True
        rospy.loginfo("------------------ launched asa_ros ------------------")
    

    # Adds the found anchors id to the available id list
    def asa_found_anchor_callback(self, data):
        self.found_anchor_callback(data.anchor_id)
