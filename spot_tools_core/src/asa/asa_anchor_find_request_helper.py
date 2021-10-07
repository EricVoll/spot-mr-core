import rospy
from std_msgs.msg import String 
from asa_ros_msgs.srv import FindAnchor
class AsaAnchorFindHelper:
    def __init__(self):
        rospy.init_node("asa_request_helper")
        rospy.loginfo("Waiting for asa_ros/find_anchor serivce...")
        rospy.Subscriber('spot/asa_ros/request_anchor', String, self.asa_requested_cb)
        rospy.wait_for_service('/asa_ros/find_anchor')
        rospy.loginfo("asa_ros/find_anchor serivce is available. Ready to accept requests.")
        
    def asa_requested_cb(self, data):
        rospy.loginfo("Requesting to find the ASA anchor")
        # call into asa_ros service
        proxy = rospy.ServiceProxy('/asa_ros/find_anchor', FindAnchor)
        
        try:
            proxy(data.data)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))

if __name__ == "__main__":
    a = AsaAnchorFindHelper()
    rospy.spin()