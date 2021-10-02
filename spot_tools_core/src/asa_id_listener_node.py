import rospy

import asa_ros_msgs.msgs

from asa.asa_handler import asa_handler 

class AsaIdListener:
    def __init__(self):

        rospy.Subscriber("/asa/find_anchor_request", std_msgs.msg.String, self.find_anchor_request, queue_size=10)

        self.asa_handler = None

    def find_anchor_request(self, data):
        # Create asa ros node and query the anchor
        self.asa_handler = asa_handler(data.data, self.asa_anchor_found)

    def asa_anchor_found(self, frame_id):
        rospy.loginfo(f"Found the anchor {frame_id}")
        

if __name__ == "__main__":
    node = AsaIdListener()
    rospy.spin()