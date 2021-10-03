import rospy

'''
Type: Ros-Node
Description: Listens to a topic publishing commands relative to some frame and republishes them relative to spot's body frame
Input: Commands on the topic "/command/raw"
Output: Commands on the topic "/command/processed"

'''

class CommandRepublisher():
    def __init__(self):
        rospy.init_node("command_republisher")

if __name__ == "__main__":
    c = CommandRepublisher()
    c.spin()