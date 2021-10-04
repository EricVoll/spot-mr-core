import rospy

from spot_msgs.msg import TrajectoryGoal, TrajectoryAction
from geometry_msgs.msg import PoseStamped
import actionlib
from std_msgs.msg import Duration, String

class CommandExecuter():
    def __init__(self):
        rospy.init_node("command_executer")
        rospy.Subscriber('/spot/command/processed', PoseStamped, self.received_cmd)

        self.action_client = actionlib.SimpleActionClient('spot/trajectory', TrajectoryAction)
        rospy.loginfo("Waiting for spot/trajectory server to come online")
        self.action_client.wait_for_server()
        rospy.loginfo("spot/trajecotry server is online. Ready to take commands.")
        self.cmd_feedback_pub = rospy.Publisher('command/feedback', String, queue_size=10)

    def received_cmd(self, data):
        self.cmd_feedback_pub.publish(String("Received command"))
        goal = TrajectoryGoal()
        goal.target_pose = data
        goal.precise_positioning = False
        goal.target_pose.header.stamp = rospy.Time(0)
        goal.target_pose.header.frame_id = "body"
        goal.duration = Duration()
        goal.duration.data.secs = 10

        self.action_client.send_goal(goal, done_cb = self.command_finished)
        self.action_client.wait_for_result()
    
    def command_finished(self):
        self.cmd_feedback_pub.publish("Executed command")

    def spin(self):
        rospy.spin()


if __name__ == "__main__":
    e = CommandExecuter()
    e.spin()