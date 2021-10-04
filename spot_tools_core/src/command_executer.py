import rospy

from spot_msgs.srv import Trajectory, TrajectoryResponse
from geometry_msgs.msg import PoseStamped

class CommandExecuter():
    def __init__(self):
        rospy.Subscriber('/spot/command/processed', PoseStamped, self.received_cmd)

    def received_cmd(self, data):

        # build trajectory command
        rospy.loginfo("Marked goal. Sleep.")
        rospy.sleep(self.options["command_delay"])
        rospy.loginfo("Starting command.")
        trajectory = Trajectory()
        trajectory.target_pose = data.pose
        trajectory.target_pose.header.frame_id = "body"
        trajectory.duration = 15
        
        rospy.wait_for_service('spot/trajectory')
        trajectory_service = rospy.ServiceProxy('spot/trajectory', Trajectory)
        response = trajectory_service(trajectory.target_pose, trajectory.duration)
        rospy.loginfo(response.success)
        rospy.loginfo(response.message)

    def spin(self):
        rospy.spin()


if __name__ == "__main__":
    e = CommandExecuter()
    e.spin()