from .test_class import TestClass
import rospy

class AnchorRotatorTests(TestClass):
    def __init__(self):
        rospy.init_node('anchor_rotator_tests')
        super.__init__()
    

if __name__ == "__main__":
    tester = AnchorRotatorTests()
    tester.run()