import rospy
import tf2_ros as tf

class TestResult:
    def __init__(self, name, msg):
        self.success = False
        self.msg = msg
        self.name = name

    def set_result(self, value):
        self.success = value

    def print(self):
        text = ": " + self.msg if not self.success else ""
        print_method = rospy.loginfo if self.success else rospy.logerr
        print_method(f"  Test {self.name}: {self.success} {text}")
        

class TestClass:
    def __init__(self, name, requires_tf):
        rospy.init_node(name)
        self.test_idx = 0
        self.test_results = []
        self.test_handles = []
        self.is_finished = False
        self.test_data = {}

        if requires_tf:
            #tf setup
            self.tf_Buffer = tf.Buffer(rospy.Duration(10))
            self.tf_listener = tf.TransformListener(self.tf_Buffer)
            self.tf_broadcaster = tf.TransformBroadcaster()
            self.tf_static_broadcaster = tf.StaticTransformBroadcaster()

    # Adds a delegate/handle to the test list, which will be called as soon as 
    # TestClass.advance_test() is called
    def register_test_handle(self, name, msg, handle):
        self.test_handles.append(handle)
        self.test_results.append(TestResult(name, msg))
        self.test_data[len(self.test_handles)-1] = {}


    def get_test_idx(self, handle):
        return self.test_handles.index(handle)

    # Starts the next test
    def advance_test(self, index, current_succcss):
        if index != self.test_idx:
            return

        self.test_results[self.test_idx].set_result(current_succcss)

        if len(self.test_results) > self.test_idx + 1:
            self.test_idx += 1

            # Start the next test
            self.run_next_test()
        else:
            # Log the test results to the screen
            self.test_idx = 999 
            self.report_tests_finished()

    def run_next_test(self):
        rospy.loginfo(f"Starting Test {self.test_idx}: {self.test_results[self.test_idx].name}")
        self.test_handles[self.test_idx]()


    # Reports the results of this test
    def report_tests_finished(self):
        rospy.loginfo("Tests are finished. Now printing results: ")
        for result in self.test_results:
            result.print()

        if any([not result.success for result in self.test_results]):
            rospy.logerr("At least one test failed!")

        self.is_finished = True


    # Waits for the self.is_finished field to return True and then lets the node die
    def run(self):
        if len(self.test_handles) > 0:
            self.run_next_test()
            rate = rospy.Rate(0.05)
            while not self.is_finished:
                rate.sleep()
        else:
            rospy.loginfo("This TestNode did not register any tests")
    