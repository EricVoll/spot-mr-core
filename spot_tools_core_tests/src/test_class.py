import rospy

class TestResult:
    def __init__(self, name, msg):
        self.success = False
        self.msg = msg
        self.name = name

    def set_result(self, value):
        self.success = value

    def print(self):
        text = ": " + self.msg if not self.success else ""
        rospy.loginfo(f"Test {self.name}: {self.success} {text}")

class TestClass:
    def __init__(self):
        self.test_idx = 0
        self.test_results = []
        self.test_handles = []
        self.is_finished = False

    # Adds a delegate/handle to the test list, which will be called as soon as 
    # TestClass.advance_test() is called
    def register_test_handle(self, name, msg, handle):
        self.test_handles.append(handle)
        self.test_results.append(TestResult(name, msg))

    def get_test_idx(self, handle):
        return self.test_handles.index(handle)

    # Starts the next test
    def advance_test(self, index, current_succcss):
        if index < self.test_idx:
            rospy.loginfo("SKipped test end because it was reported already")
            return

        self.test_results[self.test_idx].set_result(current_succcss)

        if len(self.test_results) >= self.test_idx + 1:
            self.test_idx += 1

            # Start the next test
            self.test_handles[self.test_idx]()
        else:
            # Log the test results to the screen
            self.report_tests_finished()

    # Reports the results of this test
    def report_tests_finished(self):
        for result in self.test_results:
            result.print()

        self.is_finished = True


    # Waits for the self.is_finished field to return True and then lets the node die
    def run(self):
        if len(self.test_handles) > 0:
            self.test_handles[0]()
            rate = rospy.Rate(1)
            while not self.is_finished:
                rate.sleep()
        else:
            rospy.loginfo("This TestNode did not register any tests")
    