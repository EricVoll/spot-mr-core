import rospy
import tf2_ros as tf
import queue
from threading import Thread

class TestResult:
    def __init__(self, name, msg):
        self.success = False
        self.msg = msg
        self.name = name
        self.timed_out = False

    def set_result(self, value):
        self.success = value

    def print(self):
        text = ": " + self.msg if not self.success else ""
        print_method = rospy.loginfo if self.success else rospy.logerr
        success_text = self.success if not self.timed_out else "Failed: TimedOut"
        print_method(f"  Test {self.name}: {success_text} {text}")

    def report_timedout(self):
        self.timed_out = True
        

class TestClass:
    def __init__(self, name, requires_tf):
        rospy.init_node(name)
        self.test_idx = 0
        self.test_results = []
        self.test_handles = []
        self.is_finished = False
        self.test_data = {}
        self.queue = queue.Queue()

        if requires_tf:
            #tf setup
            self.tf_Buffer = tf.Buffer(rospy.Duration(10))
            self.tf_listener = tf.TransformListener(self.tf_Buffer)
            self.tf_broadcaster = tf.TransformBroadcaster()
            self.tf_static_broadcaster = tf.StaticTransformBroadcaster()

    def dispatch_to_main_thread(self, func):
        self.queue.put(func)
        rospy.loginfo("dispatched")

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
        self.dispatch_to_main_thread(self.test_handles[self.test_idx])

        idx = self.test_idx

        thread = Thread(target = self.wait_for_timeout, args=[idx])
        thread.start()
        

    def wait_for_timeout(self, idx):
        rospy.sleep(3)

        if self.test_idx == idx:
            self.test_results[idx].report_timedout()
            self.advance_test(idx, False)


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
            while not self.is_finished:
                try:
                    callback = self.queue.get(False) #doesn't block
                    thread = Thread(target=callback)
                    thread.start()

                except queue.Empty:
                    pass

                rospy.sleep(0.1)

        else:
            rospy.loginfo("This TestNode did not register any tests")
    