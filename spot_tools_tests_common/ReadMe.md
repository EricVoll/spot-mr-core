# Spot Tools Tests Common
This package contains python classes that can be used to test ROS Nodes and a few scripts to mock data.

## Usage
A basic test-ROS-node would look like this:

```python
import rospy
from spot_tools_tests_common.test_class import TestClass

class TestNode(TestClass):
   def __init__(self):
      super(TestNode, self).__init__('test_node', True)
      self.register_test_handle("TestCaseName", "Description of what went wrong if this test fails", self.test_callback)
      self.register_test_handle("TestCaseName2", "Description ...", self.test_callback_2)

   def test_callback(self):
      # Test some ros nodes by publishing mocked data
      # This test_callback is automatically called with a three seconds timeout.
      SomeFakedMsg = None
      self.publisher.publish(SomeFakedMsg)
      
   def test_callback_succeeded(self):
      # Somewhere (does not matter where) you have to call `advance_test` with the correct test ID. It will set the test with the corresponding ID to 
      # success/failure depending on the boolean sent with it. If `advance_test` is not called within 3 seconds of the test's start, then the test is auto-failed.
      # `advance_test` will automatically start the next test registered. (in this case test_callback_2)
      success = True
      self.advance_test(0, success)
      
   def test_callback_2(self):
      #Run code to test feature nr 2
      
if __name__ == "__main__":
   node = TestNode()
   node.run()
   
```
