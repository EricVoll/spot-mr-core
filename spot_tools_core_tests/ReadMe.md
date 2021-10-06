# Spot Tools Core Tests
This package contains ROS Nodes testing the functionality of the spot_tools_core package.

# Launching tests
Tests can be launched by executing this command: `roslaunch spot_tools_core_tests run_tests.launch`. Using the arguments `asa_tests`, `cmd_tests` and `odom_tests` you can opt-out of testing a certain ROS node from the main package.
E.g.: `roslaunch spot_tools_core_tests run_tests.launch asa_tests:=False` will not run the tests testing the ASA related nodes.

## Test framework usage
See [here](https://github.com/EricVoll/spot-mr-core/tree/master/spot_tools_tests_common)

## Test output:
Example output:
```
[INFO] [1633353488.458546]: Tests are finished. Now printing results: 
[INFO] [1633353488.463170]:   Test MissingAnchorCmd: True 
[INFO] [1633353488.467311]:   Test RepublishCmdRelative: True 
[INFO] [1633353488.471476]:   Test AutoSelectRotatedAnchor: True
```
If no tests are printed in red color, then all tests passed.
