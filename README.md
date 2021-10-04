# Spot MR Tools

This repository contains a few packages facilitating the work with Boston Dynamics Spot robot and Azure Spatial Anchors.
Currently these repositories exist with the corresponding purposes:

| Package-Name | Description |
|-|-|
|[spot_tools_core](spot_tools_core) | Ros package containing ROS-nodes performing tasks to command spot and send spot's odometry relative to ASA anchors |
|[spot_tools_core_tests](spot_tools_core_tests) | Ros package containing ROS-nodes that test the functionality of the `spot_tools_core` package without specifically requiring an `asa_ros` node running |
|[spot_tools_tests_common](spot_tools_tests_common) | Ros package containing classes needed for the Unit-Testing framework and to mock data |

Every package contains its own ReadMe to explain the usage.
