# spot_tools_core package

This package contains RosNodes which help users to work with Spot and Azure Spatial Anchors. The current list of Ros Nodes is:
| Ros Node Name | Description | Ros-Launch file |
|-|-|-|
|AsaAnchorRotatorNode| Listens to `ASA Found Anchor` events and rotates the tf-frames. This is required for ASA anchors that are created in Unity (for example on HoloLens2) because Unity uses a y-up system. With these rotations, the anchor can be used as a z-up frame | RosNode of type `asa_anchor_rotator_node.py` |
|CommandRepublisher| Listens to trajectory commands send relative to an ASA anchor and republishes them relative to Spot's base coordinate frame | `cmd_republisher.launch` |
|CommandExecuter| Listens to trajectory commands relative to spot's body frame and calls the `spot_driver`'s trajectory service to execute the command | `cmd_executer.launch` |
|AsaOdomtry| Listens to spot's odometry topic and republishes it relative to the closes ASA anchor | `odom_republisher.launch` |

# Launch files
The launch file `viscon.launch` is an example of a launch file launching all required nodes for spot to be commanded relative to an ASA node. 

# ROS TCP Endpoint
To communicate with Unity you will also have to install the [ros-tcp-endpoint](https://github.com/Unity-Technologies/ROS-TCP-Endpoint) package. Simply clone the repository into the catkin_ws/src foulder and build it. The `spot_tools_core` package does not have the `ros-tcp-endpoint` as a dependency, because none of the features depend on it, even though many of them only make sense in combination with it.

The `viscon.launch` launchfile already supports launching this node if wanted. It can be activated/deactivated using the `launch_tcp_node` argument.

The IP address of the launched tcp endpoint can be configued by editing the `/spot_tools_core/config/ros_tcp_endpoint_params.yaml` file.
