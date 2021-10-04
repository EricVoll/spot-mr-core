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
