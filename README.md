# MRS actionlib interface

Simple interface for the MRS UAV System and its 3D planner

## Dependencies

You need to install the mrs_uav_system, and its extensions - uav_modules and octomap_mapping_plannig. Follow the instructions in readme files of the repositories:

* [mrs_uav_system - uav_core and simulation](https://github.com/ctu-mrs/mrs_uav_system)
* [uav_modules](http://github.com/ctu-mrs/uav_modules)
* [octomap_mapping_planning](https://github.com/ctu-mrs/octomap_mapping_planning)

## The interface

The interface is based on the [actionlib](http://wiki.ros.org/actionlib) stack. it provides the user with takeoff, landing and goto commands.
The commands are defined in the command.action file, in the action folder:

```
# Define the goal
uint8 COMMAND_TAKEOFF=0
uint8 COMMAND_LAND=1
uint8 COMMAND_GOTO_PATHFINDER=2
uint8 COMMAND_GOTO_DIRECT=3
uint8 command

float64 goto_x
float64 goto_y
float64 goto_z
float64 goto_hdg
---
# Define the result
bool success
string message
---
# Define a feedback message
string message
float64 goto_distance_to_reference
```
To issue command to the action server, publish a goal to the topic:

```
/$UAV_NAME/mrs_actionlib_interface/goal
```

there are three possible commands, specified as numbers:

```
uint8 COMMAND_TAKEOFF=0
uint8 COMMAND_LAND=1
uint8 COMMAND_GOTO_PATHFINDER=2
uint8 COMMAND_GOTO_DIRECT=3
```
When the GOTO command is issued, fill also the goto variables to set the reference:

```
float64 goto_x
float64 goto_y
float64 goto_z
float64 goto_hdg
```

When a goal is being executed, feedback is provided on a topic:

```
/$UAV_NAME/mrs_actionlib_interface/feedback
```

Results (goal completed, goal aborted etc.) are published on a topic:

```
/$UAV_NAME/mrs_actionlib_interface/result
```

## Example session

Example tmuxinator session is provided in `simulation/start.sh`.
