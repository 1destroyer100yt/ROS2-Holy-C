# The ros2 command line, launch and turtlesim

## The CLI
The TempleOS shell is a HolyC prompt, so the `ros2` commands are plain functions:

| ros2 | ROS2-Holy-C |
|---|---|
| `ros2 node list` | `RosNodeList;` |
| `ros2 node info /talker` | `RosNodeInfo("/talker");` |
| `ros2 topic list` / `-t` | `RosTopicList;` / `RosTopicListT;` |
| `ros2 topic info /chatter` / `-v` | `RosTopicInfo("/chatter");` / `RosTopicInfoV(...)` |
| `ros2 topic echo /chatter` | `RosTopicEcho("/chatter");` (any key stops) |
| `ros2 topic echo /odom --field pose.pose` | `RosTopicEchoN("/odom","pose.pose",0,timeout);` |
| `ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}"` | `RosTopicPub("/cmd_vel","geometry_msgs/msg/Twist","{linear: {x: 0.5}}",1.0);` |
| `ros2 topic pub --once ...` | `RosTopicPubOnce(...)` |
| `ros2 topic hz /scan` | `RosTopicHz("/scan");` (measures for 5 s) |
| `ros2 service list` / `-t` | `RosServiceList;` / `RosServiceListT;` |
| `ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 1, b: 2}"` | `RosServiceCall("/add_two_ints","example_interfaces/srv/AddTwoInts","{a: 1, b: 2}");` |
| `ros2 param list /talker` | `RosParamList("/talker");` |
| `ros2 param get /talker rate` | `RosParamGet("/talker","rate");` |
| `ros2 param set /talker rate 5.0` | `RosParamSetCli("/talker","rate","5.0");` |
| `ros2 param dump /talker` | `RosParamDump("/talker");` |
| `ros2 interface list` / `show` / `proto` | `RosInterfaceList;` / `RosInterfaceShow("...");` / `RosInterfaceProto("...");` |

Values can be YAML flow style (`{linear: {x: 0.5}}`) or `linear.x=0.5 angular.z=1.0`.
Every command also has a `...Str` version that returns the text instead of printing it.
The parameter commands go through the node's parameter services, the same way `ros2 param` does.

## Launch
A launch file is a HolyC file:
```
CRosLaunch *l=RosLaunchNew();
CRosLaunchNode *t=RosLaunchAdd(l,"talker","/demo",&TalkerSetup,NULL);
RosLaunchRemap(t,"chatter","news");     //remapping
RosLaunchParam(t,"rate","2.0");         //parameter (or RosParamFile("params.yaml") first)
RosLaunchStart(l);                      //one TempleOS task per node
...
RosLaunchStop(l);
```
`TalkerSetup(CRosNode *n,U8 *arg)` makes the node's publishers, timers and so on. To keep
everything in the current task instead, call `RosLaunchStartHere(l)` and spin `l->exec`.
Use `RosNodeOnDel(node,&Cleanup,state)` to free a node's state when the node goes away.

## Turtlesim
`TurtleSim;` runs turtlesim in the current window. It has the same topics and services as ROS 2:
`/turtle1/cmd_vel`, `/turtle1/pose`, `/spawn`, `/kill`, `/clear`, `/reset`, `/turtle1/set_pen`,
`/turtle1/teleport_absolute` and `/turtle1/teleport_relative`. The arrow keys drive turtle1,
and Esc quits. `examples/TurtleSquare.HC` is a launch file that starts turtlesim with a node
that drives the turtle in a square.

The drawing and keyboard code is TempleOS-only and hasn't been run on a real install yet
(watch the `VERIFY` notes in `src/Tools/Turtlesim.HC`). The simulation itself is tested on Linux.
