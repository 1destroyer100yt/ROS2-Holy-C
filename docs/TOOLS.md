# The ros2 command line, launch, turtlesim and rosbag

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
| `ros2 action list` / `-t` | `RosActionList;` / `RosActionListT;` |
| `ros2 action info /fibonacci` | `RosActionInfo("/fibonacci");` |
| `ros2 action send_goal --feedback /fibonacci example_interfaces/action/Fibonacci "{order: 5}"` | `RosActionSendGoalCli("/fibonacci","example_interfaces/action/Fibonacci","{order: 5}",TRUE);` (any key cancels) |
| `ros2 bag record -o bag1 /scan /tf` | `RosBagRecord("~/bag1","/scan /tf",0);` (any key stops; `NULL` = all topics) |
| `ros2 bag play bag1` / `--rate 2` | `RosBagPlay("~/bag1",1.0);` / `RosBagPlay("~/bag1",2.0);` (`0.0` = as fast as possible) |
| `ros2 bag info bag1` | `RosBagInfo("~/bag1");` |

Values can be YAML flow style (`{linear: {x: 0.5}}`) or `linear.x=0.5 angular.z=1.0`.
Every command also has a `...Str` version that returns the text instead of printing it.
The parameter commands go through the node's parameter services, the same way `ros2 param` does.
Topic and service lists hide names with a part starting with `_` (such as `/fibonacci/_action/...`),
like ros2 does without `--include-hidden-topics`.

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
`/turtle1/teleport_absolute` and `/turtle1/teleport_relative`, and the `/turtle1/rotate_absolute`
action. The arrow keys drive turtle1,
and Esc quits. `examples/TurtleSquare.HC` is a launch file that starts turtlesim with a node
that drives the turtle in a square.

The drawing and keyboard code is TempleOS-only and hasn't been run on a real install yet
(watch the `VERIFY` notes in `src/Tools/Turtlesim.HC`). The simulation itself is tested on Linux.

## Rosbag
Bags are rosbag2 directories in MCAP, the default storage of ROS 2 Jazzy: `bag1/bag1_0.mcap`
plus `metadata.yaml`. A bag recorded in TempleOS can be copied out and opened with
`ros2 bag info`/`ros2 bag play` on Linux, and bags recorded on Linux play in TempleOS.

- Written like rosbag2 writes them: messages sorted by time in chunks of about 768 KB, with
  message indexes, chunk indexes, statistics and a summary, so readers can seek.
- Every schema carries the full message definition (with the `MSG: pkg/Name` blocks of the
  types it uses). When reading, types we don't have are defined from that text, so bags with
  custom messages still play.
- Chunks must be uncompressed: zstd and lz4 aren't implemented. Record on Linux without
  `--compression-mode` to make bags TempleOS can read. A split bag only reads its first file.
- Messages are kept in memory while recording and the file is written when it stops.

In code: `RosBagCreate`/`RosBagWriteMsg`/`RosBagClose` write a bag directly,
`RosBagOpen` reads one (`bag->chans`, `bag->msgs` with the CDR bytes of each message), and
`RosBagRecordStart`/`RosBagRecordStop` record in the background.

`tools/bagtest/run.sh` checks all of this against two independent implementations: rosbags and
the `mcap` package read the bags we write (several chunks, nested types), and we read a chunked
bag rosbags wrote with a custom message type.
