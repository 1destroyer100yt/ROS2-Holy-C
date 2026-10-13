# interfaces/

ROS 2 message and service definitions, plus the HolyC generated from them.

| What | Where |
|---|---|
| `.msg` / `.srv` source files | `interfaces/<pkg>/msg/*.msg`, `interfaces/<pkg>/srv/*.srv` |
| Load order (dependencies first) | `Index.txt` |
| Generated HolyC per package | `<pkg>/Types.HC` (don't edit) |
| Loads all packages | `Load.HC` (don't edit) |

179 interfaces in 24 packages: `builtin_interfaces std_msgs geometry_msgs sensor_msgs nav_msgs
rcl_interfaces tf2_msgs rosgraph_msgs diagnostic_msgs trajectory_msgs visualization_msgs
action_msgs actionlib_msgs lifecycle_msgs shape_msgs stereo_msgs statistics_msgs
unique_identifier_msgs rmw_dds_common service_msgs type_description_interfaces std_srvs
example_interfaces turtlesim`.

## Where the definitions come from
- **Messages** are the ROS 2 Jazzy definitions (Apache-2.0, from the ROS 2 repositories),
  exported by `tools/ifacegen/export_rosbags.py`. That script reads them from the `rosbags`
  Python library. Comments and default values aren't kept in that export.
- **Services, turtlesim, example_interfaces** are written by hand to match ROS 2.

## Using them in HolyC

```
CMsg_geometry_msgs_Twist *cmd=RosMsgNew(ros_t_geometry_msgs_Twist);
cmd->linear.x=0.5;
RosMsgPrint(ros_t_geometry_msgs_Twist,cmd);
```

Every message gets a class `CMsg_<pkg>_<Name>` and a type handle `ros_t_<pkg>_<Name>`.
Constants become `#define ROS_<pkg>_<Name>_<CONST>`. Services get
`CSrv_<pkg>_<Name>_Request/_Response` and `ros_srv_<pkg>_<Name>`. See `examples/MsgDemo.HC`.

## Adding your own message
1. Put `my_pkg/msg/Thing.msg` (or `srv/Thing.srv`) in here.
2. Add `my_pkg/msg/Thing` to `Index.txt` after everything it uses. Or run
   `tools/ifacegen/export_rosbags.py --index-only`, which rebuilds the order for you.
3. Regenerate: `tools/gen_interfaces.sh` on Linux, or `Cd("~/ROS2-Holy-C"); RosIfaceGen(".");` in TempleOS.

You can also define a type at runtime without generating anything:
`RosMsgDefine("my_pkg/msg/Thing","float64 x\nstring name\n");`. You only lose the typed class.
