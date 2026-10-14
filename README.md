# ROS2-Holy-C

ROS 2, written from the ground up in pure HolyC for TempleOS.

- What this is and why: [ABOUT.md](ABOUT.md)
- The full roadmap: [PLAN.md](PLAN.md)
- C++ plugin → HolyC converter: [tools/cpp2hc](tools/cpp2hc/README.md)
- Messages (176 ROS 2 interfaces: messages, services, actions): [interfaces/](interfaces/README.md), demo in [examples/MsgDemo.HC](examples/MsgDemo.HC)
- Nodes, topics, services, parameters: [docs/GRAPH.md](docs/GRAPH.md), examples in [examples/](examples/)
- The ros2 command line, actions, launch, turtlesim, rosbag: [docs/TOOLS.md](docs/TOOLS.md)
- Talking to real ROS 2 over the network (DDS/RTPS): [docs/NETWORK.md](docs/NETWORK.md);
  tested against a real ROS 2 Jazzy with both Fast DDS and Cyclone DDS (`tools/ros2test`)
- Coordinate frames (tf2): [docs/TF.md](docs/TF.md)
- RvizLite, a 2D rviz: [docs/RVIZ.md](docs/RVIZ.md)
- Getting it into TempleOS (disk image, QEMU, network): [docs/TEMPLEOS.md](docs/TEMPLEOS.md)
- Hardware (serial ports, RoboClaw motor controllers, a diff-drive base): [docs/HARDWARE.md](docs/HARDWARE.md)

## Test it (no TempleOS needed)
```
tools/check.sh
```
Builds the HolyC as C on Linux and runs the unit tests (see [tools/hctest](tools/hctest/README.md)).
HolyC coding rules: [docs/HOLYC_RULES.md](docs/HOLYC_RULES.md).

## Try it (inside TempleOS)
Copy this repo onto the TempleOS drive, then at the prompt:

```
#include "~/ROS2-Holy-C/tests/Run.HC"
RosTestsRun;
PluginLoad("~/ROS2-Holy-C/tools/cpp2hc/examples/polygon/Polygons.HC");
PluginList;
```
(`Run.HC` loads everything first; outside of tests use `#include "~/ROS2-Holy-C/Load.HC"`.)

All of it compiles on a real TempleOS 5.03, and all 2152 unit tests pass there (in QEMU, see
[docs/TEMPLEOS.md](docs/TEMPLEOS.md)).
If something doesn't compile, open an issue with the error text.
