# ROS2-Holy-C

ROS 2, written from the ground up in pure HolyC for TempleOS.

- What this is and why: [ABOUT.md](ABOUT.md)
- The full roadmap: [PLAN.md](PLAN.md)
- C++ plugin → HolyC converter: [tools/cpp2hc](tools/cpp2hc/README.md)
- Messages (179 ROS 2 interfaces): [interfaces/](interfaces/README.md), demo in [examples/MsgDemo.HC](examples/MsgDemo.HC)

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

The HolyC files have not been compiled on a real TempleOS install yet.
If something doesn't compile, open an issue with the error text.
