# ROS2-Holy-C

ROS 2, written from the ground up in pure HolyC for TempleOS.

- What this is and why: [ABOUT.md](ABOUT.md)
- The full roadmap: [PLAN.md](PLAN.md)
- C++ plugin → HolyC converter: [tools/cpp2hc](tools/cpp2hc/README.md)

## Try it (inside TempleOS)
Copy this repo onto the TempleOS drive, then at the prompt:

```
Cd("~/ROS2-Holy-C/src");
#include "Load.HC"
PluginLoad("~/ROS2-Holy-C/tools/cpp2hc/examples/polygon/Polygons.HC");
PluginList;
```

The HolyC files have not been compiled on a real TempleOS install yet.
If something doesn't compile, open an issue with the error text.
