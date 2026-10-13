# About ROS2-Holy-C

**ROS2-Holy-C** is ROS 2 rebuilt from scratch in **pure HolyC**, the language of TempleOS.

It isn't a port or a wrapper around the C++ code. The ROS 2 ideas (nodes, topics,
services, parameters, executors, plugins) are written again in HolyC, and later the
real DDS/RTPS wire protocol too, so a TempleOS machine can talk to a normal ROS 2 robot.

## Why
- Because it's never been done.
- Everything runs in one address space, so local message passing is zero-copy without
  any extra work.
- HolyC is compiled from source when it loads, so plugins can be **edited and reloaded
  while the robot is running**. No rebuild and no restart.
- The TempleOS shell is a HolyC prompt, so ROS commands like `RosTopicList;` are just functions.

## What's in here
| Path | What it is |
|---|---|
| [`PLAN.md`](PLAN.md) | The full roadmap: architecture, phases, limits, and what can't be done |
| `Load.HC` | Loads everything: `#include "~/ROS2-Holy-C/Load.HC"` |
| `src/` | The HolyC library (core, messages, plugins) |
| `interfaces/` | 179 ROS 2 message/service types and their generated HolyC classes |
| `tests/` | Unit tests; they run in TempleOS and on Linux (`tools/check.sh`) |
| `src/Plugin/PluginLib.HC` | Plugin system (replaces pluginlib) |
| `tools/cpp2hc/` | Converts C++ ROS 2 plugins into HolyC |
| `tools/hctest/` | Builds and tests the HolyC on Linux, plus a HolyC linter |

## Status
Done so far: the plan, the core library, ROS 2 messages (179 interface types,
byte-compatible with ROS 2's wire format), nodes/topics/services/parameters/executors
inside TempleOS (talker and listener run as separate tasks), the plugin registry, and
the C++→HolyC plugin converter. The rest is tracked in [`PLAN.md`](PLAN.md).

## Author
Made by **Andrew Detroyer** ([@1destroyer100yt](https://github.com/1destroyer100yt)).
