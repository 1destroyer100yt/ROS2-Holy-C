# ROS2-Holy-C — The Plan

Goal: a ROS 2–compatible robotics middleware written **from the ground up in pure HolyC**,
running natively on TempleOS (and kept compatible with ZealOS where practical).
No C++, no Python, no Rust in the runtime. No porting of `rclcpp`/`rcl`/`rmw` code —
we re-implement the *ideas* and the *wire protocol*, not the source.

---

## 0. Ground truth: what HolyC / TempleOS gives us and what it doesn't

Everything in this plan is shaped by these facts. If one of them turns out wrong on a
real install, fix this table first, then the plan.

| Fact | Consequence for us |
|---|---|
| Single address space, everything runs in ring 0, no memory protection | "Shared memory transport" is free: every task can see every pointer. One bad pointer crashes the whole OS, so we need defensive checks and a debug mode. |
| HolyC is JIT-compiled from source (`#include`, `ExeFile`), AOT `.BIN` also exists | We don't need `colcon`/CMake. A "package" is a folder with a `Load.HC`. Plugins can be loaded **as source at runtime** — this is our big advantage. |
| No dynamic linking of ELF/`.so` files | `pluginlib`'s class loader can't exist as-is. Replaced by a source-level plugin registry (`src/Plugin/PluginLib.HC`) plus the `cpp2hc` converter. |
| Only one float type: `F64` (no `F32`) | ROS messages full of `float32` must be stored as `F64` in memory and converted to/from IEEE-754 single precision **by hand** when serializing. |
| Classes are data only: no methods, no templates, no overloading, no namespaces, single inheritance of fields (`class B:A`) | API style is C-like: `RosPublisherPublish(pub, msg)`. Message "templates" are generated code, one `.HC` per message type. |
| Postfix casts `x(I64)` only relabel bits; number conversion is `ToI64`/`ToF64` | Converted code and our own code must use `ToI64`/`ToF64` for float↔int. |
| `try` / `catch` / `throw` exist but exceptions are just `I64` codes | Errors are return codes (`ROS_OK`, `ROS_ERR_*`); `throw` only for truly fatal stuff. |
| Built-in hash tables, `MAlloc`/`CAlloc`/`Free`, string helpers, `Print`/`StrPrint` | We build our own tiny containers (vector, ring buffer, map) on top. No STL. |
| Multicore exists (`Spawn` onto a CPU), but plan for **cooperative** scheduling — long loops must `Yield` | Executor is designed around cooperative ticks. A multi-threaded executor pins work onto other cores. |
| TempleOS has **no network stack and no NIC drivers** | Talking to a real ROS 2 system needs a NIC driver + Ethernet/ARP/IPv4/UDP + RTPS. That's the hardest part, so it comes last and in stages. ZealOS has started a network stack; we can borrow ideas but stay self-contained. |
| 640×480, 16 colors, DolDoc documents | `rviz`-style tools have to be tiny and 2D. DolDoc lets us make clickable, live-updating CLI output. |
| RedSea / FAT32 file systems, file names limited to ~37 chars | Keep file names short: `Pub.HC`, `Sub.HC`, `Cdr.HC`, not `subscription_intra_process_buffer.hpp`. |
| No YAML, XML or JSON parser | We write small subset parsers: `.msg/.srv`, parameter YAML (flat subset), plugin XML (subset). |

---

## 1. Architecture

```
 +--------------------------------------------------------------+
 |  Tools:   RosTopicList  RosTopicEcho  RosNodeList  RosParam   |   <- HolyC shell commands
 |           RosLaunch(file)  RvizLite  cpp2hc (host side)       |
 +--------------------------------------------------------------+
 |  Client lib "rhc" (like rclcpp, but C-style HolyC)            |
 |  Node  Publisher  Subscription  Timer  Service  Client        |
 |  Parameters  Logging  Clock/Time  Executor  QoS  Plugins      |
 +--------------------------------------------------------------+
 |  Graph core: topic table, matching, QoS check, discovery db   |
 +----------------------+---------------------------------------+
 |  Transport: Local    |  Transport: Serial  |  Transport: RTPS |
 |  (zero-copy, same OS)|  (COM1 bridge)      |  (UDP/IPv4, DDS  |
 |                      |                     |   wire compat)   |
 +----------------------+---------------------+------------------+
 |  Core lib: Mem, Str, Vec, Ring, Map, Time, Log, CDR, F32<->F64|
 +--------------------------------------------------------------+
 |  Drivers (later): NIC (e1000 / RTL8139 / PCNet in QEMU)       |
 |  Net: Ethernet, ARP, IPv4, UDP, IGMP multicast                |
 +--------------------------------------------------------------+
 |  TempleOS / ZealOS                                            |
 +--------------------------------------------------------------+
```

Repository layout (target):

```
PLAN.md                 this file
README.md
src/
  Load.HC               #includes everything in the right order
  Core/                 Mem.HC Str.HC Vec.HC Ring.HC Map.HC Time.HC Log.HC HcCompat.HC
  Msg/                  MsgParse.HC (reads .msg/.srv), MsgGen.HC (writes .HC), Cdr.HC, F32.HC
  Graph/                Node.HC Pub.HC Sub.HC Timer.HC Srv.HC Cli.HC Param.HC QoS.HC Exec.HC
  Transport/            Local.HC Serial.HC Rtps*.HC
  Net/                  Eth.HC Arp.HC Ip.HC Udp.HC Igmp.HC
  Drv/                  E1000.HC RTL8139.HC PCNet.HC
  Plugin/               PluginLib.HC (pluginlib replacement)
  Tools/                Cli.HC Launch.HC Rviz.HC
interfaces/             std_msgs, geometry_msgs, sensor_msgs ... (.msg sources + generated .HC)
examples/               Talker.HC Listener.HC AddTwoInts.HC Turtle.HC ...
tests/                  HolyC unit tests, run inside the OS, results printed to COM1
tools/cpp2hc/           host-side C++ plugin -> HolyC converter (see section 7)
```

Naming conventions (match TempleOS style):
- Functions `RosThingVerb` (`RosPubNew`, `RosPubPublish`, `RosNodeSpin`).
- Classes `CRosNode`, `CRosPub`, ... Message classes `CMsg_std_msgs_String`.
- Constants `ROS_UPPER_CASE`.
- Every public function documented with a `//` line above it.

---

## 2. Phases and milestones

Each phase ends with a **demo you can run in a VM**. Don't start the next phase until
the demo works on real TempleOS in QEMU/VirtualBox.

### Phase 0 — Toolchain and test loop (1–2 weekends)
- QEMU + TempleOS ISO set up; one script to build a RedSea/FAT32 disk image holding `src/`.
- `tests/Run.HC`: runs every test, prints `PASS/FAIL name` lines to the screen **and** COM1,
  so the host can capture results with `qemu -serial file:out.txt`.
- A host script that boots QEMU, waits, and greps `out.txt` for `FAIL`. This becomes CI.
- **Demo:** `#include "Load.HC"; RunTests;` prints `0 failed`.

### Phase 1 — Core library
- `Mem.HC`: pool allocator for fixed-size message buffers (avoid `MAlloc` in hot paths).
- `Str.HC`: owned strings, builder, split, trim, compare.
- `Vec.HC`: growable array of bytes with element size (our `std::vector`).
- `Ring.HC`: lock-free single-producer/single-consumer ring buffer (for queues between cores).
- `Map.HC`: string→pointer map (wrap TempleOS hash tables or our own open addressing).
- `Time.HC`: monotonic time from TSC/HPET, `CRosTime {I64 sec; I64 nsec;}`, rate/sleep helpers.
- `Log.HC`: `RosLogDebug/Info/Warn/Error/Fatal` with levels, colors and node name.
- `F32.HC`: `U32 F64ToF32Bits(F64)`, `F64 F32BitsToF64(U32)` — rounding, NaN, Inf, denormals.
- **Demo:** unit tests for every module.

### Phase 2 — Messages
- `MsgParse.HC`: parse ROS 2 `.msg` and `.srv` (primitive types, fixed/bounded/unbounded
  arrays, strings, constants, default values, nested types, `---` split for services).
- **As built:** serialization is driven by runtime type descriptions (parsed from the .msg
  text at load time) instead of generated per-type functions, and the generator only makes the
  typed classes. That's less generated code, and it works for types defined at runtime too.
  Original idea:
- `MsgGen.HC`: generates, per message, a `.HC` file with:
  - `class CMsg_pkg_Name { ... };`
  - `CMsg_pkg_Name *Msg_pkg_Name_New()`, `_Del`, `_Copy`
  - `I64 Msg_pkg_Name_Ser(CMsg_pkg_Name *m, CRosBuf *out)` and `_Deser` (CDR, little endian,
    alignment rules exactly as XCDR1 so it's byte-compatible with Fast DDS / Cyclone).
  - A type descriptor (`CRosTypeInfo`) with field names/offsets for generic echo/printing.
- The generator itself runs **inside TempleOS** (pure HolyC). Generated files are committed.
- Ship `std_msgs`, `builtin_interfaces`, `geometry_msgs`, `sensor_msgs` (subset), `nav_msgs`,
  `std_srvs`, `example_interfaces`, `rcl_interfaces`.
- **Demo:** serialize `geometry_msgs/Twist` and compare bytes against a golden file captured
  from real ROS 2 (captured once on a Linux box and committed as `tests/golden/*.bin`).

### Phase 3 — Graph and executor (local only)
- `Node.HC`: name, namespace, remapping table, list of entities.
- `Pub.HC` / `Sub.HC`: create/destroy, publish, callback, per-subscription queue depth.
- `QoS.HC`: reliability, durability (`TRANSIENT_LOCAL` keeps last N), history, depth, deadline
  (later), compatibility checks like real ROS 2.
- `Timer.HC`, `Srv.HC`, `Cli.HC` (services with request/response matching by sequence number).
- `Param.HC`: declare/get/set, types bool/int/double/string/arrays, callbacks on change,
  load from a YAML subset file.
- `Exec.HC`: single-threaded executor (`RosSpin`, `RosSpinOnce`, `RosSpinSome`) plus a
  multi-core executor that spawns worker tasks on other CPUs with SPSC rings.
- `Local.HC` transport: zero-copy — publish hands a refcounted pointer to every local
  subscriber. This is something real ROS 2 has to work hard for; here it's trivial.
- **Demo:** `Talker` + `Listener` in two TempleOS tasks; `AddTwoInts` service; parameters.

### Phase 4 — Tools (the "ros2 CLI")
The TempleOS shell *is* a HolyC REPL, so tools are just functions:
- `RosNodeList;` `RosTopicList;` `RosTopicEcho("/chatter");` `RosTopicPub("/cmd_vel","geometry_msgs/Twist","linear.x=0.5");`
  `RosTopicHz("/scan");` `RosServiceCall(...)` `RosParamGet/Set` `RosInterfaceShow("std_msgs/String");`
- `RosLaunch("Bringup.HC")`: launch files are plain HolyC scripts — no Python launch system.
- DolDoc output: topic names in `RosTopicList` are clickable and open an echo window.
- **Demo:** drive a simulated turtle (`examples/Turtle.HC`, 640×480 drawing) with `RosTopicPub`.

### Phase 5 — Plugins (pluginlib replacement)
- `PluginLib.HC`: register classes by name and base type, create/destroy instances,
  look up methods by name, list declared/loaded classes, **hot reload** from source.
- Plugin description manifests (`plugins.xml` subset) → `PluginDescribe(...)` calls.
- `tools/cpp2hc`: converter for existing C++ pluginlib plugins (see section 7).
- **Demo:** load a converted plugin, call it, edit its source, reload it without restarting.

### Phase 6 — Serial bridge (first contact with real ROS 2)
- `Serial.HC`: framed protocol over COM1 (`0x7E` framing, length, CRC16, topic id, CDR payload).
- Because the payload is real CDR, the other end only needs to forward bytes into DDS.
  A host-side bridge is *outside* this repo's "pure HolyC" rule and is optional; it exists only
  until Phase 7 lands. (Could be a ~100-line program for whoever wants it.)
- **Demo:** TempleOS publishes `/chatter`, `ros2 topic echo /chatter` on Linux sees it.

### Phase 7 — Native networking + DDS/RTPS (the big one)
Stage A — drivers and IP:
- NIC driver for one QEMU-emulated card first (pick **e1000** or **RTL8139**; PCNet if we
  follow ZealOS). PCI enumeration, DMA rings, interrupts or polling.
- Ethernet, ARP (cache + replies), IPv4 (no fragmentation at first), UDP, ICMP echo (for `ping`),
  IGMP for multicast group join. Static IP first, DHCP later.

Stage B — RTPS (DDS wire protocol, OMG DDSI-RTPS 2.x):
- Message header, submessages: `DATA`, `HEARTBEAT`, `ACKNACK`, `INFO_TS`, `INFO_DST`, `GAP`.
- GUIDs, sequence numbers, locators, parameter lists.
- SPDP (participant discovery, multicast 239.255.0.1, port 7400 + domain math).
- SEDP (endpoint discovery: publications/subscriptions builtin topics).
- Best-effort writer/reader first, then reliable (heartbeat/acknack/resend).
- ROS 2 name mangling: topic `/chatter` → DDS topic `rt/chatter`, type
  `std_msgs::msg::dds_::String_`; services `rq/` / `rr/`.
- User data / `enclave` QoS bits needed so `ros2 node list` shows our nodes.
- **Demo:** `ros2 topic list` on Linux shows topics from TempleOS; talker/listener both ways;
  `ros2 service call` into TempleOS.

### Phase 8 — Robot-facing stuff
- `tf2`-lite: transform tree, buffer with time, lookup + interpolation (quaternion math in F64).
- `RvizLite`: 2D view of `LaserScan`, `OccupancyGrid`, `Path`, TF frames, at 640×480.
- Rosbag: record/play rosbag2 MCAP bags, so they move between TempleOS and Linux.
- Hardware: serial motor controllers, joysticks via PS/2 or serial. (USB is hard on TempleOS.)

---

## 3. Testing strategy

- **Unit tests in HolyC** under `tests/`, one file per module, `RunTests;` runs them all.
- Results printed to COM1 so they can be captured headlessly.
- **Golden files**: CDR bytes and RTPS packets captured once from real ROS 2 (Wireshark has an
  RTPS dissector) and committed. Our serializer must produce the same bytes.
- **Interop matrix** (Phase 7): ROS 2 Humble and Jazzy, Fast DDS and Cyclone DDS, both directions.
- Fuzz the parsers (`.msg`, YAML, RTPS) with random/mutated input inside the VM — remember a
  crash takes down the whole OS, so the parsers must never trust lengths.

---

## 4. Things that will NOT be possible (or not soon) — and what we do instead

| ROS 2 feature | Why it's a problem here | What we do |
|---|---|---|
| `pluginlib` / `class_loader` (`.so` loading) | No dynamic linker, no C++ ABI | Source-level plugin registry + hot reload + `cpp2hc` converter |
| C++ templates (`create_publisher<T>`) | No templates | Generated per-type functions + a generic `RosPubNew(node, topic, &type_info, qos)` |
| Python nodes, Python launch files | No Python | HolyC launch scripts; nodes in HolyC only |
| `float32` fields | No `F32` type | Store `F64`, convert bits on (de)serialize |
| Security (SROS2, DDS-Security, TLS) | No crypto libs, everything ring 0 anyway | Out of scope. Treat it as a trusted, isolated network. |
| Shared-memory transports to *other* machines' processes (iceoryx etc.) | Different OS | Only our own zero-copy local transport |
| USB devices (cameras, most sensors) | TempleOS has basically no USB | Serial/PS/2 devices, or get sensor data from another ROS 2 machine over the network |
| Gazebo, RViz2, rqt | Huge GUI/3D stacks | `RvizLite` 2D, sim on the Linux side over DDS |
| Real-time guarantees | No priority-inheritance RTOS features; cooperative scheduling | Keep callbacks short, pin hot work to its own core |
| Full DDS (all QoS, content filters, XTypes) | Enormous spec | Subset that ROS 2 actually uses by default |
| Large messages (images, point clouds) over UDP | Needs IP/RTPS fragmentation | Phase 7 later stage: RTPS `DATA_FRAG` |

---

## 5. Limits of writing code outside the VM

Being honest about this up front so nobody gets surprised later:

1. **Code written on a normal dev machine can't be run there.** There's no TempleOS image or
   HolyC compiler outside the VM. Every HolyC file needs to be
   tested by a human in a VM. Expect small syntax fixes on the first compile.
2. **HolyC has no formal spec.** Some dialect details (function-pointer syntax in some
   positions, exact library function names) are written from memory of the TempleOS sources
   and must be checked against a real install.
3. **No real ROS 2 install to test against** unless one is set up on a Linux box. Golden CDR/RTPS captures must be
   recorded by a human on a Linux machine with ROS 2.
4. **No hardware access.** NIC drivers, serial devices and robots need real testing;
   QEMU emulation is a first step, not the last.
5. **Timing/performance can only be measured** on the real system.
6. **This plan file is the project's memory.** Keep it updated
   with what was finished and what broke.

What *can* be done well from outside the VM: writing code against this plan, the message
generator logic, the CDR/RTPS byte layouts (checked against golden files), host-side tools
like `cpp2hc` (which *can* be tested anywhere), and reviewing bug reports from the VM.

Recommended workflow: write code → copy disk image into VM → `RunTests;` → paste the
`FAIL` lines / compiler errors back → fix → repeat.

---

## 6. Decisions to make (owner's call)

- **TempleOS 5.03 only, or ZealOS too?** Recommendation: develop on TempleOS 5.03 (the purist
  target), keep a `HcCompat.HC` shim so ZealOS can work, and look at ZealOS's networking code
  for reference when Phase 7 starts.
- **Which ROS 2 distro to be wire-compatible with first?** Recommendation: Jazzy (LTS), then Humble.
- **Which NIC first?** Recommendation: whatever is easiest in QEMU and documented best — e1000.

---

## 7. Plugins and the `cpp2hc` converter

ROS 2 plugins (`pluginlib`) are C++ classes compiled into `.so` libraries and loaded at runtime
by class name (`"my_pkg::MyPlanner"`). None of that machinery can exist on TempleOS. (Same
story for Rust: pluginlib is C++-only, so Rust ROS 2 crates can't consume C++ plugins either.)

What we do instead is **better**, not just a workaround:

1. **`src/Plugin/PluginLib.HC`** — the registry.
   - `PluginClassAdd(type, base, &Ctor, &Dtor)` / `PluginMethodAdd(type, "method", &Fn)`
   - `PluginLoad("Square.HC")` JIT-compiles the plugin **from source**.
   - `PluginCreate("polygon_plugins::Square")`, `PluginMethod(obj, "area")`, `PluginDestroy(obj)`
   - `PluginReload(file)` — edit the plugin and reload it while the robot is running.
     pluginlib needs a full rebuild and a process restart for that.
   - `PluginList;` prints every class, its base, its methods and the file it came from.

2. **`tools/cpp2hc/cpp2hc.py`** — converts existing C++ pluginlib plugins to HolyC.
   It's a *host-side* developer tool (the C++ source lives on your Linux box anyway), with zero
   dependencies beyond Python 3. It is deliberately **dumb**:
   - Pattern-based, predictable, one file, no clang/LLVM required.
   - Converts the boring 80%: classes → HolyC classes (with `class B:A` inheritance), methods →
     `Class_method(Class *self, ...)`, constructors/destructors → `Class_New`/`Class_Del`,
     member access → `self->x`, C++ types → HolyC types, `std::cout`/`printf`/`RCLCPP_*` →
     `Print`/`RosLog*`, `new`/`delete`, casts, `for (int i...)` hoisting, math functions,
     `PLUGINLIB_EXPORT_CLASS` → registry calls, `plugins.xml` → `PluginDescribe` manifest.
   - **Never silently wrong**: anything it doesn't understand (templates, STL containers,
     lambdas, smart pointers, overloads, typed exceptions...) gets a
     `//TODO(cpp2hc): reason` marker on that exact line.
   - Prints a **portability score** (% of lines with no TODO) and a list of every TODO,
     so you know in one second whether a plugin is a 5-minute fix or a rewrite.
   - `--strict` exits non-zero when TODOs remain (for scripts/CI).
   - Fully unit tested on the host (`python3 -m unittest` in `tools/cpp2hc`).

---

## 8. Progress tracker

- [x] Plan written
- [x] `cpp2hc` converter v0 + tests
- [x] `PluginLib.HC` + `HcCompat.HC` v0 (needs first compile on real TempleOS)
- [~] Phase 0 toolchain + test runner
  - [x] `tests/Run.HC` + `src/Core/Test.HC` test framework (same tests run in TempleOS and on Linux)
  - [x] Host test loop: `tools/hctest` (HolyC→C + gcc + ASan/UBSan), `tools/check.sh`, GitHub Actions CI
  - [x] HolyC linter for precedence / `$` / casts / C-isms (`docs/HOLYC_RULES.md`)
  - [ ] QEMU + TempleOS image script, COM1 capture of test results
- [x] Phase 1 core library: `F32 Mem Vec Str Ring Map Time Log` with 125 unit tests;
      F32 checked bit-exact against C across all 2^32 float32 patterns
  - [ ] First compile + `RosTestsRun;` on real TempleOS (check every `VERIFY` comment, e.g.
        `GetTSC`/`cnts.time_stamp_freq` in `Time.HC`, `StrPrintJoin` in `Log.HC`)
- [x] Phase 2 messages
  - [x] 179 interfaces / 24 packages (`interfaces/`), Jazzy definitions + hand-written services
  - [x] `.msg`/`.srv` parser -> runtime type descriptions (`src/Msg/Type.HC`, `Parse.HC`)
  - [x] CDR (XCDR1) serializer/deserializer, LE + BE reading, bounds-checked (`Cdr.HC`)
  - [x] new/free/deep copy, set-by-path (`linear.x`, `poses[2].pose...`), YAML like `ros2 topic echo` (`Msg.HC`)
  - [x] HolyC code generator for typed classes (`Gen.HC`) + generated layout test
  - [x] Golden tests: all 196 message/request/response types decode, re-encode byte-identical,
        copy and print correctly against an independent serializer (rosbags)
  - [x] F64 printing exact (Burger & Dybvig, `Dtoa.HC`), identical to Python repr
  - [x] Jazzy type hashes (RIHS01): `src/Msg/Hash.HC`, checked against rosbags for every message
  - [x] Actions (.action files -> CAct_* classes, golden CDR + hashes from rosbags)
  - [ ] wstring
  - [ ] First compile on real TempleOS (watch: long string literals in TestGolden.HC,
        `offset()` in TestLayout.HC, `FileRead`/`FileWrite` in File.HC)
- [x] Phase 3 graph/executor (`src/Graph/`)
  - [x] Names (relative/absolute/~private, validation), node + global remapping
  - [x] Nodes, publishers, subscriptions with zero-copy shared messages (refcounted), loaned messages
  - [x] QoS: history/depth, reliability, durability (transient local late-joiners), lifespan,
        compatibility rules + incompatible counters
  - [x] Timers, executor (spin, spin_once, spin_some, spin_for, spin_until), one executor per task
  - [x] Services + clients (async callbacks and blocking call), safe when a client disappears
  - [x] Parameters stored as rcl_interfaces/ParameterValue: declare/get/set, types, read-only,
        set-callback, YAML params files + `/**` wildcard, all 6 parameter services, /parameter_events
  - [x] /rosout (rcl_interfaces/Log) per node
  - [x] 110 graph tests; ThreadSanitizer stress test (3 publisher threads + executor thread)
  - [x] Examples: TalkerListener (two TempleOS tasks), AddTwoInts, ParamDemo
  - [x] Actions: server/client over rcl_action's services and topics, cancel policy, result
        expiry, `ros2 action` CLI, turtlesim rotate_absolute; Cyclone DDS completes a goal on us
  - [ ] Multi-threaded executor (worker tasks on other cores)
  - [ ] Creating/destroying entities from a task other than the one spinning that node is not safe yet
- [x] Phase 4 tools (`src/Tools/`, docs/TOOLS.md)
  - [x] node list/info, topic list/info/echo/pub/hz, service list/call, param list/get/set/dump
        (through parameter services), interface list/show/proto
  - [x] Launch: nodes with names, namespaces, remaps, params; one task per node or one shared executor
  - [x] Turtlesim: same topics/services as ROS 2, pen trail, spawn/kill/clear/reset/teleport,
        TempleOS drawing + arrow-key teleop (VERIFY on real install); TurtleSquare launch demo
  - [ ] DolDoc clickable output (topic names that open an echo)
  - [x] ros2 bag record/play/info (Phase 8)
- [x] Phase 5 plugins
  - [x] Registry: create/destroy, methods by name, base-type check (`PluginCreateAs`),
        `PluginClassesFor(base)`, manifests (`PluginDescribe`), replace methods at runtime
  - [x] cpp2hc output registers itself in `PluginInit_<file>()` (runs on load in TempleOS)
  - [x] Converted C++ tutorial plugins compile and run in host tests; `examples/PluginNode.HC`
        picks its plugin from a parameter and swaps it while running
  - [ ] Test `PluginLoad`/`PluginReload` (ExeFile) on a real TempleOS install
- [x] Phase 6 serial bridge: done as SLIP (IP over the serial port), so DDS runs over it unchanged
- [~] Phase 7 networking + RTPS (`src/Dds/`, docs/NETWORK.md)
  - [x] RTPS messages, SPDP + SEDP discovery, best-effort + reliable writers/readers,
        transient local, GAP, DATA_FRAG + NACK_FRAG/HEARTBEAT_FRAG
  - [x] ROS<->DDS bridge (`rt/` names, dds_ type names, ros_discovery_info)
  - [x] Interop test against real Cyclone DDS over loopback with packet loss (tools/ddstest)
  - [x] Services over DDS (rmw_cyclonedds request header), remote nodes/topics/services in the CLI
  - [x] Jazzy RIHS01 type hashes in SEDP user data and bag metadata (match rosbags for every message)
  - [ ] rmw_fastrtps-style services
  - [x] IP stack (Ethernet/ARP/IPv4+fragments/UDP/ICMP/IGMP), tested against Linux over TAP
  - [x] Full stack over our IP stack + multicast against Cyclone DDS (TAP)
  - [x] SLIP serial link (tested over a pty against Linux) - this is also Phase 6, in pure HolyC
  - [~] e1000 driver written (VERIFY on QEMU), `RosNetwork(...)` / `RosNetworkSlip(...)` one-liners
- [~] Phase 8 tf/rviz-lite/rosbag-lite
  - [x] tf2: transforms, buffer with interpolation, lookups across the tree, /tf + /tf_static,
        tf2_echo (`src/Tf/`, docs/TF.md)
  - [x] rosbag2 MCAP record/play/info, read + written against rosbags and mcap
        (`src/Tools/Bag.HC`, tools/bagtest); compressed chunks not supported
