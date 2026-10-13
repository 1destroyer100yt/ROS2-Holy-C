# Networking: talking to real ROS 2

`src/Dds/` is a small DDS implementation in HolyC. It speaks RTPS, the wire protocol under
every DDS vendor, so TempleOS nodes can talk to ROS 2 nodes on other machines.

```
RosDdsStart(0,TRUE);                       //domain 0, multicast discovery
RosDdsAddPeer(ros_dds,RosIpParse("10.0.2.2"));   //optional: also look here (unicast)
...                                        //every publisher/subscription is now on the network
RosDdsStop;
```

## What's implemented
- **Discovery:** SPDP (participants, multicast and/or unicast peers) and SEDP (publications and
  subscriptions: topic, type, reliability, durability, locators), with participant leases and
  goodbye messages.
- **Writers and readers:** best effort and reliable (HEARTBEAT, ACKNACK, GAP), transient-local
  late joiners, volatile writers sending GAPs for old samples, out-of-order buffering.
- **Big messages:** DATA_FRAG both ways. Missing fragments are repaired with NACK_FRAG and
  HEARTBEAT_FRAG, which Cyclone DDS relies on.
- **ROS 2 conventions:** `rt/` topic prefix, `pkg::msg::dds_::Type_` type names, XCDR1 payloads,
  and `ros_discovery_info` so `ros2 node list` shows our nodes. The participant's user data is
  `enclave=/;`.
- **Jazzy type hashes:** every endpoint's user data carries `typehash=RIHS01_...;`, like rmw
  does in Jazzy (`ros2 topic info -v` shows it). The hashes come from `src/Msg/Hash.HC`
  (SHA-256 in `src/Core/Sha256.HC`) and match rosbags for all 162 message types we ship
  (`tests/TestHashGolden.HC`, made by `tools/ifacegen/make_hashes.py`). Service hashes follow
  rosidl's request/response/event layout but haven't been checked against a real install yet.
  Remote endpoints' hashes are kept in `CRtpsRemoteEp.type_hash`.
- **Services:** `rq/<name>Request` and `rr/<name>Reply`, both ways, in both RMW dialects:
  `rmw_cyclonedds` puts a {client id, sequence number} header in the payload; `rmw_fastrtps`
  (Jazzy's default) sends plain payloads and matches replies through a related sample identity
  in inline QoS (PID 0x0083). We answer each request the way it came and call a service the way
  its machine's DDS does (Fast DDS = vendor eProsima in SPDP). A client counts as ready only once the
  remote request reader has acknowledged us, which avoids the DDS discovery race that drops
  the first request.
- **CLI sees other machines:** `RosNodeList`, `RosTopicList`, `RosServiceList` and
  `RosTopicEcho` include remote nodes, topics and services, read from SEDP and
  `ros_discovery_info`.

## How it's tested
`tools/ros2test/run.sh` runs against a **real ROS 2 Jazzy** (installed from RoboStack with
micromamba, see `tools/ros2test/env.yml`; CI does the same), once with `rmw_fastrtps_cpp` and
once with `rmw_cyclonedds_cpp`:
- the ros2 command line on a HolyC node: `ros2 node list`, `ros2 topic echo`, `ros2 topic pub`
  into a HolyC listener, `ros2 topic info -v` (the type hash), `ros2 service call`,
  `ros2 action send_goal -f`, `ros2 param get`
- HolyC calling Python nodes: demo_nodes_py's `add_two_ints_server`, action_tutorials_py's
  `fibonacci_action_server` (its action type defined at run time from the .action text),
  listening to the Python talker, and seeing them in `RosNodeListStr`
- rosbag2 both ways: HolyC reads what `ros2 bag record -s mcap` wrote; `ros2 bag info` and
  `ros2 bag play` work on a bag HolyC wrote

`tools/ddstest/run.sh` runs the whole HolyC stack against **Cyclone DDS** (the DDS behind
`rmw_cyclonedds`) over loopback, with every 7th outgoing packet deliberately dropped:
- topics both ways (reliable)
- service calls both ways; Cyclone's node, topic and service show up in our CLI
- a 100 KB message from us (fragmented) and a 5 KB transient-local message from Cyclone
  (fragmented, delivered to a late joiner)
- Cyclone reads our `ros_discovery_info` and finds our node
- Cyclone sees the right type hash in our publication's user data

`pip install cyclonedds` enables it; `tools/check.sh` and CI run it.

## Our own IP stack (`src/Net/`)
TempleOS has no networking, so `src/Net/Net.HC` is a small IPv4 stack written in HolyC. It covers
Ethernet, ARP (with a queue of packets waiting for an answer), IPv4 with fragmentation and
reassembly, UDP, ICMP echo, and IGMPv2 for multicast. Links plug in underneath it: a network
card driver, a serial line, or a Linux TAP device in tests.

- `tools/nettest/run.sh` tests it against the **Linux kernel** over TAP: ARP, ping, UDP, 30 KB and
  65 KB datagrams (IP fragmentation both ways), and a burst of 200 datagrams.
- `tools/ddstest/run.sh tap` runs the **whole stack** (ROS nodes → DDS → RTPS → our UDP/IP/ARP/IGMP
  → TAP) against Cyclone DDS on the Linux side, with **multicast discovery**. Everything above the
  network card is the code TempleOS runs.

Both need root for the TAP device. CI runs them with sudo.

## Links
- **e1000** (`src/Net/E1000.HC`): QEMU's default card and VirtualBox's "PRO/1000 MT". It's polled
  and uses the memory-mapped registers (QEMU doesn't implement the card's I/O-port window).
  `tools/e1000test/run.sh` runs it on QEMU's emulated card: the HolyC is built as a static Linux
  program and booted as `/init` of a Linux guest, where it finds the card on the PCI bus itself,
  gets an address from a DHCP server (Scapy) and passes the same UDP/ICMP suite as the TAP test,
  against the host's Linux kernel. On TempleOS, check the `VERIFY` notes (BAR0 mapping).
- **SLIP** (`src/Net/Slip.HC`): IP over a serial port (COM1 at 115200), for machines without a
  supported card. Tested end to end: `tools/nettest/run.sh slip` runs our stack over a pty, with
  a Linux TUN interface on the other end.
- **TAP** (`src/Net/Tap.HC`): Linux only, for tests.

## Running it on TempleOS (QEMU)
On Linux, give QEMU a TAP interface:
```
sudo ip tuntap add dev tap0 mode tap user $USER
sudo ip addr add 10.0.3.1/24 dev tap0 && sudo ip link set tap0 up
qemu-system-x86_64 -m 2048 -smp 2 -enable-kvm -drive file=TempleOS.qcow2 \
  -netdev tap,id=n0,ifname=tap0,script=no,downscript=no -device e1000,netdev=n0
```
In TempleOS:
```
#include "~/ROS2-Holy-C/Load.HC"
RosNetwork("10.0.3.2","255.255.255.0","10.0.3.1",0);   //e1000 + DDS on domain 0
#include "~/ROS2-Holy-C/examples/TalkerListener.HC"
TalkerListener;
```
Then on Linux, `ros2 topic echo /chatter` should show TempleOS talking (domain 0, any RMW).
Multicast discovery works over the TAP interface. Linux is also added as a unicast peer.

Over a serial line instead: start QEMU with `-serial pty`. Then on Linux run
`sudo slattach -p slip -s 115200 /dev/pts/N` and
`sudo ip addr add 10.0.4.1 peer 10.0.4.2 dev sl0 && sudo ip link set sl0 up`
(or use `tools/nettest/slip_relay.py`, which makes a TUN interface). In TempleOS:
`RosNetworkSlip(0x3F8,"10.0.4.2","10.0.4.1",0);`

Getting the files into TempleOS isn't covered here yet. You need a disk image that TempleOS
can read, and long file names matter (`TestGolden.HC` etc.).

## Not done yet
- Running the e1000 driver on a real TempleOS: it works on QEMU's e1000 from a Linux guest
  (`tools/e1000test`), but TempleOS's mapping of BAR0 and its port I/O names are unchecked.
