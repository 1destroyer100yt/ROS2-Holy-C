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

## How it's tested
`tools/ddstest/run.sh` runs the whole HolyC stack against **Cyclone DDS** (the DDS behind
`rmw_cyclonedds`) over loopback, with every 7th outgoing packet deliberately dropped:
- topics both ways (reliable)
- a 100 KB message from us (fragmented) and a 5 KB transient-local message from Cyclone
  (fragmented, delivered to a late joiner)
- Cyclone reads our `ros_discovery_info` and finds our node

`pip install cyclonedds` enables it; `tools/check.sh` and CI run it.

## Not done yet
- Services over DDS (`rq/`/`rr/` topics; `rmw_cyclonedds` and `rmw_fastrtps` use different
  request headers)
- Showing remote topics and nodes in `RosTopicList` / `RosNodeList`
- Jazzy type hashes in endpoint user data. These aren't needed for DDS matching, but
  `ros2 topic info -v` shows them.
- On TempleOS, UDP needs our own IP stack and a NIC driver (`src/Net`). The code above runs
  on Linux sockets in tests.
