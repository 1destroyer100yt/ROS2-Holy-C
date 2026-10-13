#!/usr/bin/env python3
"""Cyclone DDS peer for the ROS2-Holy-C interop test (tools/ddstest/run.sh).

Talks to the HolyC side over loopback using ROS 2's DDS naming:
  - reads   rt/chatter      (std_msgs/String), answers on rt/chatter_echo
  - writes  rt/from_cyclone (std_msgs/String) every 50 ms
  - writes  rt/big_string   one 5000-char string (Cyclone sends DATA_FRAGs)
  - reads   rt/holyc_big    a 100000-char string (we send DATA_FRAGs)
  - reads   ros_discovery_info, looks for the node "holyc_node"
Prints one RESULT line and exits 0 if everything was seen.
"""
import os
import sys
import time
from dataclasses import dataclass

os.environ["CYCLONEDDS_URI"] = (
    "<CycloneDDS><Domain id='any'><General><Interfaces><NetworkInterface address='127.0.0.1'/>"
    "</Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><Peers>"
    "<Peer address='127.0.0.1'/></Peers><ParticipantIndex>auto</ParticipantIndex>"
    "<MaxAutoParticipantIndex>9</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>")

from cyclonedds.core import Qos, Policy  # noqa: E402
from cyclonedds.domain import DomainParticipant  # noqa: E402
from cyclonedds.idl import IdlStruct, types  # noqa: E402
from cyclonedds.pub import DataWriter  # noqa: E402
from cyclonedds.sub import DataReader  # noqa: E402
from cyclonedds.topic import Topic  # noqa: E402
from cyclonedds.util import duration  # noqa: E402


@dataclass
class String_(IdlStruct, typename="std_msgs::msg::dds_::String_"):
    data: str


@dataclass
class Gid_(IdlStruct, typename="rmw_dds_common::msg::dds_::Gid_"):
    data: types.array[types.uint8, 16]


@dataclass
class NodeEntitiesInfo_(IdlStruct, typename="rmw_dds_common::msg::dds_::NodeEntitiesInfo_"):
    node_namespace: types.bounded_str[256]
    node_name: types.bounded_str[256]
    reader_gid_seq: types.sequence[Gid_]
    writer_gid_seq: types.sequence[Gid_]


@dataclass
class ParticipantEntitiesInfo_(IdlStruct, typename="rmw_dds_common::msg::dds_::ParticipantEntitiesInfo_"):
    gid: Gid_
    node_entities_info_seq: types.sequence[NodeEntitiesInfo_]


@dataclass
class AddReq_(IdlStruct, typename="example_interfaces::srv::dds_::AddTwoInts_Request_"):
    client: types.uint64   # rmw_cyclonedds request header
    seq: types.int64
    a: types.int64
    b: types.int64


@dataclass
class AddResp_(IdlStruct, typename="example_interfaces::srv::dds_::AddTwoInts_Response_"):
    client: types.uint64
    seq: types.int64
    sum: types.int64


def main():
    timeout = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
    rel = Qos(Policy.Reliability.Reliable(max_blocking_time=duration(milliseconds=100)))
    rel_tl = Qos(Policy.Reliability.Reliable(max_blocking_time=duration(milliseconds=100)),
                 Policy.Durability.TransientLocal, Policy.History.KeepLast(1))
    dp = DomainParticipant(0)
    chatter = DataReader(dp, Topic(dp, "rt/chatter", String_), qos=rel)
    echo = DataWriter(dp, Topic(dp, "rt/chatter_echo", String_), qos=rel)
    from_cy = DataWriter(dp, Topic(dp, "rt/from_cyclone", String_), qos=rel)
    big_w = DataWriter(dp, Topic(dp, "rt/big_string", String_), qos=rel_tl)
    big_r = DataReader(dp, Topic(dp, "rt/holyc_big", String_), qos=rel)
    info = DataReader(dp, Topic(dp, "ros_discovery_info", ParticipantEntitiesInfo_), qos=rel_tl)
    only = DataWriter(dp, Topic(dp, "rt/cyclone_only", String_), qos=rel)
    info_w = DataWriter(dp, Topic(dp, "ros_discovery_info", ParticipantEntitiesInfo_), qos=rel_tl)
    gid = list(dp.guid.bytes) if hasattr(dp.guid, "bytes") else list(bytes(dp.guid))
    info_w.write(ParticipantEntitiesInfo_(gid=Gid_(data=gid), node_entities_info_seq=[
        NodeEntitiesInfo_(node_namespace="/", node_name="cyclone_node", reader_gid_seq=[], writer_gid_seq=[])]))
    srv_rq = DataReader(dp, Topic(dp, "rq/add_two_intsRequest", AddReq_), qos=rel)
    srv_rr = DataWriter(dp, Topic(dp, "rr/add_two_intsReply", AddResp_), qos=rel)
    cli_rq = DataWriter(dp, Topic(dp, "rq/holyc_addRequest", AddReq_), qos=rel)
    cli_rr = DataReader(dp, Topic(dp, "rr/holyc_addReply", AddResp_), qos=rel)
    served, answers = 0, 0
    big_w.write(String_(data="B" * 5000))
    got_chatter, got_big, saw_node, n = 0, 0, False, 0
    end = time.time() + timeout
    while time.time() < end:
        for s in chatter.take(N=100):
            if not hasattr(s, "data"):
                continue  # InvalidSample: a writer went away
            got_chatter += 1
            echo.write(String_(data="echo: " + s.data))
        for s in big_r.take(N=10):
            if hasattr(s, "data") and len(s.data) == 100000 and s.data == "H" * 100000:
                got_big += 1
        for s in info.take(N=10):
            for ni in getattr(s, "node_entities_info_seq", []):
                if ni.node_name == "holyc_node":
                    saw_node = True
        for q in srv_rq.take(N=10):
            if hasattr(q, "a"):
                srv_rr.write(AddResp_(client=q.client, seq=q.seq, sum=q.a + q.b))
                served += 1
        for a in cli_rr.take(N=10):
            if hasattr(a, "sum") and a.client == 777 and a.sum == a.seq + 100:
                answers += 1
        n += 1
        from_cy.write(String_(data="cyclone %d" % n))
        only.write(String_(data="only %d" % n))
        if cli_rq.get_matched_subscriptions():
            cli_rq.write(AddReq_(client=777, seq=n, a=n, b=100))
        if got_chatter >= 5 and got_big and saw_node and served and answers and n > 40:
            break
        time.sleep(0.05)
    ok = got_chatter >= 5 and got_big >= 1 and saw_node and served >= 1 and answers >= 1
    print("RESULT cyclone: chatter=%d holyc_big=%d saw_holyc_node=%s served=%d holyc_answers=%d -> %s"
          % (got_chatter, got_big, saw_node, served, answers, "OK" if ok else "FAIL"), flush=True)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
