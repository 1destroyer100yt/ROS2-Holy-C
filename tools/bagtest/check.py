"""rosbag2 interop check against independent readers/writers.

  check.py write DIR   rosbags writes an MCAP bag (chunked, with a custom
                       message type) for HolyC to read
  check.py read DIR    rosbags and the mcap package read the bag HolyC
                       wrote and check every message

needs: pip install rosbags mcap
"""
import sys
from pathlib import Path

from rosbags.rosbag2 import Reader, StoragePlugin, Writer
from rosbags.typesys import Stores, get_typestore, get_types_from_msg

PAIR = "string name\ngeometry_msgs/Point where\nint32[] ids\n"
T0 = 1_700_000_000 * 10**9


def store():
    ts = get_typestore(Stores.ROS2_JAZZY)
    ts.register(get_types_from_msg(PAIR, "bagtest_msgs/msg/Pair"))
    return ts


def write(path):
    ts = store()
    Twist = ts.types["geometry_msgs/msg/Twist"]
    Vector3 = ts.types["geometry_msgs/msg/Vector3"]
    String = ts.types["std_msgs/msg/String"]
    Pair = ts.types["bagtest_msgs/msg/Pair"]
    Point = ts.types["geometry_msgs/msg/Point"]
    import numpy as np
    with Writer(Path(path), version=9, storage_plugin=StoragePlugin.MCAP) as w:
        tw = w.add_connection("/cmd_vel", Twist.__msgtype__, typestore=ts)
        st = w.add_connection("/chatter", String.__msgtype__, typestore=ts)
        pr = w.add_connection("/pair", Pair.__msgtype__, typestore=ts)
        for i in range(20):
            m = Twist(linear=Vector3(x=i * 0.25, y=0.0, z=0.0),
                      angular=Vector3(x=0.0, y=0.0, z=-i))
            w.write(tw, T0 + i * 50_000_000, ts.serialize_cdr(m, Twist.__msgtype__))
        w.write(st, T0 + 7, ts.serialize_cdr(String(data="from rosbags"), String.__msgtype__))
        p = Pair(name="pair one", where=Point(x=1.5, y=-2.0, z=3.0),
                 ids=np.array([4, 5, 6], dtype=np.int32))
        w.write(pr, T0 + 9, ts.serialize_cdr(p, Pair.__msgtype__))


def read(path):
    ts = get_typestore(Stores.ROS2_JAZZY)
    bad = []
    got = {}
    with Reader(Path(path)) as r:
        if r.message_count != 26:
            bad.append("message_count %d" % r.message_count)
        for conn, stamp, raw in r.messages():
            m = ts.deserialize_cdr(raw, conn.msgtype)
            got.setdefault(conn.topic, []).append((stamp, m))
    tw = got.get("/cmd_vel", [])
    if len(tw) != 20:
        bad.append("twists %d" % len(tw))
    for i, (stamp, m) in enumerate(tw):
        if stamp != T0 + i * 50_000_000 or m.linear.x != i * 0.25 or m.angular.z != -i:
            bad.append("twist %d" % i)
    s = got.get("/chatter", [])
    if len(s) != 1 or s[0][1].data != "from holyc":
        bad.append("chatter")
    big = got.get("/big", [])
    if [m.data for _, m in big] != [c * 400000 for c in "abc"]:
        bad.append("big")
    tf = got.get("/tf", [])
    if len(tf) != 2 or tf[1][1].transforms[0].child_frame_id != "base_link" or \
            tf[1][1].transforms[0].transform.translation.x != 2.0:
        bad.append("tf")
    # Jazzy type hashes in metadata.yaml
    meta = (Path(path) / "metadata.yaml").read_text()
    for t in ("geometry_msgs/msg/Twist", "std_msgs/msg/String", "tf2_msgs/msg/TFMessage"):
        line = "type: %s\n        serialization_format: cdr\n        offered_qos_profiles: []\n" \
               "        type_description_hash: %s\n" % (t, ts.hash_rihs01(t))
        if line not in meta:
            bad.append("type hash of " + t)
    # and with the mcap package's own reader (checks the summary + footer)
    from mcap.reader import make_reader
    f = sorted(Path(path).glob("*.mcap"))[0]
    with open(f, "rb") as fh:
        rd = make_reader(fh)
        summ = rd.get_summary()
        if summ is None or summ.statistics is None or summ.statistics.message_count != 26:
            bad.append("mcap summary")
        if summ is None or len(summ.chunk_indexes) < 2:
            bad.append("expected several chunks")
        n = sum(1 for _ in rd.iter_messages())
        if n != 26:
            bad.append("mcap messages %d" % n)
        schemas = {s.name: s for s in summ.schemas.values()} if summ else {}
        tfs = schemas.get("tf2_msgs/msg/TFMessage")
        if not tfs or b"MSG: geometry_msgs/TransformStamped" not in tfs.data or \
                b"MSG: std_msgs/Header" not in tfs.data:
            bad.append("tf schema text")
    if bad:
        print("bagtest read: FAIL", bad)
        sys.exit(1)
    print("bagtest read: OK (rosbags + mcap read 26 messages)")


if __name__ == "__main__":
    {"write": write, "read": read}[sys.argv[1]](sys.argv[2])
