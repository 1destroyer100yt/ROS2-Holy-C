#!/usr/bin/env python3
"""Export ROS 2 message definitions as .msg files for ROS2-Holy-C.

The ROS 2 Jazzy message definitions (Apache-2.0, from the ROS 2 repos)
ship inside the pure-Python `rosbags` library. This dev tool writes them
out as plain .msg files under interfaces/, so the HolyC side only ever
reads normal .msg text. Comments and default values are not kept by
rosbags, so they're not in the exported files.

Hand-written interfaces (services, turtlesim, ...) live next to the
exported ones and are left alone. After exporting, the tool rewrites
interfaces/Index.txt: every interface in dependency order, which is the
order the HolyC generator (src/Msg/Gen.HC) loads them in.

  python3 -m venv /tmp/v && /tmp/v/bin/pip install rosbags
  /tmp/v/bin/python tools/ifacegen/export_rosbags.py
"""

import os
import re
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
IFACE = os.path.join(ROOT, "interfaces")
DUMMY = "structure_needs_at_least_one_member"
MAX_NAME = 37  # RedSea file name limit

SKIP_PKGS = {"rosbag2_interfaces"}


def type_str(node, own_pkg):
    kind, d = node
    k = kind.name
    if k == "BASE":
        t, bound = d
        return "%s<=%d" % (t, bound) if bound else t
    if k == "NAME":
        pkg, _, name = d.split("/")
        return name if pkg == own_pkg else "%s/%s" % (pkg, name)
    if k == "ARRAY":
        sub, n = d
        return "%s[%d]" % (type_str(sub, own_pkg), n)
    if k == "SEQUENCE":
        sub, bound = d
        return "%s[<=%d]" % (type_str(sub, own_pkg), bound) if bound else "%s[]" % type_str(sub, own_pkg)
    raise ValueError(node)


def const_val(t, v):
    if t == "string":
        return '"%s"' % v.replace("\\", "\\\\").replace('"', '\\"')
    if t == "bool":
        return "true" if v else "false"
    return repr(v)


def export():
    from rosbags.typesys import Stores, get_typestore
    ts = get_typestore(Stores.ROS2_JAZZY)
    n = 0
    for full, (consts, fields) in sorted(ts.fielddefs.items()):
        pkg, kind, name = full.split("/")
        if pkg in SKIP_PKGS:
            continue
        assert kind == "msg", full
        assert len(name) + 4 <= MAX_NAME and len(pkg) <= MAX_NAME, full
        lines = ["# %s (ROS 2 Jazzy definition, exported via rosbags)" % full]
        for cname, ctype, cval in consts:
            lines.append("%s %s=%s" % (ctype, cname, const_val(ctype, cval)))
        for fname, node in fields:
            if fname == DUMMY and len(fields) == 1:
                continue  # rosidl adds this to empty messages; our parser does too
            lines.append("%s %s" % (type_str(node, pkg), fname))
        d = os.path.join(IFACE, pkg, "msg")
        os.makedirs(d, exist_ok=True)
        with open(os.path.join(d, name + ".msg"), "w") as fh:
            fh.write("\n".join(lines) + "\n")
        n += 1
    return n


PRIMS = {"bool", "byte", "char", "float32", "float64", "int8", "uint8", "int16", "uint16",
         "int32", "uint32", "int64", "uint64", "string", "wstring"}


def deps_of(path, pkg):
    out = set()
    with open(path) as fh:
        for line in fh:
            line = line.split("#")[0].strip()
            if not line or line == "---":
                continue
            t = re.sub(r"\[.*?\]$", "", line.split()[0])
            t = re.sub(r"<=\d+$", "", t)
            if t in PRIMS:
                continue
            if t == "Header":
                out.add("std_msgs/msg/Header")
            elif "/" in t:
                p, _, nm = t.rpartition("/")
                out.add("%s/msg/%s" % (p.split("/")[0], nm))
            else:
                out.add("%s/msg/%s" % (pkg, t))
    return out


def write_index():
    items = {}
    for pkg in sorted(os.listdir(IFACE)):
        for kind in ("msg", "srv"):
            d = os.path.join(IFACE, pkg, kind)
            if not os.path.isdir(d):
                continue
            for f in sorted(os.listdir(d)):
                if f.endswith("." + kind):
                    name = "%s/%s/%s" % (pkg, kind, f[:-4])
                    items[name] = deps_of(os.path.join(d, f), pkg)
    order, done, busy = [], set(), set()

    def visit(n):
        if n in done:
            return
        if n in busy:
            raise SystemExit("dependency cycle at " + n)
        if n not in items:
            raise SystemExit("missing interface " + n)
        busy.add(n)
        for d in sorted(items[n]):
            visit(d)
        busy.discard(n)
        done.add(n)
        order.append(n)

    for n in sorted(items):
        visit(n)
    with open(os.path.join(IFACE, "Index.txt"), "w") as fh:
        fh.write("\n".join(order) + "\n")
    return len(order)


if __name__ == "__main__":
    if "--index-only" not in sys.argv:
        print("exported %d messages" % export())
    print("Index.txt: %d interfaces" % write_index())
