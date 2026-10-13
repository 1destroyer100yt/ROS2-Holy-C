#!/usr/bin/env python3
"""Write tests/TestGolden.HC: CDR bytes made by an independent ROS 2
serializer (the pure-Python `rosbags` library) for every interface in
interfaces/Index.txt, plus the YAML our RosMsgToYaml should print.

The HolyC test decodes each byte string, re-encodes it (must give the
same bytes) and compares the printed YAML. Values are random but
reproducible (seeded by type name).

  /tmp/v/bin/python tools/ifacegen/make_golden.py
"""

import os
import random

import numpy as np
from rosbags.typesys import Stores, get_typestore, get_types_from_msg

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
IFACE = os.path.join(ROOT, "interfaces")
DUMMY = "structure_needs_at_least_one_member"

INT_RANGE = {
    "int8": (-128, 127), "uint8": (0, 255), "byte": (0, 255), "char": (0, 255),
    "int16": (-32768, 32767), "uint16": (0, 65535),
    "int32": (-2**31, 2**31 - 1), "uint32": (0, 2**32 - 1),
    "int64": (-2**63, 2**63 - 1), "uint64": (0, 2**64 - 1),
}
NP = {"int8": np.int8, "uint8": np.uint8, "byte": np.uint8, "char": np.uint8, "int16": np.int16,
      "uint16": np.uint16, "int32": np.int32, "uint32": np.uint32, "int64": np.int64,
      "uint64": np.uint64, "float32": np.float32, "float64": np.float64, "bool": np.bool_}
F32_VALUES = [0.0, 0.5, -1.25, 3.0, 100.0, 0.25, -7.5, 1024.0]
F64_VALUES = [0.0, 0.1, -2.5, 1e-05, 1e+20, 123.456, 3.141592653589793, -0.001, 42.0, 2.5e-08]
STRINGS = ["", "hi", "it's", "a b: c", "map", "base_link"]


def strip_comments(text):
    out = []
    for line in text.splitlines():
        out.append(line.split("#")[0].rstrip())
    return "\n".join(out) + "\n"


def load_typestore():
    ts = get_typestore(Stores.ROS2_JAZZY)
    names = [ln.strip() for ln in open(os.path.join(IFACE, "Index.txt")) if ln.strip()]
    out = []
    for name in names:
        pkg, kind, base = name.split("/")
        if kind == "action":
            with open(os.path.join(IFACE, pkg, "action", base + ".action")) as fh:
                goal, result, feedback = strip_comments(fh.read()).split("\n---\n")
            pre = "%s/action/%s" % (pkg, base)
            parts = [("_Goal", goal), ("_Result", result), ("_Feedback", feedback),
                     ("_SendGoal_Request", "unique_identifier_msgs/UUID goal_id\n%s_Goal goal\n" % pre),
                     ("_SendGoal_Response", "bool accepted\nbuiltin_interfaces/Time stamp\n"),
                     ("_GetResult_Request", "unique_identifier_msgs/UUID goal_id\n"),
                     ("_GetResult_Response", "int8 status\n%s_Result result\n" % pre),
                     ("_FeedbackMessage", "unique_identifier_msgs/UUID goal_id\n%s_Feedback feedback\n" % pre)]
            for suffix, text in parts:
                ts.register(get_types_from_msg(text, pre + suffix))
                out.append((pre + suffix, pre + suffix))
            continue
        if kind == "msg":
            if name not in ts.fielddefs:
                with open(os.path.join(IFACE, pkg, "msg", base + ".msg")) as fh:
                    ts.register(get_types_from_msg(strip_comments(fh.read()), name))
            out.append((name, name))
        else:
            with open(os.path.join(IFACE, pkg, "srv", base + ".srv")) as fh:
                text = strip_comments(fh.read())
            req, resp = text.split("\n---\n") if "\n---\n" in text else ("", text.split("---", 1)[1])
            for part, suffix in ((req, "_Request"), (resp, "_Response")):
                # rosbags mangles pkg/srv/ names, so register under pkg/msg/
                alias = "%s/msg/%s%s" % (pkg, base, suffix)
                if not part.strip():
                    part = "uint8 %s\n" % DUMMY  # what rosidl adds to empty structs
                ts.register(get_types_from_msg(part, alias))
                out.append(("%s/srv/%s%s" % (pkg, base, suffix), alias))
    return ts, out


class Gen:
    def __init__(self, ts, rnd):
        self.ts, self.rnd = ts, rnd

    def prim(self, t, bound=0):
        r = self.rnd
        if t == "bool":
            return r.random() < 0.5
        if t == "float32":
            return r.choice(F32_VALUES)
        if t == "float64":
            return r.choice(F64_VALUES)
        if t == "string":
            s = r.choice(STRINGS)
            return s[:bound] if bound else s
        lo, hi = INT_RANGE[t]
        return r.choice([lo, hi, 0, 1, r.randint(lo, hi)])

    def value(self, node):
        kind, d = node
        k = kind.name
        if k == "BASE":
            v = self.prim(d[0], d[1])
            if d[0] == "byte" and v > 127:
                v -= 256  # rosbags packs scalar byte as signed; same wire byte
            return v
        if k == "NAME":
            return self.msg(d)
        sub, n = d
        if k == "SEQUENCE":
            n = self.rnd.randint(0, min(2, n) if n else 2)
        if sub[0].name == "BASE":
            items = [self.prim(*sub[1]) for _ in range(n)]
            if sub[1][0] != "string":
                return np.array(items, dtype=NP[sub[1][0]])
            return items
        return [self.value(sub) for _ in range(n)]

    def msg(self, name):
        _consts, fields = self.ts.fielddefs[name]
        return self.ts.types[name](**{f: self.value(node) for f, node in fields})


def yaml_scalar(t, v):
    if t == "string":
        return "'" + v.replace("'", "''") + "'"
    if t == "bool":
        return "true" if v else "false"
    if t in ("float32", "float64"):
        return repr(float(v))
    if t == "byte":
        return str(int(v) & 0xFF)  # ROS 2 byte is an unsigned octet
    return str(int(v))


def yaml_fields(ts, name, m, indent, first_inline, out):
    _c, fields = ts.fielddefs[name]
    for i, (fname, node) in enumerate(fields):
        pre = "" if (first_inline and i == 0) else " " * indent
        kind, d = node
        v = getattr(m, fname)
        if kind.name == "BASE":
            out.append("%s%s: %s\n" % (pre, fname, yaml_scalar(d[0], v)))
        elif kind.name == "NAME":
            out.append("%s%s:\n" % (pre, fname))
            yaml_fields(ts, d, v, indent + 2, False, out)
        else:
            sub = d[0]
            if len(v) == 0:
                out.append("%s%s: []\n" % (pre, fname))
                continue
            out.append("%s%s:\n" % (pre, fname))
            for item in v:
                if sub[0].name == "NAME":
                    out.append(" " * indent + "- ")
                    yaml_fields(ts, sub[1], item, indent + 2, True, out)
                else:
                    out.append(" " * indent + "- " + yaml_scalar(sub[1][0], item) + "\n")


def holyc_str(s):
    return '"%s"' % s.replace("\\", "\\\\").replace('"', '\\"').replace("\n", "\\n").replace("$", "$$")


def main():
    ts, names = load_typestore()
    lines = ["//Generated by tools/ifacegen/make_golden.py. Don't edit.",
             "//CDR bytes from an independent serializer (rosbags) for every interface,",
             "//and the YAML RosMsgToYaml must print for them.", "",
             "U0 TestGolden()", "{", '  RosTestGroup("Golden CDR");']
    for name, rb_name in names:
        rnd = random.Random(name)
        m = Gen(ts, rnd).msg(rb_name)
        data = bytes(ts.serialize_cdr(m, rb_name))
        out = []
        yaml_fields(ts, rb_name, m, 0, False, out)
        lines.append("  RosTestGolden(%s,%s,\n\t%s);" % (holyc_str(name), holyc_str(data.hex()),
                                                         holyc_str("".join(out))))
    lines.append("}")
    with open(os.path.join(ROOT, "tests", "TestGolden.HC"), "w") as fh:
        fh.write("\n".join(lines) + "\n")
    print("TestGolden.HC: %d messages" % len(names))


if __name__ == "__main__":
    main()
