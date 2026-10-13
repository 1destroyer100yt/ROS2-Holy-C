#!/usr/bin/env python3
"""Check RosStrBufAddF64 (src/Core/Str.HC + Dtoa.HC) prints exactly what
Python's repr() prints, for random doubles of every size plus edge cases."""
import os
import random
import struct
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
N = int(sys.argv[1]) if len(sys.argv) > 1 else 200000

rnd = random.Random(1)
vals = [5e-324, 2.2250738585072014e-308, 2.225073858507201e-308, 1.7976931348623157e308,
        0.1, 0.2, 0.3, 1.0, 1e16, 1e15, 9007199254740993.0, 123456789012345678.0, 1e-5, 1e-4,
        0.30000000000000004, 2.0**-1074 * 3, 1e22, 1e23, 5e-310]
while len(vals) < N:
    pick = rnd.random()
    if pick < 0.4:
        b = rnd.getrandbits(63)
        v = struct.unpack("<d", struct.pack("<Q", b))[0]
        if v != v or v in (float("inf"),):
            continue
    elif pick < 0.7:
        v = rnd.randint(-10**6, 10**6) / 10 ** rnd.randint(0, 6)
    else:
        v = rnd.random() * 10 ** rnd.randint(-30, 30)
    vals.append(v)

tmp = tempfile.mkdtemp()
lib = os.path.join(tmp, "lib.c")
subprocess.check_call([sys.executable, os.path.join(HERE, "hc2c.py"), "c", os.path.join(ROOT, "src", "Load.HC"),
                       "-o", lib])
with open(os.path.join(tmp, "data.txt"), "w") as fh:
    for v in vals:
        fh.write("%016x %s\n" % (struct.unpack("<Q", struct.pack("<d", v))[0], repr(v)))
main = os.path.join(tmp, "main.c")
with open(main, "w") as fh:
    fh.write(r'''
int main(int argc, char **argv) {
  FILE *f = fopen(argv[1], "r"); char want[64]; unsigned long long b; long n = 0, bad = 0;
  while (fscanf(f, "%llx %63s", &b, want) == 2) {
    double d; memcpy(&d, &b, 8);
    CRosStrBuf *sb = RosStrBufNew(); RosStrBufAddF64(sb, d);
    if (strcmp((char *)sb->buf, want)) { if (bad++ < 10) printf("%s: got %s\n", want, sb->buf); }
    RosStrBufDel(sb); n++;
  }
  printf("f64fmt_check: %ld values, %ld differ from Python repr\n", n, bad);
  return bad != 0;
}''')
exe = os.path.join(tmp, "a.out")
subprocess.check_call(["gcc", "-O2", "-fwrapv", "-w", "-include", lib, main, "-o", exe, "-lm"])
sys.exit(subprocess.call([exe, os.path.join(tmp, "data.txt")]))
