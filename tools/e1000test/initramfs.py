"""Make a tiny initramfs (newc cpio): /init = the given program,
/dev/console, /dev/mem, /proc.   python3 initramfs.py PROGRAM OUT.cpio
"""
import sys


def entry(name, mode, data=b"", rdev=(0, 0)):
    name = name.encode() + b"\0"
    hdr = "070701" + "".join("%08X" % v for v in (
        0, mode, 0, 0, 1, 0, len(data), 0, 0, rdev[0], rdev[1], len(name), 0))
    out = hdr.encode() + name
    out += b"\0" * (-len(out) % 4)
    out += data + b"\0" * (-len(data) % 4)
    return out


prog = open(sys.argv[1], "rb").read()
cpio = (entry("dev", 0o040755) + entry("dev/console", 0o020600, rdev=(5, 1)) +
        entry("dev/mem", 0o020600, rdev=(1, 1)) +
        entry("proc", 0o040555) + entry("init", 0o100755, prog) + entry("TRAILER!!!", 0))
with open(sys.argv[2], "wb") as fh:
    fh.write(cpio + b"\0" * (-len(cpio) % 512))
