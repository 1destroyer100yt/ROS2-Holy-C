#!/usr/bin/env python3
"""Poke a running TempleOS VM (started with vm.py) through its QMP socket.
  ctl.py SOCK shot out.png
  ctl.py SOCK type 'Dir;\\n'        (\\n = Enter, \\e = Esc)
  ctl.py SOCK key ctrl alt t
  ctl.py SOCK quiet [seconds]      wait until the screen stops changing
"""
import socket
import sys
import time

sys.path.insert(0, __import__("os").path.dirname(__file__))
from vm import Vm  # noqa: E402

v = Vm()
__import__("os").rmdir(v.dir)          # not ours: we only talk to a running VM
v.qmp_path = sys.argv[1]
v.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
v.sock.connect(v.qmp_path)
v._read()
v.cmd("qmp_capabilities")
op = sys.argv[2]
if op == "shot":
    print(v.screenshot(sys.argv[3]))
elif op == "type":
    text = sys.argv[3].encode().decode("unicode_escape")
    for part in text.split("\x1b"):
        v.type(part)
        if part is not text.split("\x1b")[-1]:
            v.key("esc")
elif op == "key":
    v.key(*sys.argv[3:])
elif op == "quiet":
    print(v.wait_quiet(float(sys.argv[3]) if len(sys.argv) > 3 else 3.0, timeout=900))
