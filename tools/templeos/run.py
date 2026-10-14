#!/usr/bin/env python3
"""Compile (and run) ROS2-Holy-C inside real TempleOS, in QEMU, headless.

  python3 tools/templeos/run.py TempleOS.ISO 'C:/ROS2-Holy-C/Load.HC' [CMD...]

Boots the ISO (live CD, no install) with a FAT32 disk image of the repo
(tools/mkdisk) as C:. A generated C:/Auto/Auto.HC compiles the file with
ExeFile2 inside try/catch, runs the extra commands, then
sends the terminal's text out of COM1 (tools/templeos/Com.HC); this
script prints it (QEMU writes COM1 to a file). Screenshots go to $SHOTS (default /tmp).
Environment: QEMU, BOOT_WAIT (s, default 40), TIMEOUT (s), CPUS,
STUCK (s the screen may stay unchanged, default 600).
"""
import os
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from vm import Vm  # noqa: E402

REPO = os.path.abspath(os.path.join(HERE, "..", ".."))


def step(vm, text, wait=2.0):
    vm.type(text)
    time.sleep(wait)


def main():
    iso, inc = sys.argv[1], sys.argv[2]
    cmds = sys.argv[3:]
    shots = os.environ.get("SHOTS", tempfile.gettempdir())
    work = tempfile.mkdtemp(prefix="tos")
    img = os.path.join(work, "repo.img")
    auto = os.path.join(work, "Auto")
    os.mkdir(auto)
    with open(os.path.join(auto, "Auto.HC"), "w") as fh:
        fh.write('#include "C:/ROS2-Holy-C/tools/templeos/Com.HC"\n')
        fh.write('try {\n  ExeFile2("%s");\n' % inc)     # ExeFile2: errors come here
        for c in cmds:
            fh.write('  ExePutS2("%s");\n' % c.replace("\\", "\\\\").replace('"', '\\"'))
        # ComDump in both branches: after an exception the shell may not
        # run the next statement
        fh.write('  ComDump;\n} catch {\n  "\\n@@EXCEPTION@@\\n";\n  Fs->catch_except=TRUE;\n  ComDump;\n}\n')
    subprocess.run([sys.executable, os.path.join(REPO, "tools", "mkdisk", "mkdisk.py"), img, REPO, auto],
                   check=True, stdout=subprocess.DEVNULL)
    vm = Vm(iso=iso, disk=img, mem=1024, cpus=int(os.environ.get("CPUS", "1")))
    vm.start()
    out = ""
    try:
        time.sleep(float(os.environ.get("BOOT_WAIT", "40")))
        step(vm, "n", 3)                      # install onto hard drive? no
        step(vm, "n", 3)                      # take tour? no
        step(vm, "Mount;\n", 3)
        step(vm, "C", 2)                      # drive letter
        step(vm, "\n", 2)                     # all partitions
        step(vm, "p", 8)                      # probe
        step(vm, "1\n", 6)                    # the first (primary IDE) disk
        step(vm, "\n", 2)                     # done mounting
        vm.screenshot(os.path.join(shots, "tos_mounted.png"))
        step(vm, 'DocClear;#include "C:/Auto/Auto.HC"\n', 1)
        end = time.time() + float(os.environ.get("TIMEOUT", "1800"))
        print("VM dir (ctl.py %s/ctl.sock shot x.png):" % vm.dir, vm.dir, file=sys.stderr, flush=True)
        last, since, nxt = None, time.time(), 0
        while "@@END@@" not in vm.serial() and time.time() < end:
            time.sleep(2)
            if time.time() >= nxt:            # a progress picture; give up when the screen froze
                nxt = time.time() + 60
                vm.screenshot(os.path.join(shots, "tos_progress.png"))
                b = vm.body()
                if last is None or len(b) != len(last) or sum(x != y for x, y in zip(b, last)) > 600:  # more than the cursor
                    last, since = b, time.time()
                elif time.time() - since > float(os.environ.get("STUCK", "600")):
                    print("screen unchanged for %ss: stuck" % os.environ.get("STUCK", "600"), file=sys.stderr)
                    break
        vm.screenshot(os.path.join(shots, "tos_done.png"))
        out = vm.serial()
    finally:
        vm.stop()
    if "@@BEGIN@@" in out:
        out = out.split("@@BEGIN@@", 1)[1].split("@@END@@", 1)[0]
    else:
        out = "(no output from TempleOS: timed out)\n" + out
    print(out)


if __name__ == "__main__":
    main()
