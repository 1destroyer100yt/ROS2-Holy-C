#!/usr/bin/env python3
"""The parts of ROS2-Holy-C that only a real TempleOS can show, in one
boot of TempleOS 5.03 in QEMU (no screen):

  serial   the RoboClaw driver on COM2's UART, against tools/hwtest's
           simulator (checked against the official Basicmicro library)
  turtle   TurtleSim in the shell's window: arrow keys go in over QEMU,
           the screenshots must show the turtle and its trail
  rviz     RvizLite in the window showing a latched map (tests/templeos/Screen.HC)
  e1000    the demo node (tests/ros2/Demo.HC) on our IP stack and e1000
           driver, on a TAP device with a DHCP server; the real ros2
           command line on this machine checks it (tools/ros2test/checks.sh),
           with Fast DDS and Cyclone DDS

  sudo python3 tools/templeos/hw.py TempleOS.ISO [serial turtle rviz e1000]

Needs root and /dev/net/tun (for e1000), QEMU=..., scapy and basicmicro
for this python, and ROS 2 (ROS_SETUP=.../setup.bash, or ros2 on PATH) for
e1000. Screenshots go to $SHOTS. Prints "HW name: OK|FAIL" per part.
TempleOS's terminal comes out of COM1 as it's written (Com.HC ComTail).
"""
import os
import shlex
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from vm import Vm  # noqa: E402

REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
PY = sys.executable

# TempleOS's 16 colors (Adam/Gr/GrPalette.HC)
PALETTE = [(0, 0, 0), (0, 0, 0xAA), (0, 0xAA, 0), (0, 0xAA, 0xAA), (0xAA, 0, 0), (0xAA, 0, 0xAA),
           (0xAA, 0x55, 0), (0xAA, 0xAA, 0xAA), (0x55, 0x55, 0x55), (0x55, 0x55, 0xFF),
           (0x55, 0xFF, 0x55), (0x55, 0xFF, 0xFF), (0xFF, 0x55, 0x55), (0xFF, 0x55, 0xFF),
           (0xFF, 0xFF, 0x55), (0xFF, 0xFF, 0xFF)]
BLACK, BLUE, GREEN, LTGRAY, DKGRAY, YELLOW, WHITE = 0, 1, 2, 7, 8, 14, 15


def colors(vm, box=None):
    """The screen as a grid of palette indexes (rows), cropped to box."""
    s = vm.screen_bytes()
    parts = s.split(b"\n", 3)
    w, h = (int(v) for v in parts[1].split())
    px = parts[3]
    idx = {c: i for i, c in enumerate(PALETTE)}
    x0, y0, x1, y1 = box or (0, 0, w, h)
    rows = []
    for y in range(y0, y1):
        row = []
        for x in range(x0, x1):
            o = (y * w + x) * 3
            row.append(idx.get(tuple(px[o:o + 3]), -1))
        rows.append(row)
    return rows


def count(rows, c):
    return sum(r.count(c) for r in rows)


class Run:
    def __init__(self, iso, parts):
        self.parts = parts
        self.shots = os.environ.get("SHOTS", tempfile.gettempdir())
        self.work = tempfile.mkdtemp(prefix="toshw")
        self.procs = []
        self.ok = {}

    def bg(self, cmd, log, env=None):
        fh = open(os.path.join(self.work, log), "w")
        p = subprocess.Popen(cmd, stdout=fh, stderr=subprocess.STDOUT, env=env)
        self.procs.append(p)
        return p

    def log(self, name):
        p = os.path.join(self.work, name)
        return open(p).read() if os.path.exists(p) else ""

    def wait_serial(self, text, timeout, start=0):
        end = time.time() + timeout
        while time.time() < end:
            s = self.vm.serial()
            i = s.find(text, start)
            if i >= 0:
                return i
            if "@@EXCEPTION@@" in s[start:]:     # TempleOS gave up: no point waiting
                return -1
            time.sleep(1)
        return -1

    def auto(self):
        cmds = ['ComTailStart;', 'ComPutS("@@LOADED@@\\n");']
        if "serial" in self.parts:
            cmds += ['HwRoboClawCom;', 'ComPutS("@@SERIAL_DONE@@\\n");']
        if "turtle" in self.parts:
            cmds += ['WinMax;', 'ComPutS("@@TURTLE@@\\n");', 'TurtleSim;', 'ComPutS("@@TURTLE_DONE@@\\n");']
        if "rviz" in self.parts:
            cmds += ['WinMax;', 'ComPutS("@@RVIZ@@\\n");', 'ScreenRviz;', 'ComPutS("@@RVIZ_DONE@@\\n");']
        if "e1000" in self.parts:
            cmds += ['demo_seconds=%d;' % int(os.environ.get("DEMO_SECONDS", "900")),
                     'Ros2DemoE1000;', 'ComPutS("@@E1000_DONE@@\\n");']
        cmds += ['ComTailStop;']
        out = ['#include "C:/ROS2-Holy-C/tools/templeos/Com.HC"', 'ComPutS("@@START@@\\n");', 'try {',
               '  ExeFile2("C:/ROS2-Holy-C/tests/templeos/All.HC");']
        for c in cmds:   # run one by one: they name things the compile above defines
            out.append('  ExePutS2("%s");' % c.replace("\\", "\\\\").replace('"', '\\"'))
        out += ['  ComDump;', '} catch {', '  "\\n@@EXCEPTION@@\\n";', '  Fs->catch_except=TRUE;',
                '  ComDump;', '}', ';', '']
        d = os.path.join(self.work, "Auto")
        os.mkdir(d)
        with open(os.path.join(d, "Auto.HC"), "w") as fh:
            fh.write("\n".join(out))
        return d

    def start(self, iso):
        img = os.path.join(self.work, "repo.img")
        subprocess.run([PY, os.path.join(REPO, "tools", "mkdisk", "mkdisk.py"), img, REPO, self.auto()],
                       check=True, stdout=subprocess.DEVNULL)
        extra = []
        self.com2 = os.path.join(self.work, "com2.sock")
        # COM1 is our output (vm.py); COM2 goes to the RoboClaw simulator
        extra += ["-chardev", "socket,id=c2,path=%s,server=on,wait=off" % self.com2, "-serial", "chardev:c2"]
        if "e1000" in self.parts:
            extra += ["-netdev", "tap,id=n0,ifname=rhe0,script=no,downscript=no",
                      "-device", "e1000,netdev=n0"]
        self.vm = Vm(iso=iso, disk=img, mem=1024, cpus=1, extra=extra)
        self.vm.start()
        if "serial" in self.parts:
            self.bg([PY, os.path.join(REPO, "tools", "hwtest", "roboclaw_sim.py"), "--unix", self.com2,
                     "--seconds", "7200"], "sim.txt")
        if "e1000" in self.parts:
            self.bg([PY, os.path.join(REPO, "tools", "nettest", "dhcp_server.py"), "rhe0", "--offer",
                     "10.82.0.2", "--gw", "10.82.0.1", "--serve", "7200", "--setup"], "dhcp.txt")

    def boot(self):
        vm = self.vm
        time.sleep(float(os.environ.get("BOOT_WAIT", "40")))
        for text, wait in (("n", 3), ("n", 3), ("Mount;\n", 3), ("C", 2), ("\n", 2), ("p", 8),
                           ("1\n", 6), ("\n", 2)):
            vm.type(text)
            time.sleep(wait)
        for _ in range(4):
            vm.type('DocClear;#include "C:/Auto/Auto.HC"\n')
            if self.wait_serial("@@START@@", 90) >= 0:
                return True
            vm.type("\n")
        return False

    def keys(self, *names, gap=1.2):
        for n in names:
            self.vm.key(n)
            time.sleep(gap)

    def turtle(self, at):
        if self.wait_serial("@@TURTLE@@", 300, at) < 0:
            return False, "TurtleSim didn't start"
        time.sleep(15)
        before = colors(self.vm)
        self.vm.screenshot(os.path.join(self.shots, "turtle_start.png"))
        self.keys("left", "up", "up", "right", "up", "up", gap=2.5)
        time.sleep(5)
        after = colors(self.vm)
        self.vm.screenshot(os.path.join(self.shots, "turtle_moved.png"))
        self.vm.key("esc")
        blue, yellow = count(after, BLUE), count(after, YELLOW)
        trail = count(after, WHITE) - count(before, WHITE)
        moved = sum(a != b for ra, rb in zip(before, after) for a, b in zip(ra, rb))
        why = "blue=%d turtle(yellow)=%d new trail(white)=%d changed=%d" % (blue, yellow, trail, moved)
        return blue > 20000 and yellow > 20 and trail > 50, why

    def rviz(self, at):
        if self.wait_serial("@@RVIZ@@", 300, at) < 0:
            return False, "RvizLite didn't start"
        time.sleep(20)
        rows = colors(self.vm)
        self.vm.screenshot(os.path.join(self.shots, "rviz.png"))
        self.vm.key("esc")
        free, wall = count(rows, LTGRAY), count(rows, BLACK)
        # the map is 4 m = 160 px across at 40 px/m: about 150x150 free cells
        why = "map free(ltgray)=%d walls+background(black)=%d grid(dkgray)=%d axes(green)=%d" % (
            free, wall, count(rows, DKGRAY), count(rows, GREEN))
        return free > 15000, why

    def e1000(self, at):
        i = self.wait_serial("READY", 300, at)
        if i < 0:
            s = self.vm.serial()[at:]
            return False, "the demo didn't start: " + s[-300:].replace("\n", " | ")
        setup = os.environ.get("ROS_SETUP", "")
        src = "source %s; " % shlex.quote(setup) if setup else ""
        fails, out = 0, []
        for rmw in os.environ.get("RMWS", "rmw_fastrtps_cpp rmw_cyclonedds_cpp").split():
            r = subprocess.run(["bash", "-c", src + "ROS_STATIC_PEERS=10.82.0.2 "
                                "ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET RMW_IMPLEMENTATION=%s %s" % (
                                    rmw, os.path.join(REPO, "tools", "ros2test", "checks.sh"))],
                               capture_output=True, text=True)
            out.append("== %s\n%s" % (rmw, r.stdout))
            fails += r.stdout.count("FAIL") + (0 if "CHECK" in r.stdout else 1)
        print("".join(out))
        return fails == 0, "%d failed check(s)" % fails

    def main(self, iso):
        self.start(iso)
        try:
            if not self.boot():
                print("HW boot: FAIL (the command never arrived)")
                return 1
            print("compiling in TempleOS...", flush=True)
            at = self.wait_serial("@@LOADED@@", float(os.environ.get("TIMEOUT", "4000")))
            if at < 0:
                self.vm.screenshot(os.path.join(self.shots, "hw_noload.png"))
                print(self.vm.serial()[-2000:])
                print("HW load: FAIL")
                return 1
            if "serial" in self.parts:
                i = self.wait_serial("@@SERIAL_DONE@@", 600, at)
                s = self.vm.serial()[at:i if i > 0 else None]
                line = [ln for ln in s.splitlines() if "RESULT holyc" in ln]
                good = bool(line) and "hw_errors=0" in line[-1] and line[-1].rstrip().endswith("-> 1")
                self.ok["serial"] = (good, (line[-1] if line else "no RESULT line") + "; sim: " +
                                     self.log("sim.txt").strip().replace("\n", " "))
                at = max(at, i)
            if "turtle" in self.parts:
                self.ok["turtle"] = self.turtle(at)
                at = max(at, self.wait_serial("@@TURTLE_DONE@@", 60, at))
            if "rviz" in self.parts:
                self.ok["rviz"] = self.rviz(at)
                at = max(at, self.wait_serial("@@RVIZ_DONE@@", 60, at))
            if "e1000" in self.parts:
                self.ok["e1000"] = self.e1000(at)
                i = self.wait_serial("@@E1000_DONE@@", float(os.environ.get("DEMO_SECONDS", "900")) + 120, at)
                s = self.vm.serial()[at:i if i > 0 else None]
                for ln in s.splitlines():
                    if ln.startswith(("e1000:", "DHCP", "RESULT")):
                        print("  templeos:", ln)
            self.wait_serial("@@END@@", 120)
            self.vm.screenshot(os.path.join(self.shots, "hw_done.png"))
        finally:
            self.vm.stop()
            for p in self.procs:
                p.terminate()
            for p in self.procs:
                try:
                    p.wait(10)
                except subprocess.TimeoutExpired:
                    p.kill()
        bad = 0
        for name in self.parts:
            good, why = self.ok.get(name, (False, "didn't run"))
            print("HW %s: %s (%s)" % (name, "OK" if good else "FAIL", why))
            bad += not good
        return 1 if bad else 0


if __name__ == "__main__":
    parts = sys.argv[2:] or ["serial", "turtle", "rviz", "e1000"]
    sys.exit(Run(sys.argv[1], parts).main(sys.argv[1]))
