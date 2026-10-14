#!/usr/bin/env python3
"""Drive TempleOS in QEMU with no screen: boot it, take screenshots,
type at the HolyC prompt. Used to compile and test ROS2-Holy-C inside
real TempleOS (tools/templeos/run.sh).

  vm = Vm(iso="TempleOS.ISO", disk="ros2holyc.img")
  vm.start()
  vm.screenshot("boot.png")
  vm.type('Dir;\\n')
  vm.stop()

QEMU is $QEMU or qemu-system-x86_64 on PATH. Screenshots are PPM
(QEMU's screendump), converted to PNG when Pillow is installed.
"""
import json
import os
import socket
import subprocess
import tempfile
import time

# QEMU key names for typed characters (US layout)
KEYS = {" ": "spc", "\n": "ret", "\t": "tab", "-": "minus", "=": "equal", "[": "bracket_left",
        "]": "bracket_right", "\\": "backslash", ";": "semicolon", "'": "apostrophe",
        ",": "comma", ".": "dot", "/": "slash", "`": "grave_accent"}
SHIFTED = {"!": "1", "@": "2", "#": "3", "$": "4", "%": "5", "^": "6", "&": "7", "*": "8",
           "(": "9", ")": "0", "_": "minus", "+": "equal", "{": "bracket_left",
           "}": "bracket_right", "|": "backslash", ":": "semicolon", '"': "apostrophe",
           "<": "comma", ">": "dot", "?": "slash", "~": "grave_accent"}


class Vm:
    def __init__(self, iso=None, disk=None, mem=1024, cpus=2, extra=()):
        self.iso, self.disk, self.mem, self.cpus, self.extra = iso, disk, mem, cpus, list(extra)
        self.dir = tempfile.mkdtemp(prefix="templeos")
        self.qmp_path = os.path.join(self.dir, "qmp.sock")
        self.proc = None
        self.sock = None
        self.buf = b""

    def start(self, boot="d"):
        qemu = os.environ.get("QEMU", "qemu-system-x86_64")
        cmd = [qemu, "-m", str(self.mem), "-smp", str(self.cpus), "-display", "none",
               "-qmp", "unix:%s,server,nowait" % self.qmp_path,
               "-qmp", "unix:%s,server,nowait" % os.path.join(self.dir, "ctl.sock"),  # for ctl.py
               "-boot", boot,
               "-rtc", "base=utc", "-serial", "file:%s" % os.path.join(self.dir, "com1.txt")]
        if self.iso:
            cmd += ["-cdrom", self.iso]
        if self.disk:
            cmd += ["-drive", "file=%s,format=raw,if=ide,index=0" % self.disk]
        if os.path.exists("/dev/kvm") and os.access("/dev/kvm", os.W_OK):
            cmd += ["-enable-kvm"]
        cmd += self.extra
        self.proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
        for _ in range(100):
            if os.path.exists(self.qmp_path):
                break
            time.sleep(0.1)
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.sock.connect(self.qmp_path)
        self._read()                          # greeting
        self.cmd("qmp_capabilities")

    def _read(self):
        while b"\n" not in self.buf:
            chunk = self.sock.recv(65536)
            if not chunk:
                raise RuntimeError("QEMU went away: %s" % self.proc.stderr.read().decode()[-500:])
            self.buf += chunk
        line, self.buf = self.buf.split(b"\n", 1)
        return json.loads(line)

    def cmd(self, name, **args):
        self.sock.sendall(json.dumps({"execute": name, "arguments": args}).encode() + b"\n")
        while True:
            r = self._read()
            if "return" in r or "error" in r:
                if "error" in r:
                    raise RuntimeError(r["error"])
                return r["return"]

    def screenshot(self, path):
        ppm = path if path.endswith(".ppm") else path + ".ppm"
        self.cmd("screendump", filename=os.path.abspath(ppm))
        if path.endswith(".ppm"):
            return path
        try:
            from PIL import Image
            Image.open(ppm).save(path)
            os.remove(ppm)
            return path
        except ImportError:
            return ppm

    def screen_bytes(self):
        """The screen as raw PPM bytes (for comparing frames)."""
        p = os.path.join(self.dir, "s.ppm")
        self.cmd("screendump", filename=p)
        with open(p, "rb") as fh:
            return fh.read()

    def key(self, *names, hold_ms=60):
        self.cmd("send-key", keys=[{"type": "qcode", "data": n} for n in names], **{"hold-time": hold_ms})

    def type(self, text, delay=0.03):
        for ch in text:
            if ch in SHIFTED:
                self.key("shift", SHIFTED[ch])
            elif ch.isupper():
                self.key("shift", ch.lower())
            elif ch in KEYS:
                self.key(KEYS[ch])
            else:
                self.key(ch)
            time.sleep(delay)

    def body(self, skip_rows=16):
        """Screen pixels without the top rows (TempleOS's clock lives there)."""
        s = self.screen_bytes()
        parts = s.split(b"\n", 3)               # P6, size, maxval, pixels
        w = int(parts[1].split()[0])
        return parts[3][w * 3 * skip_rows:]

    def wait_quiet(self, seconds=3.0, timeout=600):
        """Wait until the screen (below the clock) stops changing for `seconds`."""
        end = time.time() + timeout
        last, since = None, time.time()
        while time.time() < end:
            s = self.body()
            if s != last:
                last, since = s, time.time()
            elif time.time() - since >= seconds:
                return True
            time.sleep(0.5)
        return False

    def serial(self):
        """What TempleOS wrote to COM1 so far."""
        p = os.path.join(self.dir, "com1.txt")
        return open(p, "rb").read().decode("latin-1") if os.path.exists(p) else ""

    def stop(self):
        try:
            self.cmd("quit")
        except Exception:  # noqa: BLE001
            pass
        if self.proc:
            try:
                self.proc.wait(10)
            except subprocess.TimeoutExpired:
                self.proc.kill()


if __name__ == "__main__":
    import sys
    vm = Vm(iso=sys.argv[1] if len(sys.argv) > 1 else None)
    vm.start()
    time.sleep(float(os.environ.get("WAIT", "5")))
    print(vm.screenshot(os.environ.get("SHOT", "screen.png")))
    vm.stop()
