#!/usr/bin/env python3
"""A RoboClaw (packet serial) simulator, for testing src/Hw/RoboClaw.HC
without the hardware. Motors follow speed commands exactly; encoders
count accordingly.

  roboclaw_sim.py --selfcheck
      the official Basicmicro library (pip install basicmicro) drives the
      simulator over a pty: proves the simulator speaks the real protocol
  roboclaw_sim.py --pty FILE --seconds N
      serve on a new pty (its path is written to FILE) for N seconds
  roboclaw_sim.py --unix PATH --seconds N
      serve on a unix socket someone else listens on (QEMU's serial port)

At the end prints "RESULT sim: ..." with what it saw.
"""
import argparse
import os
import select
import socket
import sys
import time
import tty

VERSION = b"USB Roboclaw 2x7a v4.2.8\n"


def crc16(data, crc=0):
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


class Sim:
    WRITE_LEN = {34: 4, 37: 8, 20: 0}     # data bytes after the command
    READS = {78, 21}

    def __init__(self, addr=0x80):
        self.addr = addr
        self.enc = [0.0, 0.0]
        self.speed = [0, 0]
        self.t = time.monotonic()
        self.buf = b""
        self.packets = 0
        self.bad_crc = 0
        self.speeds_seen = []

    def advance(self):
        now = time.monotonic()
        dt, self.t = now - self.t, now
        for i in (0, 1):
            self.enc[i] += self.speed[i] * dt

    def feed(self, data):
        """Bytes in; returns the bytes to answer."""
        self.buf += data
        out = b""
        while len(self.buf) >= 2:
            addr, cmd = self.buf[0], self.buf[1]
            if addr != self.addr:
                self.buf = self.buf[1:]
                continue
            if cmd in self.READS:
                self.buf = self.buf[2:]
                self.packets += 1
                out += self.read_reply(cmd)
                continue
            n = self.WRITE_LEN.get(cmd)
            if n is None:
                self.buf = self.buf[1:]
                continue
            if len(self.buf) < 2 + n + 2:
                break
            pkt, self.buf = self.buf[:4 + n], self.buf[4 + n:]
            if crc16(pkt[:-2]) != int.from_bytes(pkt[-2:], "big"):
                self.bad_crc += 1          # no ack: the sender retries
                continue
            self.packets += 1
            self.advance()
            d = pkt[2:-2]
            if cmd == 37:
                self.speed = [int.from_bytes(d[0:4], "big", signed=True),
                              int.from_bytes(d[4:8], "big", signed=True)]
                if not self.speeds_seen or self.speeds_seen[-1] != tuple(self.speed):
                    self.speeds_seen.append(tuple(self.speed))
            elif cmd == 34:
                self.speed = [int.from_bytes(d[0:2], "big", signed=True) // 10,
                              int.from_bytes(d[2:4], "big", signed=True) // 10]
            elif cmd == 20:
                self.enc = [0.0, 0.0]
            out += b"\xff"
        return out

    def read_reply(self, cmd):
        head = bytes([self.addr, cmd])
        if cmd == 78:
            self.advance()
            data = b"".join((int(e) & 0xFFFFFFFF).to_bytes(4, "big") for e in self.enc)
        else:
            data = VERSION + b"\0"
        return data + crc16(head + data).to_bytes(2, "big")

    def result(self):
        return "RESULT sim: packets=%d bad_crc=%d speeds=%s enc=%d,%d" % (
            self.packets, self.bad_crc, self.speeds_seen, int(self.enc[0]), int(self.enc[1]))


def serve(fd, sim, seconds, send):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        r, _, _ = select.select([fd], [], [], 0.05)
        if not r:
            continue
        try:
            data = os.read(fd, 4096) if isinstance(fd, int) else fd.recv(4096)
        except OSError:
            data = b""
        if not data:
            if not isinstance(fd, int):
                break           # socket closed
            time.sleep(0.01)    # pty: nobody there yet
            continue
        out = sim.feed(data)
        if out:
            send(out)


def selfcheck():
    """The official library against the simulator."""
    import threading
    from basicmicro import Basicmicro
    master, slave = os.openpty()
    tty.setraw(slave)
    sim = Sim()
    threading.Thread(target=serve, args=(master, sim, 15, lambda b: os.write(master, b)),
                     daemon=True).start()
    bm = Basicmicro(os.ttyname(slave), 38400)
    ok = bm.Open()
    ok = ok and bm.ResetEncoders(0x80)
    ok = ok and bm.SpeedM1M2(0x80, 2000, -1000)
    time.sleep(0.5)
    enc = bm.GetEncoders(0x80)
    ver = bm.ReadVersion(0x80)
    good = (ok and enc[0] and 900 <= enc[1] <= 1100 and 0xFFFFFFFF - 600 <= enc[2] <= 0xFFFFFFFF - 400
            and ver[0] and ver[1].startswith("USB Roboclaw"))
    print("RESULT selfcheck: basicmicro library -> encoders %s version %r -> %s"
          % (enc, ver, "OK" if good else "FAIL"))
    return 0 if good else 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--selfcheck", action="store_true")
    ap.add_argument("--pty")
    ap.add_argument("--unix")
    ap.add_argument("--seconds", type=float, default=30)
    a = ap.parse_args()
    if a.selfcheck:
        return selfcheck()
    sim = Sim()
    import signal

    def stop(*_):
        print(sim.result(), flush=True)
        sys.exit(0)
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    if a.pty:
        master, slave = os.openpty()
        tty.setraw(slave)
        with open(a.pty + ".tmp", "w") as fh:
            fh.write(os.ttyname(slave))
        os.rename(a.pty + ".tmp", a.pty)
        serve(master, sim, a.seconds, lambda b: os.write(master, b))
    elif a.unix:
        s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        for _ in range(300):
            try:
                s.connect(a.unix)
                break
            except OSError:
                time.sleep(0.1)
        serve(s, sim, a.seconds, s.sendall)
    print(sim.result(), flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
