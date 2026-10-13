#!/usr/bin/env python3
"""Linux side of the IP stack test: talks to our HolyC stack on rhc0."""
import os
import socket
import struct
import sys
import time

DST = (os.environ.get("NET_DST", "10.77.0.2"), 7777)
SRC = os.environ.get("NET_SRC", "10.77.0.1")
TIMEOUT = float(os.environ.get("NET_TIMEOUT", "2"))  # SLIP over a pty relay is slower


def icmp_sum(b):
    if len(b) % 2:
        b += b"\0"
    s = sum(struct.unpack("!%dH" % (len(b) // 2), b))
    s = (s >> 16) + (s & 0xFFFF)
    s += s >> 16
    return ~s & 0xFFFF


def main():
    ok = {}
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind((SRC, 0))
    s.settimeout(TIMEOUT)

    def echo(data):
        s.sendto(data, DST)
        try:
            got, _ = s.recvfrom(70000)
            return got == data
        except socket.timeout:
            return False

    ok["small"] = echo(b"hello holyc")
    ok["fragmented 30000"] = echo(os.urandom(30000))
    ok["max 65000"] = echo(os.urandom(65000))
    # ping
    r = socket.socket(socket.AF_INET, socket.SOCK_RAW, socket.IPPROTO_ICMP)
    r.settimeout(2.0)
    pkt = struct.pack("!BBHHH", 8, 0, 0, 0x1234, 1) + b"pingdata"
    pkt = pkt[:2] + struct.pack("!H", icmp_sum(pkt)) + pkt[4:]
    r.sendto(pkt, (DST[0], 0))
    ok["ping"] = False
    try:
        end = time.time() + 2
        while time.time() < end:
            data, addr = r.recvfrom(1500)
            if addr[0] == DST[0] and data[20] == 0 and data[24:26] == b"\x12\x34":
                ok["ping"] = True
                break
    except socket.timeout:
        pass
    # burst
    s.settimeout(0.5)
    for i in range(200):
        s.sendto(b"burst %d" % i, DST)
    got = 0
    try:
        while True:
            s.recvfrom(1500)
            got += 1
    except socket.timeout:
        pass
    ok["burst 200 (>=190 back)"] = got >= 190
    s.sendto(b"quit", DST)
    good = all(ok.values())
    print("RESULT linux: " + ", ".join("%s=%s" % kv for kv in ok.items()) + " (burst got %d) -> %s"
          % (got, "OK" if good else "FAIL"), flush=True)
    return 0 if good else 1


if __name__ == "__main__":
    sys.exit(main())
