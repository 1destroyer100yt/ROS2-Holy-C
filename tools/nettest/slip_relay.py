#!/usr/bin/env python3
"""A serial line for the SLIP test: a pty (the "cable") whose other end is a
Linux TUN interface rhcs0 (10.79.0.1). Prints the pty path, then relays
SLIP frames <-> IP packets until killed.
  slip_relay.py --unix PATH   use QEMU's serial port socket instead of a pty
"""
import fcntl
import os
import select
import socket
import struct
import sys
import termios
import tty

TUNSETIFF, IFF_TUN, IFF_NO_PI = 0x400454ca, 0x0001, 0x1000
END, ESC, ESC_END, ESC_ESC = 0xC0, 0xDB, 0xDC, 0xDD


def ifconfig(name, addr):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    pack = lambda a: struct.pack("16sH2s4s8s", name, socket.AF_INET, b"\0\0", socket.inet_aton(a), b"\0" * 8)
    fcntl.ioctl(s, 0x8916, pack(addr))             # SIOCSIFADDR
    fcntl.ioctl(s, 0x891c, pack("255.255.255.0"))  # SIOCSIFNETMASK
    fl = struct.unpack("16sH", fcntl.ioctl(s, 0x8913, struct.pack("16sH", name, 0)))[1]
    fcntl.ioctl(s, 0x8914, struct.pack("16sH", name, fl | 0x1 | 0x40))  # UP | RUNNING


def main():
    tun = os.open("/dev/net/tun", os.O_RDWR)
    fcntl.ioctl(tun, TUNSETIFF, struct.pack("16sH", b"rhcs0", IFF_TUN | IFF_NO_PI))
    ifconfig(b"rhcs0", "10.79.0.1")
    if len(sys.argv) > 2 and sys.argv[1] == "--unix":
        import time
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        for _ in range(300):
            try:
                sock.connect(sys.argv[2])
                break
            except OSError:
                time.sleep(0.1)
        master = sock.fileno()
        print(sys.argv[2], flush=True)
    else:
        master, slave = os.openpty()
        tty.setraw(slave)
        print(os.ttyname(slave), flush=True)
    buf, esc = bytearray(), False
    while True:
        r, _, _ = select.select([tun, master], [], [], 1.0)
        if tun in r:
            pkt = os.read(tun, 65536)
            out = bytearray([END])
            for b in pkt:
                if b == END:
                    out += bytes([ESC, ESC_END])
                elif b == ESC:
                    out += bytes([ESC, ESC_ESC])
                else:
                    out.append(b)
            out.append(END)
            os.write(master, bytes(out))
        if master in r:
            try:
                data = os.read(master, 65536)
            except ConnectionResetError:     # QEMU went away
                return 1
            if not data and len(sys.argv) > 2:
                return 0                     # QEMU closed the serial port
            for b in data:
                if b == END:
                    if buf:
                        os.write(tun, bytes(buf))
                    buf, esc = bytearray(), False
                elif esc:
                    buf.append(END if b == ESC_END else ESC if b == ESC_ESC else b)
                    esc = False
                elif b == ESC:
                    esc = True
                else:
                    buf.append(b)


if __name__ == "__main__":
    sys.exit(main())
