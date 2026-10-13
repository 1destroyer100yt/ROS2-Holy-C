"""An independent DHCP server for the HolyC client: Scapy's DHCP_am on
a TAP device.

  python3 dhcp_server.py rhc2
      waits for the HolyC side (tests/net/NetTap.HC NetTapDhcp) to get
      10.81.0.50 and send a datagram from it; RESULT line, exit status
  python3 dhcp_server.py IFACE --offer 10.82.0.2 --gw 10.82.0.1 --serve 60 [--setup]
      just serve for 60 s (--setup: give IFACE the gw address and bring it up)

Needs root and pip install scapy (and libpcap).
"""
import argparse
import fcntl
import socket
import struct
import sys
import threading
import time

from scapy.all import DHCP_am, Net, conf


def wait_iface(name):
    for _ in range(200):  # whoever creates the TAP device
        try:
            socket.if_nametoindex(name)
            return True
        except OSError:
            time.sleep(0.1)
    return False


def setup(name, ip):
    """ip/24 on the interface, and up (like ip addr add + ip link set up)."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    ifn = name.encode().ljust(16, b"\0")
    addr = struct.pack("16sH2s4s8s", ifn, socket.AF_INET, b"\0\0", socket.inet_aton(ip), b"\0" * 8)
    fcntl.ioctl(s, 0x8916, addr)  # SIOCSIFADDR
    mask = struct.pack("16sH2s4s8s", ifn, socket.AF_INET, b"\0\0", socket.inet_aton("255.255.255.0"),
                       b"\0" * 8)
    fcntl.ioctl(s, 0x891C, mask)  # SIOCSIFNETMASK
    flags = struct.unpack("16sH", fcntl.ioctl(s, 0x8913, struct.pack("16sH", ifn, 0))[:18])[1]
    fcntl.ioctl(s, 0x8914, struct.pack("16sH", ifn, flags | 0x1 | 0x40 | 0x1000))  # UP RUNNING MULTICAST
    s.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("iface")
    ap.add_argument("--offer", default="10.81.0.50")
    ap.add_argument("--gw", default="10.81.0.1")
    ap.add_argument("--serve", type=float, default=0)
    ap.add_argument("--setup", action="store_true")
    a = ap.parse_args()
    conf.verb = 0
    if not wait_iface(a.iface):
        print("RESULT linux: no interface %s" % a.iface)
        return 1
    if a.setup:
        setup(a.iface, a.gw)
    net = a.gw.rsplit(".", 1)[0] + ".0/24"
    server = DHCP_am(iface=a.iface, domain="ros2-holyc.test", pool=Net(a.offer + "/32"),
                     network=net, gw=a.gw, lease_time=3600, renewal_time=1800)
    threading.Thread(target=server, daemon=True).start()
    if a.serve:
        time.sleep(a.serve)
        return 0
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", 9999))
    sock.settimeout(25)
    try:
        data, (ip, _) = sock.recvfrom(1000)
        ok = ip == a.offer and data == ("hello from %s" % a.offer).encode()
        print("RESULT linux: datagram from %s: %r -> %s" % (ip, data, "OK" if ok else "FAIL"))
    except socket.timeout:
        ok = False
        print("RESULT linux: nothing from the HolyC side -> FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
