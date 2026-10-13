"""An independent DHCP server for the HolyC client: Scapy's DHCP_am on
the TAP device the HolyC side opens. Prints RESULT once the HolyC
side, configured by DHCP, sends a datagram from the address it got.

  python3 dhcp_server.py rhc2        (needs root; pip install scapy)
"""
import socket
import sys
import threading
import time

from scapy.all import DHCP_am, Net, conf

conf.verb = 0
iface = sys.argv[1]
for _ in range(100):  # the HolyC side creates the TAP device
    try:
        socket.if_nametoindex(iface)
        break
    except OSError:
        time.sleep(0.1)
server = DHCP_am(iface=iface, domain="ros2-holyc.test", pool=Net("10.81.0.50/32"),
                 network="10.81.0.0/24", gw="10.81.0.1", lease_time=3600, renewal_time=1800)
threading.Thread(target=server, daemon=True).start()

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind(("0.0.0.0", 9999))
sock.settimeout(25)
try:
    data, (ip, _) = sock.recvfrom(1000)
    ok = ip == "10.81.0.50" and data == b"hello from 10.81.0.50"
    print("RESULT linux: datagram from %s: %r -> %s" % (ip, data, "OK" if ok else "FAIL"))
except socket.timeout:
    ok = False
    print("RESULT linux: nothing from the HolyC side -> FAIL")
sys.exit(0 if ok else 1)
