#!/bin/bash
# The e1000 driver (src/Net/E1000.HC) on QEMU's emulated Intel 82540EM,
# against the Linux kernel. The HolyC (driver + IP stack + echo server,
# tests/net/NetTap.HC E1000Guest) is built as a static Linux program and
# booted as /init of a Linux guest: the driver finds the card on the PCI
# bus and drives its registers and DMA rings itself; Linux only gives it
# memory. On the host side, the card's TAP device gets a DHCP server
# (Scapy) and the same UDP/ICMP test client as tools/nettest.
#
# Needs root, /dev/net/tun, qemu-system-x86_64 ($QEMU), a kernel image
# ($VMLINUZ; else an Ubuntu one is fetched with apt-get download), scapy.
#
#   tools/e1000test/run.sh ros2   instead: the ROS demo node (tests/ros2/Demo.HC)
#     on our stack + e1000 in the guest, and the real ros2 CLI on the host
#     (tools/ros2test/checks.sh; needs ROS 2: ROS_SETUP=... or ros2 on PATH)
#   E1000_DEVICE=e1000e ...       the 82574L instead of the 82540EM
cd "$(dirname "$0")/../.."
PY=${PYTHON:-python3}
source tools/guest/lib.sh
if [ ! -w /dev/net/tun ] || ! guest_ok; then echo "e1000test: needs root, /dev/net/tun and $QEMU, skipping"; exit 0; fi
if ! $PY -c "import scapy" 2>/dev/null; then echo "e1000test: scapy not installed for $PY, skipping"; exit 0; fi
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT
guest_kernel "$tmp" || { echo "e1000test: no kernel image, skipping"; exit 0; }
if [ "$1" = "ros2" ]; then
  [ -n "$ROS_SETUP" ] && source "$ROS_SETUP"
  if ! command -v ros2 >/dev/null; then echo "e1000test ros2: no ROS 2, skipping"; exit 0; fi
  guest_initrd tests/ros2/Demo.HC Ros2DemoE1000 "$tmp" || exit 1
  append="DEMO_SECONDS=${DEMO_SECONDS:-200}"
else
  guest_initrd tests/net/NetTap.HC E1000Guest "$tmp" || exit 1
  append=""
fi
"$QEMU" -m 256 -display none -no-reboot -serial file:"$tmp/console.txt" \
  -kernel "$VMLINUZ" -initrd "$tmp/initrd.cpio" -append "console=ttyS0 panic=-1 quiet $append" \
  -netdev tap,id=n0,ifname=rhe0,script=no,downscript=no -device ${E1000_DEVICE:-e1000},netdev=n0 &
qemu=$!
$PY tools/nettest/dhcp_server.py rhe0 --offer 10.82.0.2 --gw 10.82.0.1 --serve 300 --setup &
dhcp=$!
i=0
while ! grep -q READY "$tmp/console.txt" 2>/dev/null && [ $i -lt 1800 ] && kill -0 $qemu 2>/dev/null; do
  sleep 0.1; i=$((i+1)); done
if [ "$1" = "ros2" ]; then
  fail=0
  if grep -q READY "$tmp/console.txt"; then
    for rmw in ${RMWS:-rmw_fastrtps_cpp rmw_cyclonedds_cpp}; do
      echo "== $rmw, HolyC on e1000"
      ROS_STATIC_PEERS=10.82.0.2 ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET RMW_IMPLEMENTATION=$rmw \
        tools/ros2test/checks.sh | tee "$tmp/checks.log"
      grep -q FAIL "$tmp/checks.log" && fail=1
    done
  else
    fail=1
  fi
  kill $qemu $dhcp 2>/dev/null; wait 2>/dev/null
  grep -aE "e1000:|DHCP|RESULT" "$tmp/console.txt"
  [ $fail = 0 ] && echo "e1000test ros2: OK" || { tail -20 "$tmp/console.txt"; echo "e1000test ros2: FAIL"; exit 1; }
  exit 0
fi
if grep -q READY "$tmp/console.txt"; then
  NET_DST=10.82.0.2 NET_SRC=10.82.0.1 NET_TIMEOUT=10 python3 tools/nettest/client.py; client=$?
else
  client=1
fi
for i in $(seq 1 300); do kill -0 $qemu 2>/dev/null || break; sleep 0.1; done
kill $qemu $dhcp 2>/dev/null; wait 2>/dev/null
grep -aE "e1000:|DHCP|RESULT" "$tmp/console.txt"
if [ $client -eq 0 ] && grep -q "echoed" "$tmp/console.txt"; then echo "e1000test: OK"
else tail -20 "$tmp/console.txt"; echo "e1000test: FAIL"; exit 1; fi
