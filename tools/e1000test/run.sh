#!/bin/bash
# The e1000 driver (src/Net/E1000.HC) on QEMU's emulated Intel 82540EM,
# against the Linux kernel. The HolyC (driver + IP stack + echo server,
# tests/net/NetTap.HC E1000Guest) is built as a static Linux program and
# booted as /init of a Linux guest: the driver finds the card on the PCI
# bus and drives it through its I/O ports itself; Linux only gives it
# memory. On the host side, the card's TAP device gets a DHCP server
# (Scapy) and the same UDP/ICMP test client as tools/nettest.
#
# Needs root, /dev/net/tun, qemu-system-x86_64 ($QEMU), a kernel image
# ($VMLINUZ; else an Ubuntu one is fetched with apt-get download), scapy.
cd "$(dirname "$0")/../.."
PY=${PYTHON:-python3}
QEMU=${QEMU:-qemu-system-x86_64}
if [ ! -w /dev/net/tun ] || [ "$(id -u)" != 0 ]; then echo "e1000test: needs root and /dev/net/tun, skipping"; exit 0; fi
if ! command -v "$QEMU" >/dev/null; then echo "e1000test: no $QEMU, skipping"; exit 0; fi
if ! $PY -c "import scapy" 2>/dev/null; then echo "e1000test: scapy not installed for $PY, skipping"; exit 0; fi
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT
if [ -z "$VMLINUZ" ]; then
  pkg=$(apt-cache search --names-only '^linux-image-unsigned-[0-9.]+-[0-9]+-generic$' | sort -V | tail -1 | cut -d' ' -f1)
  (cd "$tmp" && apt-get download -q "$pkg" >/dev/null 2>&1 && dpkg-deb -x ./*.deb k) || { echo "e1000test: no kernel image, skipping"; exit 0; }
  VMLINUZ=$(ls "$tmp"/k/boot/vmlinuz-*)
fi
python3 tools/hctest/hc2c.py c tests/net/NetTap.HC --main E1000Guest -o "$tmp/guest.c" || exit 1
gcc -std=gnu11 -O1 -fwrapv -fno-strict-aliasing -fms-extensions -w -static "$tmp/guest.c" \
  -o "$tmp/init" -lm -lpthread || exit 1
python3 tools/e1000test/initramfs.py "$tmp/init" "$tmp/initrd.cpio"
"$QEMU" -m 256 -display none -no-reboot -serial file:"$tmp/console.txt" \
  -kernel "$VMLINUZ" -initrd "$tmp/initrd.cpio" -append "console=ttyS0 panic=-1 quiet" \
  -netdev tap,id=n0,ifname=rhe0,script=no,downscript=no -device e1000,netdev=n0 &
qemu=$!
$PY tools/nettest/dhcp_server.py rhe0 --offer 10.82.0.2 --gw 10.82.0.1 --serve 120 --setup &
dhcp=$!
i=0
while ! grep -q READY "$tmp/console.txt" 2>/dev/null && [ $i -lt 1800 ] && kill -0 $qemu 2>/dev/null; do
  sleep 0.1; i=$((i+1)); done
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
