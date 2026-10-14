#!/bin/sh
# Test the HolyC IP stack (src/Net) against the Linux kernel.
#   tools/nettest/run.sh        over a TAP device (Ethernet + ARP)
#   tools/nettest/run.sh slip   over SLIP on a pty, Linux side a TUN device
#   tools/nettest/run.sh dhcp   our DHCP client against Scapy's DHCP server (TAP)
#   tools/nettest/run.sh slipcom  SLIP over COM2's 16550 UART in a QEMU guest
#                               (the HolyC drives the UART itself; see tools/guest)
# Needs root (or CAP_NET_ADMIN); skipped otherwise.
cd "$(dirname "$0")/../.."
if [ ! -w /dev/net/tun ] || [ "$(id -u)" != 0 ]; then echo "nettest: needs root and /dev/net/tun, skipping"; exit 0; fi
out=$(mktemp)
if [ "$1" = "dhcp" ]; then
  PY=${PYTHON:-python3}
  if ! $PY -c "import scapy" 2>/dev/null; then echo "nettest dhcp: scapy not installed for $PY, skipping"; exit 0; fi
  python3 tools/hctest/hc2c.py run tests/net/NetTap.HC --main NetTapDhcp > "$out" 2>&1 & holyc=$!
  $PY tools/nettest/dhcp_server.py rhc2; server=$?
  if [ -n "$g" ]; then  # QEMU: wait for the guest to finish, then stop it
  i=0; while ! grep -q "RESULT holyc" "$out" && [ $i -lt 300 ]; do sleep 0.1; i=$((i+1)); done
  kill $holyc 2>/dev/null; wait $holyc 2>/dev/null
  grep -q "RESULT holyc: echoed" "$out"; h=$?
else
  wait $holyc; h=$?
fi
  grep RESULT "$out"; rm -f "$out"
  [ $server -eq 0 ] && [ $h -eq 0 ] && echo "nettest dhcp: OK" || { echo "nettest dhcp: FAIL"; exit 1; }
  exit 0
fi
if [ "$1" = "slipcom" ]; then
  . tools/guest/lib.sh
  guest_ok || { echo "nettest slipcom: needs $QEMU, skipping"; exit 0; }
  g=$(mktemp -d)
  guest_kernel "$g" || { echo "nettest slipcom: no kernel, skipping"; rm -rf "$g"; exit 0; }
  guest_initrd tests/net/NetTap.HC SlipComGuest "$g" || exit 1
  "$QEMU" -m 256 -display none -no-reboot -serial file:"$out" \
    -chardev socket,id=c2,path="$g/com2.sock",server=on,wait=on -serial chardev:c2 \
    -kernel "$VMLINUZ" -initrd "$g/initrd.cpio" -append "console=ttyS0 panic=-1 quiet" 2> "$g/qemu.err" &
  holyc=$!
  rel=$(mktemp)
  python3 tools/nettest/slip_relay.py --unix "$g/com2.sock" > "$rel" 2>&1 & relay=$!
  dst=10.79.0.2; src=10.79.0.1; export NET_TIMEOUT=20
elif [ "$1" = "slip" ]; then
  rel=$(mktemp)
  python3 tools/nettest/slip_relay.py > "$rel" 2>&1 & relay=$!
  i=0; while [ ! -s "$rel" ] && [ $i -lt 100 ]; do sleep 0.1; i=$((i+1)); done
  SLIP_TTY=$(head -1 "$rel") python3 tools/hctest/hc2c.py run tests/net/NetTap.HC --main NetSlipEcho > "$out" 2>&1 &
  dst=10.79.0.2; src=10.79.0.1; export NET_TIMEOUT=10
else
  python3 tools/hctest/hc2c.py run tests/net/NetTap.HC --main NetTapEcho > "$out" 2>&1 &
  dst=10.77.0.2; src=10.77.0.1
fi
holyc=$!
i=0
while ! grep -q READY "$out" && [ $i -lt 600 ] && kill -0 $holyc 2>/dev/null; do sleep 0.1; i=$((i+1)); done
if ! grep -q READY "$out"; then
  cat "$out"; [ -n "$g" ] && cat "$g/qemu.err" "$rel"
  echo "nettest: HolyC side didn't start"; kill $holyc 2>/dev/null; exit 1
fi
NET_DST=$dst NET_SRC=$src python3 tools/nettest/client.py; client=$?
if [ -n "$g" ]; then  # QEMU: wait for the guest to finish, then stop it
  i=0; while ! grep -q "RESULT holyc" "$out" && [ $i -lt 300 ]; do sleep 0.1; i=$((i+1)); done
  kill $holyc 2>/dev/null; wait $holyc 2>/dev/null
  grep -q "RESULT holyc: echoed" "$out"; h=$?
else
  wait $holyc; h=$?
fi
[ -n "$relay" ] && kill $relay && rm -f "$rel"
[ -n "$g" ] && rm -rf "$g"
grep RESULT "$out"; rm -f "$out"
[ $client -eq 0 ] && [ $h -eq 0 ] && echo "nettest: OK" || { echo "nettest: FAIL"; exit 1; }
