#!/bin/sh
# Test the HolyC IP stack (src/Net) against the Linux kernel.
#   tools/nettest/run.sh        over a TAP device (Ethernet + ARP)
#   tools/nettest/run.sh slip   over SLIP on a pty, Linux side a TUN device
# Needs root (or CAP_NET_ADMIN); skipped otherwise.
cd "$(dirname "$0")/../.."
if [ ! -w /dev/net/tun ] || [ "$(id -u)" != 0 ]; then echo "nettest: needs root and /dev/net/tun, skipping"; exit 0; fi
out=$(mktemp)
if [ "$1" = "slip" ]; then
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
if ! grep -q READY "$out"; then cat "$out"; echo "nettest: HolyC side didn't start"; kill $holyc; exit 1; fi
NET_DST=$dst NET_SRC=$src python3 tools/nettest/client.py; client=$?
wait $holyc; h=$?
[ -n "$relay" ] && kill $relay && rm -f "$rel"
grep RESULT "$out"; rm -f "$out"
[ $client -eq 0 ] && [ $h -eq 0 ] && echo "nettest: OK" || { echo "nettest: FAIL"; exit 1; }
