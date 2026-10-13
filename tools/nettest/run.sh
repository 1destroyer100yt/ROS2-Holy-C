#!/bin/sh
# Test the HolyC IP stack (src/Net) against the Linux kernel through a
# TAP device. Needs root (or CAP_NET_ADMIN); skipped otherwise.
cd "$(dirname "$0")/../.."
if [ ! -w /dev/net/tun ]; then echo "nettest: no /dev/net/tun access, skipping"; exit 0; fi
out=$(mktemp)
python3 tools/hctest/hc2c.py run tests/net/NetTap.HC --main NetTapEcho > "$out" 2>&1 &
holyc=$!
i=0
while ! grep -q READY "$out" && [ $i -lt 600 ]; do sleep 0.1; i=$((i+1)); done
if ! grep -q READY "$out"; then cat "$out"; echo "nettest: HolyC side didn't start"; kill $holyc; exit 1; fi
python3 tools/nettest/client.py; client=$?
wait $holyc; h=$?
grep RESULT "$out"; rm -f "$out"
[ $client -eq 0 ] && [ $h -eq 0 ] && echo "nettest: OK" || { echo "nettest: FAIL"; exit 1; }
