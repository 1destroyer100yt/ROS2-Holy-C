#!/bin/sh
# Interop test: the HolyC DDS stack against a real Cyclone DDS (the DDS
# ROS 2 ships as rmw_cyclonedds).
#   tools/ddstest/run.sh       over the host's sockets, loopback
#   tools/ddstest/run.sh tap   over OUR IP stack on a TAP device, with
#                              multicast discovery (needs root)
#   needs: pip install cyclonedds   (PYTHON=/path/to/python to pick one)
cd "$(dirname "$0")/../.."
PY=${PYTHON:-python3}
if ! $PY -c "import cyclonedds" 2>/dev/null; then
  echo "ddstest: cyclonedds not installed for $PY, skipping"
  exit 0
fi
if [ "$1" = "tap" ]; then
  if [ ! -w /dev/net/tun ] || [ "$(id -u)" != 0 ]; then echo "ddstest tap: needs root and /dev/net/tun, skipping"; exit 0; fi
  out=$(mktemp)
  python3 tools/hctest/hc2c.py run tests/dds/Interop.HC --main DdsInteropTap > "$out" 2>&1 &
  holyc=$!
  i=0
  while ! grep -q READY "$out" && [ $i -lt 600 ] && kill -0 $holyc 2>/dev/null; do sleep 0.1; i=$((i+1)); done
  PEER_IFACE=10.78.0.1 PEER_PEER=10.78.0.2 PEER_MCAST=true $PY tools/ddstest/peer.py 25; cyc=$?
  wait $holyc; h=$?
  grep RESULT "$out"; rm -f "$out"
else
  out=$(mktemp)
  $PY tools/ddstest/peer.py 25 & peer=$!
  python3 tools/hctest/hc2c.py run tests/dds/Interop.HC --main DdsInterop > "$out" 2>&1
  h=$?
  wait $peer; cyc=$?
  grep RESULT "$out"; rm -f "$out"
fi
[ $h -eq 0 ] && [ $cyc -eq 0 ] && echo "ddstest: OK" || { echo "ddstest: FAIL"; exit 1; }
