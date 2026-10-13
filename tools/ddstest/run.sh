#!/bin/sh
# Interop test: the HolyC DDS stack against a real Cyclone DDS (the DDS
# ROS 2 ships as rmw_cyclonedds), over loopback.
#   needs: pip install cyclonedds   (PYTHON=/path/to/python to pick one)
set -e
cd "$(dirname "$0")/../.."
PY=${PYTHON:-python3}
if ! $PY -c "import cyclonedds" 2>/dev/null; then
  echo "ddstest: cyclonedds not installed for $PY, skipping"
  exit 0
fi
$PY tools/ddstest/peer.py 25 & peer=$!
python3 tools/hctest/hc2c.py run tests/dds/Interop.HC --main DdsInterop
holyc=$?
wait $peer
cyc=$?
[ $holyc -eq 0 ] && [ $cyc -eq 0 ] && echo "ddstest: OK" || { echo "ddstest: FAIL"; exit 1; }
