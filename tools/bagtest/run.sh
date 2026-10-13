#!/bin/sh
# rosbag2 interop: the HolyC MCAP writer/reader against rosbags and the
# mcap package (independent Python implementations).
#   needs: pip install rosbags mcap   (PYTHON=/path/to/python to pick one)
cd "$(dirname "$0")/../.."
PY=${PYTHON:-python3}
if ! $PY -c "import rosbags, mcap" 2>/dev/null; then
  echo "bagtest: rosbags/mcap not installed for $PY, skipping"
  exit 0
fi
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT
BAG_DIR=$tmp/holyc_bag python3 tools/hctest/hc2c.py run tests/bag/Interop.HC --main BagInteropWrite --leaks || { echo "bagtest: FAIL (HolyC write)"; exit 1; }
$PY tools/bagtest/check.py read "$tmp/holyc_bag" || { echo "bagtest: FAIL"; exit 1; }
$PY tools/bagtest/check.py write "$tmp/py_bag" || { echo "bagtest: FAIL (rosbags write)"; exit 1; }
BAG_DIR=$tmp/py_bag python3 tools/hctest/hc2c.py run tests/bag/Interop.HC --main BagInteropRead --leaks || { echo "bagtest: FAIL (HolyC read)"; exit 1; }
echo "bagtest: OK"
