#!/bin/sh
# Run every check that works without TempleOS. Used by CI too.
#   tools/check.sh          quick (F32 check samples 1 in 97 patterns)
#   tools/check.sh full     exhaustive F32 check (all 2^32, ~30 s)
set -e
cd "$(dirname "$0")/.."
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT

echo "== cpp2hc tests"
python3 -m unittest discover -s tools/cpp2hc/tests -q
echo "== hc2c tests"
python3 -m unittest discover -s tools/hctest/tests -q
echo "== HolyC lint"
python3 tools/hctest/hc2c.py lint tests/Run.HC
python3 tools/hctest/hc2c.py lint tools/hctest/GenMain.HC
echo "== HolyC unit tests (host build)"
python3 tools/hctest/hc2c.py run tests/Run.HC --main RosTestsRun --leaks
echo "== examples run"
python3 tools/hctest/hc2c.py run examples/MsgDemo.HC --main MsgDemo --leaks > /dev/null
echo "== F32 vs C float"
python3 tools/hctest/hc2c.py c src/Load.HC -o "$tmp/lib.c"
gcc -O2 -fwrapv -w -include "$tmp/lib.c" tools/hctest/f32_check.c -o "$tmp/f32" -lm
if [ "$1" = "full" ]; then "$tmp/f32"; else "$tmp/f32" quick; fi
echo "== F64 printing vs Python repr"
python3 tools/hctest/f64fmt_check.py 50000
echo "== generated interface code is up to date"
tools/gen_interfaces.sh
git diff --exit-code --stat -- interfaces tests/TestLayout.HC
echo "== all checks passed"
