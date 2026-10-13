#!/bin/sh
# Regenerate interfaces/*/Types.HC, interfaces/Load.HC and
# tests/TestLayout.HC from the .msg/.srv files listed in
# interfaces/Index.txt. Runs the HolyC generator (src/Msg/Gen.HC)
# on Linux through tools/hctest. In TempleOS: RosIfaceGen(".");
set -e
cd "$(dirname "$0")/.."
python3 tools/hctest/hc2c.py run tools/hctest/GenMain.HC --main RosGenMain
