# hctest: run our HolyC on Linux

TempleOS can't run in CI, and you don't always want to boot a VM to find out whether a hash
map works. `hc2c.py` takes our HolyC, inlines its `#include`s, turns it into C, and builds it with
gcc against `hcshim.h` (stand-ins for `MAlloc`, `Print`, `StrCmp`, and other TempleOS built-ins).
Then it runs it. The build uses AddressSanitizer and UBSan, so memory bugs show up right away.

The **same** test files (`tests/*.HC`) run inside TempleOS too, so the host doesn't replace
VM testing. It just catches most logic bugs before you get there.

```
tools/check.sh                                          # everything
python3 tools/hctest/hc2c.py run tests/Run.HC --main RosTestsRun
python3 tools/hctest/hc2c.py lint tests/Run.HC          # HolyC pitfalls only
python3 tools/hctest/hc2c.py c src/Load.HC -o lib.c     # just emit C
```

`f32_check.c` compares `src/Core/F32.HC` with the C compiler's float conversion across all
2^32 float32 bit patterns plus 20 million random doubles (`tools/check.sh full`).

The rules the code has to follow are in [docs/HOLYC_RULES.md](../../docs/HOLYC_RULES.md).

What the host build **can't** tell you: whether TempleOS's compiler accepts a construct, whether
a TempleOS built-in has the name/signature we assumed (anything marked VERIFY), or how timing
behaves on real hardware.
