# HolyC rules for this repo

Our HolyC has to do two jobs. It must run on TempleOS, and it must also build as C on Linux,
so `tools/hctest` can test it without a VM. These rules keep both working. The linter
(`python3 tools/hctest/hc2c.py lint tests/Run.HC`) checks the ones marked **(lint)**.

## Things that are different from C and will bite you

1. **Operator precedence (lint).** HolyC binds `& | ^ << >>` tighter than `+ - * /` and
   comparisons. C is the other way around. `a&1==0` means `(a&1)==0` in HolyC and
   `a&(1==0)` in C. Always put parentheses around bitwise expressions mixed with anything else.
2. **`$` is special (lint).** TempleOS source files are DolDoc documents, so a lone `$` starts a
   DolDoc command and wrecks the file. Write `$$` for a literal `$`, for example `"$$FG,RED$$"`.
3. **Casts don't convert numbers (lint).** HolyC's postfix cast `x(I64)` only relabels the bits.
   Use `ToI64(f)` / `ToF64(i)`, or just assign (`I64 i=f;` converts).
4. **No `?:` (lint)**, no `do/while` (lint), no declarations inside `for(...)` (lint).
5. **Only one float type: `F64`.** Float32 conversion lives in `src/Core/F32.HC`.
6. **Never divide a float by zero** to get inf/NaN; it may trap. Use `RosBitsToF64(ROS_F64_INF_BITS)`.
7. **Classes can't be passed or returned by value.** Pass pointers.

## Rules so the host build works

8. Use HolyC types only: `U0 I8 U8 I16 U16 I32 U32 I64 U64 F64 Bool` (lint).
9. No default arguments in `src/` (lint). They're fine in examples and tools.
10. Always call with parentheses: `RosMapNew()`, not `RosMapNew`. HolyC allows both; C doesn't.
11. Declare locals at the top of the function.
12. No top-level statements in library files (they'd run at `#include` time in HolyC). The
    only exception is `Cd(__DIR__);;` in load files, which hc2c drops.
13. Use `extern` for forward declarations.
14. Don't take `&` of a function parameter. Copy it into a local array first (see `RosVecPushI64`).
15. TempleOS-only code goes between `//@host-skip-begin` and `//@host-skip-end`. Host-only code
    goes on `//@host:` lines. Both are just comments to HolyC.

## Naming
- Functions `RosThingVerb`, classes `CRosThing`, constants `ROS_UPPER`, globals `ros_lower`.
- File names short (TempleOS limits them to about 37 characters).
- 2-space indent, like TempleOS.
