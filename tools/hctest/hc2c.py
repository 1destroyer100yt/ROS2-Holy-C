#!/usr/bin/env python3
"""hc2c - compile and run ROS2-Holy-C's HolyC on a normal Linux box.

TempleOS isn't available on a dev machine or in CI, so this tool takes
our HolyC (written in a strict, C-compatible subset), turns it into C,
compiles it with gcc against hcshim.h, and runs it. The same HolyC unit
tests then run on both TempleOS and Linux.

It also lints the HolyC for things that are legal C but wrong or
missing in HolyC (see LINT_RULES and check_precedence).

Usage:
  hc2c.py run tests/Run.HC --main RosTestsRun     # lint, build, run tests
  hc2c.py lint src/Load.HC                        # lint only
  hc2c.py c tests/Run.HC -o out.c                 # just write the C file

Markers understood in .HC files (they're plain comments to HolyC):
  //@host-skip-begin ... //@host-skip-end   code only for TempleOS
  //@host: <C code>                         code only for the host build
"""

import argparse
import os
import re
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))

LINT_RULES = [
    (r"\?", "no ?: operator in HolyC - use if/else"),
    (r"\bdo\s*\{|\bdo\s*$", "avoid do/while - use while"),
    (r"\b(const|static|volatile|unsigned|signed|int|char|void|float|double|long|short|"
     r"struct|typedef|enum\s+\w+\s*\{)\b", "C keyword/type - use HolyC types (I64, U8, F64, U0, ...)"),
    (r"\bfor\s*\(\s*[A-Z]\w*\s+\**\s*\w+\s*=", "no declarations in for(); declare at the top"),
    (r"\b[A-Z]\w*\s+\**\w+\s*\([^)]*=", "no default args in core code (host tests can't use them)"),
]

BITWISE = {"&", "|", "^", "<<", ">>"}
OTHER_BIN = {"+", "-", "*", "/", "%", "<", ">", "<=", ">=", "==", "!="}
OPS = ["<<=", ">>=", "<<", ">>", "<=", ">=", "==", "!=", "&&", "||", "->", "++", "--",
       "+=", "-=", "*=", "/=", "%=", "&=", "|=", "^=",
       "+", "-", "*", "/", "%", "<", ">", "&", "|", "^", "=", "!", "~", ",", ";", "(", ")",
       "[", "]", "{", "}", "."]


def strip_strings(line):
    """Blank out string/char literals and // comments."""
    out, i, n = [], 0, len(line)
    while i < n:
        c = line[i]
        if c in "\"'":
            j = i + 1
            while j < n and line[j] != c:
                j += 2 if line[j] == "\\" else 1
            out.append(c + "x" * max(0, min(j, n) - i - 1) + c)
            i = j + 1
            continue
        if line.startswith("//", i):
            break
        out.append(c)
        i += 1
    return "".join(out)


def tokens(code):
    i, toks = 0, []
    while i < len(code):
        c = code[i]
        if c.isspace():
            i += 1
            continue
        m = re.match(r"[A-Za-z_]\w*|0[xX][0-9a-fA-F]+|\d+\.?\d*(?:[eE][+-]?\d+)?|['\"][^'\"]*['\"]", code[i:])
        if m:
            toks.append(("v", m.group(0)))
            i += m.end()
            continue
        for op in OPS:
            if code.startswith(op, i):
                toks.append(("o", op))
                i += len(op)
                break
        else:
            toks.append(("o", c))
            i += 1
    return toks


def check_precedence(code):
    """HolyC binds & | ^ << >> tighter than + - * / and comparisons,
    C does the opposite. Mixing them without parentheses means the two
    compilers compute different things, so we forbid it."""
    toks = tokens(code)
    stack = [set()]
    prev = None
    for kind, t in toks:
        if t in ("(", "["):
            stack.append(set())
        elif t in (")", "]"):
            if len(stack) > 1:
                bad = stack.pop()
                if bad & BITWISE and bad & OTHER_BIN:
                    return True
        elif t in (",", ";", "=", "+=", "-=", "*=", "/=", "&=", "|=", "^=", "<<=", ">>=", "&&", "||",
                   "{", "}"):
            if stack[-1] & BITWISE and stack[-1] & OTHER_BIN:
                return True
            stack[-1] = set()
        elif kind == "o" and (t in BITWISE or t in OTHER_BIN):
            binary = prev is not None and (prev[0] == "v" or prev[1] in (")", "]", "++", "--"))
            if binary:
                stack[-1].add(t)
        prev = (kind, t)
    return any(s & BITWISE and s & OTHER_BIN for s in stack)


def lint(lines):
    """lines: list of (file, line_no, text, host_only). Returns error strings."""
    errs = []
    prev = None
    for f, no, text, host_only in lines:
        if not host_only:
            code = strip_strings(text).strip()
            if prev and prev[2].endswith('"') and code.startswith('"') or re.search(r'"\s+"', code):
                errs.append("%s:%d: adjacent string literals - write one literal\n    %s" % (f, no, text.strip()))
            prev = (f, no, code)
        if host_only:
            continue
        code = strip_strings(text)
        if code.lstrip().startswith("#"):
            continue
        for pat, why in LINT_RULES:
            if re.search(pat, code):
                errs.append("%s:%d: %s\n    %s" % (f, no, why, text.strip()))
        if re.search(r"(?<!\$)\$(?!\$)", text.replace("$$", "")):
            errs.append("%s:%d: lone '$' - TempleOS files are DolDoc, write $$ for a literal $\n    %s"
                        % (f, no, text.strip()))
        for m in re.finditer(r"(\)|\b\w+)\s*\(\s*(?:[IUF]\d+|U0|Bool|C[A-Z]\w*)\s*\**\s*\)", code):
            if m.group(1) != "sizeof":
                errs.append("%s:%d: postfix cast - avoid in core code, use ToI64/ToF64 or a typed variable\n    %s"
                            % (f, no, text.strip()))
                break
        if check_precedence(code):
            errs.append("%s:%d: mixes bitwise (& | ^ << >>) with other operators - add parentheses "
                        "(HolyC precedence differs from C)\n    %s" % (f, no, text.strip()))
    return errs


def int64_literals(line):
    """HolyC integer literals are 64-bit, C's are int: add LL so
    10*1000000000 means the same thing on both."""
    if line.lstrip().startswith("#"):
        return line
    out, i, n = [], 0, len(line)
    start = 0
    while i < n:
        c = line[i]
        if c in "\"'":
            out.append(re.sub(r"(?<![\w.])(?<![eE][+-])(0[xX][0-9a-fA-F]+|\d+)(?![\w.])", r"\1LL", line[start:i]))
            j = i + 1
            while j < n and line[j] != c:
                j += 2 if line[j] == "\\" else 1
            out.append(line[i:j + 1])
            i = start = j + 1
            continue
        if line.startswith("//", i):
            break
        i += 1
    out.append(re.sub(r"(?<![\w.])(?<![eE][+-])(0[xX][0-9a-fA-F]+|\d+)(?![\w.])", r"\1LL", line[start:i]))
    out.append(line[i:])
    return "".join(out)


def gather(path, seen=None):
    """Inline #includes. Returns list of (file, line_no, text, host_only)."""
    seen = seen if seen is not None else set()
    path = os.path.normpath(path)
    if path in seen:
        return []
    seen.add(path)
    out, skip = [], False
    rel = os.path.relpath(path)
    with open(path, encoding="utf-8") as fh:
        for no, line in enumerate(fh, 1):
            s = line.strip()
            if s.startswith("//@host-skip-begin"):
                skip = True
                continue
            if s.startswith("//@host-skip-end"):
                skip = False
                continue
            if skip:
                continue
            if s.startswith("//@host:"):
                out.append((rel, no, line.split("//@host:", 1)[1].rstrip("\n"), True))
                continue
            m = re.match(r'#include\s+"([^"]+)"', s)
            if m:
                out += gather(os.path.join(os.path.dirname(path), m.group(1)), seen)
                continue
            if re.match(r"Cd\s*\(", s):
                continue
            out.append((rel, no, line.rstrip("\n"), False))
    return out


def to_c(lines, main=None):
    src = []
    for f, no, text, host in lines:
        if not host:
            text = text.replace("$$", "$")  # what the DolDoc file layer does in TempleOS
            text = int64_literals(text)
        src.append('#line %d "%s"\n%s' % (no, f, text))
    text = "\n".join(src)
    # class/union Name { -> typedef + struct/union
    def cls(m):
        kind = "struct" if m.group(1) == "class" else "union"
        if ":" in (m.group(3) or ""):
            raise SystemExit("hc2c: class inheritance (%s) not supported in core code" % m.group(2))
        return "typedef %s %s %s; %s %s%s" % (kind, m.group(2), m.group(2), kind, m.group(2), m.group(4))
    text = re.sub(r"^(class|union)\s+(\w+)(\s*:\s*\w+)?([ \t]*(?://[^\n]*)?\n?(?:#line[^\n]*\n)?\s*\{)",
                  cls, text, flags=re.M)
    text = re.sub(r"\boffset\(\s*(\w+)\s*\.\s*(\w+)\s*\)", r"__builtin_offsetof(\1,\2)", text)
    head = '#include "%s"\n' % os.path.join(HERE, "hcshim.h")
    tail = ""
    if main:
        tail = "\nint main(void) { int r = %s() != 0; fflush(stdout); return r; }\n" % main
    return head + text + tail


def build(entry, main, out_c=None, extra_c=()):
    lines = gather(entry)
    errs = lint(lines)
    if errs:
        sys.stderr.write("\n".join(errs) + "\n")
        sys.stderr.write("hc2c: %d lint error(s)\n" % len(errs))
        return None
    c = to_c(lines, main)
    tmp = tempfile.mkdtemp(prefix="hc2c")
    out_c = out_c or os.path.join(tmp, "out.c")
    with open(out_c, "w") as fh:
        fh.write(c)
    exe = os.path.join(tmp, "a.out")
    cmd = ["gcc", "-std=gnu11", "-O1", "-g", "-fwrapv", "-fno-strict-aliasing",
           "-Wall", "-Werror=implicit-function-declaration", "-Werror=return-type",
           "-Wno-incompatible-pointer-types", "-Wno-int-conversion", "-Wno-pointer-sign",
           "-Wno-unused-function", "-Wno-unused-variable", "-Wno-unused-but-set-variable",
           "-Wno-parentheses", "-Wno-discarded-qualifiers",
           "-fsanitize=address,undefined", "-fno-sanitize-recover=undefined",
           "-o", exe, out_c] + list(extra_c) + ["-lm"]
    r = subprocess.run(cmd)
    if r.returncode != 0:
        return None
    return exe


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("run")
    r.add_argument("entry")
    r.add_argument("--main", required=True, help="HolyC function returning I64 (0 = success)")
    r.add_argument("--extra-c", nargs="*", default=[], help="extra C files to link (host-only checks)")
    r.add_argument("--leaks", action="store_true", help="also fail on memory leaks (LeakSanitizer)")
    l_ = sub.add_parser("lint")
    l_.add_argument("entry")
    c = sub.add_parser("c")
    c.add_argument("entry")
    c.add_argument("--main")
    c.add_argument("-o", "--output", required=True)
    a = ap.parse_args(argv)

    if a.cmd == "lint":
        errs = lint(gather(a.entry))
        sys.stderr.write("\n".join(errs) + ("\n" if errs else ""))
        print("hc2c: %d lint error(s)" % len(errs))
        return 1 if errs else 0
    if a.cmd == "c":
        with open(a.output, "w") as fh:
            fh.write(to_c(gather(a.entry), a.main))
        return 0
    exe = build(a.entry, a.main, extra_c=a.extra_c)
    if not exe:
        return 1
    env = dict(os.environ)
    env.setdefault("ASAN_OPTIONS", "detect_leaks=%d" % (1 if a.leaks else 0))
    return subprocess.run([exe], env=env).returncode


if __name__ == "__main__":
    sys.exit(main())
