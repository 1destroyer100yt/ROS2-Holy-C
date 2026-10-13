#!/usr/bin/env python3
"""cpp2hc - turn C++ ROS 2 pluginlib plugins into HolyC for ROS2-Holy-C.

This tool is dumb on purpose. It does not try to understand all of C++.
It rewrites the common, boring parts with simple, predictable rules and
puts a //TODO(cpp2hc): marker on every line it is not sure about, so the
output is never silently wrong.

Usage:
  cpp2hc.py convert square.hpp square.cpp -o Square.HC [--strict] [--exact-ints]
  cpp2hc.py manifest plugins.xml -o Manifest.HC

Standard library only. Python 3.8+.
"""

import argparse
import os
import re
import sys
import textwrap
import xml.etree.ElementTree as ET

TODO = "//TODO(cpp2hc): "

# ---------------------------------------------------------------- lexing


def strip_comments(src):
    """Remove // and /* */ comments, leaving string and char literals alone."""
    out = []
    i, n = 0, len(src)
    while i < n:
        c = src[i]
        if c in "\"'":
            j = skip_literal(src, i)
            out.append(src[i:j])
            i = j
        elif src.startswith("//", i):
            while i < n and src[i] != "\n":
                i += 1
        elif src.startswith("/*", i):
            j = src.find("*/", i + 2)
            j = n if j < 0 else j + 2
            out.append("\n" * src.count("\n", i, j))
            i = j
        else:
            out.append(c)
            i += 1
    return "".join(out)


def skip_literal(s, i):
    """s[i] is a quote. Return index just after the closing quote."""
    q = s[i]
    i += 1
    while i < len(s):
        if s[i] == "\\":
            i += 2
            continue
        if s[i] == q:
            return i + 1
        i += 1
    return len(s)


def match_close(s, i, open_ch="{", close_ch="}"):
    """s[i] == open_ch. Return index of the matching close_ch."""
    depth = 0
    while i < len(s):
        c = s[i]
        if c in "\"'":
            i = skip_literal(s, i)
            continue
        if c == open_ch:
            depth += 1
        elif c == close_ch:
            depth -= 1
            if depth == 0:
                return i
        i += 1
    raise ValueError("unbalanced %s%s" % (open_ch, close_ch))


def split_top(s, sep=",", angles=False):
    """Split on sep where not nested in () [] {} or literals.

    angles=True also treats <...> as nesting (use it for types, not
    for expressions, where < and > are comparisons).
    """
    parts, depth, cur, i = [], 0, [], 0
    while i < len(s):
        c = s[i]
        if c in "\"'":
            j = skip_literal(s, i)
            cur.append(s[i:j])
            i = j
            continue
        if s.startswith(sep, i) and depth == 0:
            parts.append("".join(cur))
            cur = []
            i += len(sep)
            continue
        if s.startswith("->", i):
            cur.append("->")
            i += 2
            continue
        if c in "([{" or (angles and c == "<"):
            depth += 1
        elif c in ")]}" or (angles and c == ">"):
            depth -= 1
        cur.append(c)
        i += 1
    parts.append("".join(cur))
    return parts


def sub_cout(fn, b):
    """Find 'std::cout << ... ;' statements (string aware) and replace them."""
    out, i = [], 0
    pat = re.compile(r"(?:\bstd::)?\bc(?:out|err)\s*(?=<<)")
    while True:
        mt = pat.search(b, i)
        if not mt:
            out.append(b[i:])
            return "".join(out)
        j = mt.end()
        while j < len(b) and b[j] != ";":
            if b[j] in "\"'":
                j = skip_literal(b, j)
                continue
            j += 1
        out.append(b[i:mt.start()])
        out.append(fn(re.match(r"(.*)", b[mt.end():j], re.S)))
        i = j + 1


def code_segments(s):
    """Yield (is_code, text) pieces so rewrites can skip string literals."""
    i, start = 0, 0
    while i < len(s):
        if s[i] in "\"'":
            if i > start:
                yield True, s[start:i]
            j = skip_literal(s, i)
            yield False, s[i:j]
            i = start = j
        else:
            i += 1
    if start < len(s):
        yield True, s[start:]


def sub_code(pattern, repl, s, flags=0):
    """re.sub, but only outside string/char literals."""
    return "".join(re.sub(pattern, repl, t, flags=flags) if code else t
                   for code, t in code_segments(s))


def top_level_chunks(s):
    """Split a block of declarations into top-level items.

    Each item ends with ';' at depth 0, or with a '{...}' block
    (plus an optional trailing ';').
    """
    items, i, start, n = [], 0, 0, len(s)
    paren = 0
    while i < n:
        c = s[i]
        if c in "\"'":
            i = skip_literal(s, i)
            continue
        if c == "(":
            paren += 1
        elif c == ")":
            paren -= 1
        elif c == ";" and paren == 0:
            items.append(s[start:i + 1])
            start = i + 1
        elif c == "{" and paren == 0:
            j = match_close(s, i)
            k = j + 1
            m = re.match(r"\s*;", s[k:])
            # "= {1, 2};" initializers and class bodies end with ';'
            if m:
                k += m.end()
            items.append(s[start:k])
            start = i = k
            continue
        elif c == "#" and s[start:i].strip() == "":
            j = s.find("\n", i)
            j = n if j < 0 else j
            items.append(s[start:j])
            start = i = j
            continue
        i += 1
    if s[start:].strip():
        items.append(s[start:])
    return [it.strip() for it in items if it.strip()]


# ---------------------------------------------------------------- types

INT_MAP_FAST = "I64"


def type_rules(exact_ints):
    i32 = "I32" if exact_ints else "I64"
    u32 = "U32" if exact_ints else "U64"
    i16 = "I16" if exact_ints else "I64"
    u16 = "U16" if exact_ints else "U64"
    return [
        (r"\bunsigned\s+long\s+long(\s+int)?\b", "U64"),
        (r"\bunsigned\s+long(\s+int)?\b", "U64"),
        (r"\bunsigned\s+int\b", u32),
        (r"\bunsigned\s+short(\s+int)?\b", u16),
        (r"\bunsigned\s+char\b", "U8"),
        (r"\bsigned\s+char\b", "I8"),
        (r"\blong\s+long(\s+int)?\b", "I64"),
        (r"\blong\s+double\b", "F64"),
        (r"\blong\s+int\b", "I64"),
        (r"\bshort(\s+int)?\b", i16),
        (r"\b(std::)?uint64_t\b", "U64"),
        (r"\b(std::)?int64_t\b", "I64"),
        (r"\b(std::)?uint32_t\b", "U32"),
        (r"\b(std::)?int32_t\b", "I32"),
        (r"\b(std::)?uint16_t\b", "U16"),
        (r"\b(std::)?int16_t\b", "I16"),
        (r"\b(std::)?uint8_t\b", "U8"),
        (r"\b(std::)?int8_t\b", "I8"),
        (r"\b(std::)?size_t\b", "U64"),
        (r"\b(std::)?ssize_t\b", "I64"),
        (r"\bstd::string\b", "U8 *"),
        (r"\bdouble\b", "F64"),
        (r"\bfloat\b", "F64"),
        (r"\bbool\b", "Bool"),
        (r"\bchar\b", "U8"),
        (r"\blong\b", "I64"),
        (r"\bunsigned\b", u32),
        (r"\bint\b", i32),
        (r"\bvoid\b", "U0"),
    ]


PRIM_CAST = (r"unsigned\s+long\s+long|unsigned\s+long|unsigned\s+int|unsigned\s+char|long\s+long|"
             r"(?:std::)?u?int(?:8|16|32|64)_t|(?:std::)?size_t|double|float|bool|char|short|long|int|unsigned")

HC_PRIMS = {"U0", "U8", "I8", "U16", "I16", "U32", "I32", "U64", "I64",
            "F64", "Bool"}

# things that are a sure sign of C++ we did not convert
LEFTOVER_CHECKS = [
    (r"\btemplate\s*<", "template - HolyC has no templates, write one version per type"),
    (r"\bstd::(vector|deque|list)\b", "STL sequence container - use src/Core/Vec.HC"),
    (r"\bstd::(map|unordered_map|set|unordered_set)\b", "STL associative container - use src/Core/Map.HC"),
    (r"\bstd::(shared_ptr|unique_ptr|weak_ptr|make_shared|make_unique)\b",
     "smart pointer - use plain pointers and an explicit _Del"),
    (r"\bstd::function\b", "std::function - use a function pointer"),
    (r"\bstd::(thread|mutex|lock_guard|unique_lock|atomic)\b",
     "threading - use Spawn/LBts/LBtr (see PLAN.md Exec)"),
    (r"\brclcpp::\w+", "rclcpp API - map to the rhc API by hand (PLAN.md phase 3)"),
    (r"\bauto\b", "auto - write the real type"),
    (r"\boperator\s*[^\w\s]", "operator overload - make it a named function"),
    (r"\[[&=]?\s*(this)?\s*\]\s*\(", "lambda - make it a named function"),
    (r"\bdynamic_cast\b", "dynamic_cast - no RTTI, check type by hand"),
    (r"\btypename\b|\btypeid\b|\bdecltype\b", "type magic - write the real type"),
    (r"\bthrow\b", "throw - HolyC throws I64 codes: throw('Name');"),
    (r"\bcatch\s*\(", "catch - HolyC catch has no type, check Fs->except_ch"),
    (r"\bstd::\w+", "std:: call not converted"),
    (r"\b\w+::\w+", "'::' scope left over - rename or qualify by hand"),
    (r"(?<![\w>])\w+\s*<\s*[\w:]+(\s*,\s*[\w:]+)*\s*>\s*[\w(]", "template use"),
    (r"\bBAD_ARGS\(", "math call with unexpected arguments"),
    (r"\?", "?: operator - HolyC has none, use if/else"),
    (r"&&", None),  # logical and is fine; listed so the ref check below skips it
]

MATH_MAP = {
    "sqrt": "Sqrt", "pow": "Pow", "sin": "Sin", "cos": "Cos", "tan": "Tan",
    "atan": "ATan", "exp": "Exp", "log": "Ln", "log10": "Log10",
    "floor": "Floor", "ceil": "Ceil", "round": "Round", "fabs": "Abs",
    "abs": "Abs", "min": "CppMin", "max": "CppMax", "fmin": "CppMin",
    "fmax": "CppMax", "hypot": None, "atan2": None,
}


def hc_cast(expr, t):
    """C++ cast -> HolyC. HolyC's postfix cast x(T) only relabels the
    bits, so number conversions must use ToI64/ToF64/ToBool."""
    t = t.strip()
    if t == "F64":
        return "ToF64(%s)" % expr
    if t == "Bool":
        return "ToBool(%s)" % expr
    if t in HC_PRIMS and t != "U0":
        return "ToI64(%s)" % expr
    return "(%s)(%s)" % (expr, t)


# ---------------------------------------------------------------- model

class Field:
    def __init__(self, ctype, name, init=None, static=False):
        self.ctype, self.name, self.init, self.static = ctype, name, init, static


class Method:
    def __init__(self, cls, name, ret, params, body, *, ctor=False, dtor=False,
                 static=False, virtual=False, pure=False, init_list=""):
        self.cls, self.name, self.ret, self.params = cls, name, ret, params
        self.body, self.ctor, self.dtor = body, ctor, dtor
        self.static, self.virtual, self.pure = static, virtual, pure
        self.init_list = init_list
        self.hc_name = None


class Klass:
    def __init__(self, name, qual, bases):
        self.name, self.qual, self.bases = name, qual, bases
        self.fields, self.methods = [], []
        self.notes = []

    def all_fields(self, classes):
        out = []
        for b in self.bases:
            if b in classes:
                out += classes[b].all_fields(classes)
        return out + [f for f in self.fields if not f.static]

    def find_method(self, name, classes):
        for m in self.methods:
            if m.name == name and not m.ctor and not m.dtor and m.body is not None:
                return m
        for m in self.methods:
            if m.name == name and not m.ctor and not m.dtor:
                return m
        for b in self.bases:
            if b in classes:
                r = classes[b].find_method(name, classes)
                if r:
                    return r
        return None


# ---------------------------------------------------------------- converter

class Converter:
    def __init__(self, exact_ints=False):
        self.rules = type_rules(exact_ints)
        self.classes = {}       # short name -> Klass
        self.order = []         # class names in definition order
        self.funcs = []         # free functions: Method with cls=None
        self.globals = []       # translated global lines
        self.exports = []       # (derived, base) from PLUGINLIB_EXPORT_CLASS
        self.enum_types = set()
        self.defs = []          # out-of-line method definitions (cls, chunk)

    # ---- types

    def xtype(self, t):
        """Translate a C++ type string. Returns (hc_type, todo_or_None)."""
        t = t.strip()
        todo = None
        t = re.sub(r"\b(const|constexpr|volatile|inline|mutable|explicit|"
                   r"register|extern|struct|class|enum)\b", "", t)
        t = re.sub(r"\s+", " ", t).strip()
        for pat, rep in self.rules:
            t = re.sub(pat, rep, t)
        for e in self.enum_types:
            t = re.sub(r"\b%s\b" % re.escape(e), "I64", t)
        t = re.sub(r"\s*\*", " *", t).replace("* *", "**").strip()
        if "&" in t:
            todo = "reference type"
        if "<" in t or "::" in t or re.search(r"\bauto\b", t):
            todo = "unknown type '%s'" % t
        return t, todo

    def is_class(self, name):
        return name in self.classes

    # ---- top level

    def feed(self, src, filename="<input>"):
        src = strip_comments(src)
        for m in re.finditer(r"PLUGINLIB_EXPORT_CLASS\s*\(\s*([\w:]+)\s*,\s*([\w:]+)\s*\)\s*;?", src):
            self.exports.append((m.group(1), m.group(2)))
        src = re.sub(r"PLUGINLIB_EXPORT_CLASS\s*\([^)]*\)\s*;?", "", src)
        self.walk(src, [])

    def walk(self, src, ns):
        guard = None
        for item in top_level_chunks(src):
            if item.startswith("#"):
                g = self.preproc(item, guard)
                if isinstance(g, tuple):
                    guard = g[1]
                    continue
                if g:
                    self.globals.append(g)
                continue
            m = re.match(r"(inline\s+)?namespace\s*([\w:]*)\s*\{", item)
            if m:
                body = item[m.end() - 1:]
                body = body[1:match_close(body, 0)]
                self.walk(body, ns + [x for x in m.group(2).split("::") if x])
                continue
            if re.match(r"using\s+namespace\b", item):
                continue
            if re.match(r"template\s*<", item):
                self.globals.append(TODO + "template skipped:\n/*\n%s\n*/" % item)
                continue
            m = re.match(r"(class|struct)\s+(\w+)\s*(final\s*)?(:\s*([^{]*))?\{", item)
            if m:
                self.parse_class(item, m, ns)
                continue
            m = re.match(r"(class|struct)\s+\w+\s*;$", item)
            if m:
                continue  # forward declaration
            m = re.match(r"enum\s+(class\s+|struct\s+)?(\w+)\s*(:\s*[\w\s]+)?\{", item)
            if m:
                self.enum_types.add(m.group(2))
                body = item[item.index("{"):]
                inner = body[1:match_close(body, 0)]
                self.globals.append("enum\n{\n  %s\n};" % ",\n  ".join(
                    p.strip() for p in split_top(inner) if p.strip()))
                continue
            if "{" in item and re.search(r"\)\s*(const\s*)?(noexcept\s*)?(override\s*)?"
                                         r"(final\s*)?(:[^{]*)?\{", item.split("{")[0] + "{"):
                self.parse_function(item, ns)
                continue
            self.globals.append(self.xglobal(item))

    def preproc(self, line, guard):
        line = line.strip()
        if re.match(r"#\s*(include|pragma)\b", line):
            return None
        m = re.match(r"#\s*ifndef\s+(\w+)$", line)
        if m and m.group(1).endswith(("_H", "_HPP", "_H_", "_HPP_")):
            return ("guard", m.group(1))
        m = re.match(r"#\s*define\s+(\w+)\s*$", line)
        if m and m.group(1) == guard:
            return None
        if re.match(r"#\s*endif", line) and guard:
            return None
        return line

    def xglobal(self, item):
        decl = item.rstrip(";").strip()
        if "(" in decl.split("=")[0]:
            return None or ""  # prototype; we emit our own
        m = re.match(r"(static\s+|const\s+|constexpr\s+)*([\w:<>\s\*&]+?)\s*(\w+)\s*(\[[^\]]*\])?\s*(=\s*(.*))?$",
                     decl, re.S)
        if not m:
            return TODO + "could not parse: " + decl
        t, todo = self.xtype(m.group(2))
        line = "%s %s%s" % (t, m.group(3), m.group(4) or "")
        if m.group(6):
            line += "=" + self.xexpr_simple(m.group(6))
        line += ";"
        return line + (" " + TODO + todo if todo else "")

    # ---- classes

    def parse_class(self, item, m, ns):
        name = m.group(2)
        bases = []
        if m.group(5):
            for b in split_top(m.group(5), angles=True):
                b = re.sub(r"\b(public|private|protected|virtual)\b", "", b).strip()
                bases.append(b.split("::")[-1])
        k = Klass(name, "::".join(ns + [name]), bases)
        k.full_bases = [re.sub(r"\b(public|private|protected|virtual)\b", "", b).strip()
                        for b in split_top(m.group(5) or "", angles=True) if b.strip()]
        body = item[item.index("{"):]
        body = body[1:match_close(body, 0)]
        body = re.sub(r"\b(public|private|protected)\s*:(?!:)", "", body)
        self.classes[name] = k
        self.order.append(name)
        for mem in top_level_chunks(body):
            self.parse_member(k, mem)

    def parse_member(self, k, mem):
        if re.match(r"(using|friend|typedef)\b", mem):
            k.notes.append(TODO + "skipped: " + " ".join(mem.split()))
            return
        if re.match(r"(class|struct|enum)\b", mem):
            k.notes.append(TODO + "nested type skipped: " + mem.split("{")[0].strip())
            return
        head = mem.split("{")[0]
        before_paren = head.split("(")[0]
        if "(" in head and "=" not in before_paren:
            self.parse_method_decl(k, mem)
            return
        static = bool(re.match(r"\s*(inline\s+)?static\b", mem))
        decl = re.sub(r"^\s*(inline\s+)?static\s+", "", mem.rstrip(";"))
        parts = split_top(decl, angles=True)
        m = re.match(r"\s*(.*?)([\w]+)\s*(\[[^\]]*\])?\s*(=\s*(.*)|\{(.*)\})?\s*$", parts[0], re.S)
        if not m:
            k.notes.append(TODO + "field not parsed: " + mem)
            return
        base_t = m.group(1)
        decls = [(m.group(2), m.group(3), m.group(5) or m.group(6))]
        for p in parts[1:]:
            pm = re.match(r"\s*(\**)\s*(\w+)\s*(\[[^\]]*\])?\s*(=\s*(.*))?$", p, re.S)
            if pm:
                decls.append((pm.group(2), pm.group(3), pm.group(5)))
        for nm, arr, init in decls:
            t, todo = self.xtype(base_t)
            f = Field(t + ((" " + arr) if arr else ""), nm, init.strip() if init else None, static)
            f.arr = arr
            f.todo = todo
            k.fields.append(f)

    def split_signature(self, head):
        """'virtual double area() const override' -> (prefix, name, params, suffix)."""
        i = head.index("(")
        j = match_close(head, i, "(", ")")
        prefix_name = head[:i].strip()
        params = head[i + 1:j]
        suffix = head[j + 1:].strip()
        m = re.match(r"(.*?)(~?\w+)$", prefix_name, re.S)
        return m.group(1).strip(), m.group(2), params, suffix

    def parse_method_decl(self, k, mem, body=None, init_list=""):
        if body is None and "{" in mem:
            bi = self.body_start(mem)
            head, body = mem[:bi], mem[bi:]
            body = body[1:match_close(body, 0)]
        else:
            head = mem.rstrip(";")
        init_list = ""
        prefix, name, params, suffix = self.split_signature(head)
        if ":" in suffix and "::" not in suffix.split(":")[0] + ":":
            suffix, init_list = suffix.split(":", 1)
        pure = bool(re.search(r"=\s*0\s*$", suffix))
        if re.search(r"=\s*(default|delete)\s*$", suffix):
            return
        flags = set(prefix.split())
        ret = re.sub(r"\b(virtual|static|inline|explicit|constexpr|friend)\b", "", prefix).strip()
        meth = Method(k, name, ret, self.parse_params(params), body,
                      ctor=(name == k.name), dtor=(name == "~" + k.name),
                      static="static" in flags, virtual="virtual" in flags,
                      pure=pure, init_list=init_list.strip())
        # an out-of-line definition replaces the in-class declaration
        for i, old in enumerate(k.methods):
            if (old.name == meth.name and old.body is None and body is not None
                    and len(old.params) == len(meth.params)):
                meth.static = meth.static or old.static
                meth.virtual = meth.virtual or old.virtual
                k.methods[i] = meth
                return
        k.methods.append(meth)

    def body_start(self, s):
        """Index of the '{' that opens a function body (skips init lists' {})."""
        i = s.index("(")
        i = match_close(s, i, "(", ")") + 1
        while i < len(s):
            c = s[i]
            if c == "(":
                i = match_close(s, i, "(", ")") + 1
                continue
            if c == "{":
                # init-list brace: "x_{1}" -> preceded by an identifier
                if re.search(r"\w\s*$", s[:i]) and re.search(r":\s*[^{]*$", s[:i]) \
                        and not re.search(r"\)\s*(const\s*)?(noexcept\s*)?(override\s*)?$", s[:i]):
                    i = match_close(s, i) + 1
                    continue
                return i
            i += 1
        raise ValueError("no body")

    def parse_params(self, params):
        out = []
        params = params.strip()
        if params in ("", "void"):
            return out
        for p in split_top(params, angles=True):
            p = p.strip()
            default = None
            if "=" in p:
                p, default = p.split("=", 1)
                p, default = p.strip(), default.strip()
            m = re.match(r"(.*?)(\w+)\s*(\[\s*\])?$", p, re.S)
            if not m or not m.group(1).strip():
                out.append(("I64", "arg%d" % len(out), default, "unnamed parameter '%s'" % p, False))
                continue
            raw = m.group(1).strip()
            const = bool(re.search(r"\bconst\b", raw))
            todo = None
            byref = False
            short = re.sub(r"\bconst\b|&", "", raw).strip().split("::")[-1]
            if "&" in raw and "&&" not in raw and short in self.classes:
                # class passed by reference -> pointer; call sites get &x
                raw = short + " *"
                byref = True
            elif "&&" in raw:
                raw = raw.replace("&&", "")
                todo = "rvalue reference"
            elif "&" in raw:
                if const:
                    raw = raw.replace("&", "")
                else:
                    raw = raw.replace("&", "*")
                    todo = "non-const reference became a pointer - callers must pass &x"
            t, t_todo = self.xtype(raw)
            if m.group(3):
                t += " *"
            if default is not None:
                default = self.xexpr_simple(default)
            out.append((t, m.group(2), default, todo or t_todo, byref))
        return out

    # ---- functions / out-of-line methods

    def parse_function(self, item, ns):
        bi = self.body_start(item)
        head = item[:bi]
        prefix, name, params, suffix = self.split_signature(head)
        m = re.match(r"(.*?)(\b\w+)::$", prefix + "::" if False else prefix)
        q = re.search(r"([\w]+)::(~?)$", head[:head.index("(")].strip()[:-len(name)])
        if q:
            cls = q.group(1)
            if cls in self.classes:
                k = self.classes[cls]
                ret = prefix[:prefix.rfind(cls + "::")].strip()
                body = item[bi:]
                body = body[1:match_close(body, 0)]
                init_list = ""
                if ":" in suffix:
                    suffix, init_list = suffix.split(":", 1)
                meth = Method(k, name, ret, self.parse_params(params), body,
                              ctor=(name == k.name), dtor=(name == "~" + k.name),
                              init_list=init_list.strip())
                for i, old in enumerate(k.methods):
                    if old.name == name and old.body is None and len(old.params) == len(meth.params):
                        meth.static, meth.virtual = old.static, old.virtual
                        k.methods[i] = meth
                        break
                else:
                    k.methods.append(meth)
                return
            self.globals.append(TODO + "method of unknown class '%s' - include its header too:\n/*\n%s\n*/"
                                % (cls, item))
            return
        ret = re.sub(r"\b(static|inline|constexpr)\b", "", prefix).strip()
        body = item[bi:]
        body = body[1:match_close(body, 0)]
        self.funcs.append(Method(None, name, ret, self.parse_params(params), body))

    # ---- naming

    def assign_names(self):
        for k in self.classes.values():
            seen = {}
            for m in k.methods:
                if m.ctor:
                    base = k.name + "_New"
                elif m.dtor:
                    base = k.name + "_Del"
                else:
                    base = "%s_%s" % (k.name, m.name)
                n = seen.get(base, 0) + 1
                seen[base] = n
                m.hc_name = base if n == 1 else "%s%d" % (base, n)
                m.overload = n > 1
        for f in self.funcs:
            f.hc_name = f.name

    # ---- expressions and statements

    def xexpr_simple(self, e):
        """Expression rewrite without class context (globals, defaults)."""
        return self.xbody(e, None, {}, expr=True).strip()

    def xbody(self, body, k, var_types, expr=False, self_name="self"):
        b = body
        hoisted = []

        # for (int i = 0; ...) -> hoist declaration
        def hoist(mt):
            t, todo = self.xtype(mt.group(1))
            hoisted.append((t, mt.group(2)))
            var_types[mt.group(2)] = t
            return "for (%s =" % mt.group(2)
        b = sub_code(r"\bfor\s*\(\s*((?:const\s+)?[\w:]+(?:\s*\*+)?)\s+(\w+)\s*=", hoist, b)
        b = sub_code(r"\bfor\s*\(\s*([^;()]*?)\s*:\s*([^;()]*)\)",
                     lambda mt: "for (%s : %s) " % (mt.group(1), mt.group(2)) +
                     "/*range-for*/", b)

        # casts: static_cast<T>(e) -> (e)(T)
        def fix_casts(s):
            while True:
                mt = re.search(r"\b(static|reinterpret|const)_cast\s*<([^<>]+)>\s*\(", s)
                if not mt:
                    return s
                j = match_close(s, mt.end() - 1, "(", ")")
                inner = s[mt.end():j]
                t, _ = self.xtype(mt.group(2))
                s = s[:mt.start()] + hc_cast(inner, t) + s[j + 1:]
        b = fix_casts(b)
        # C-style casts of primitive types: (double)x -> (x)(F64)
        b = sub_code(r"\(\s*(%s)\s*\)\s*(\w+(?:\.\w+)?|\([^()]*\))" % PRIM_CAST,
                     lambda mt: hc_cast(mt.group(2)[1:-1] if mt.group(2).startswith("(")
                                        else mt.group(2), self.xtype(mt.group(1))[0]), b)
        # local declarations: collect names/types and translate types
        b = self.xdecls(b, var_types)
        # std::cout / std::cerr chains -> Print
        b = self.xcout(b, k, var_types)
        # RCLCPP logging
        b = sub_code(r"\bRCLCPP_(DEBUG|INFO|WARN|ERROR|FATAL)\s*\(\s*[^,]+,\s*",
                     lambda mt: "RosLog%s(" % mt.group(1).capitalize(), b)
        b = sub_code(r"\b(std::)?printf\s*\(", "Print(", b)
        b = sub_code(r"\b(std::)?(puts)\s*\(", "PutS(", b)
        # format string fixups (only literals that look like formats)
        b = "".join(t if code else self.fix_fmt(t) for code, t in code_segments(b))
        # literals
        b = sub_code(r"\bnullptr\b", "NULL", b)
        b = sub_code(r"\btrue\b", "TRUE", b)
        b = sub_code(r"\bfalse\b", "FALSE", b)
        b = sub_code(r"(\b\d+\.\d*(?:[eE][+-]?\d+)?|\.\d+(?:[eE][+-]?\d+)?|\b\d+[eE][+-]?\d+)[fFlL]\b", r"\1", b)
        b = sub_code(r"\b(\d+)(?:[uU][lL]{0,2}|[lL]{1,2}[uU]?)\b", r"\1", b)
        b = sub_code(r"\bM_PI\b", "pi", b)
        b = sub_code(r"\bstd::move\s*\(([^()]*)\)", r"(\1)", b)
        b = sub_code(r"\.c_str\s*\(\s*\)", "", b)
        b = self.xmath(b)
        # this->
        b = sub_code(r"\bthis\s*->\s*", self_name + "->", b)
        b = sub_code(r"\(\s*\*\s*this\s*\)\s*\.", self_name + "->", b)
        b = sub_code(r"\*\s*this\b", "*" + self_name, b)
        b = sub_code(r"\bthis\b", self_name, b)
        if k is not None:
            b = self.xmembers(b, k, var_types, self_name)
        types = {}
        if k is not None:
            types = {f.name: f.ctype for f in k.all_fields(self.classes)}
        types.update(var_types)
        b = sub_code(r"([\w>\-]*?)(\w+)\s*\.\s*(?:size|length)\s*\(\s*\)",
                     lambda mt: "StrLen(%s%s)" % (mt.group(1), mt.group(2))
                     if (types.get(mt.group(2)) or "").replace(" ", "") == "U8*" else mt.group(0), b)
        b = self.xcalls(b, k, types, self_name)
        b = self.xnewdelete(b, types)
        # Class::static_member -> Class_static_member
        for cname in self.classes:
            b = sub_code(r"\b%s::(\w+)" % cname, r"%s_\1" % cname, b)
        if not expr and hoisted:
            seen, decl = set(), []
            for t, n in hoisted:
                if n not in seen:
                    seen.add(n)
                    decl.append("%s %s;" % (t, n))
            ind = re.match(r"\n*([ \t]*)", b).group(1)
            b = "\n" + ind + " ".join(decl) + "\n" + b.lstrip("\n")
        b = self.mark_todos(b)
        return b

    def fix_fmt(self, lit):
        if "%" not in lit:
            return lit
        lit = re.sub(r"%(-?\d*)(?:\.(\d+))?(?:l{1,2}|z|h{1,2}|j|t)?([diu])",
                     lambda m: "%" + m.group(1) + "d", lit)
        lit = re.sub(r"%(-?\d*(?:\.\d+)?)l?f", r"%\1f", lit)
        lit = re.sub(r"%(?:l{1,2}|z)?x", "%X", lit)
        return lit

    def xmath(self, b):
        def one(mt):
            name = mt.group(2)
            rep = MATH_MAP.get(name)
            if name == "atan2":
                return "__ATAN2__("
            if name == "hypot":
                return "__HYPOT__("
            return (rep or name) + "("
        b = sub_code(r"\b(std::)?(sqrt|pow|sin|cos|tan|atan2|atan|exp|log10|log|floor|ceil|"
                     r"round|fabs|abs|fmin|fmax|hypot)\s*\(", one, b)
        b = sub_code(r"\bstd::(min|max)\s*\(", lambda mt: MATH_MAP[mt.group(1)] + "(", b)
        # atan2(y,x) -> Arg(x,y); hypot(a,b) -> Sqrt(a*a+b*b)
        for tag in ("__ATAN2__(", "__HYPOT__("):
            while tag in b:
                i = b.index(tag) + len(tag) - 1
                j = match_close(b, i, "(", ")")
                args = split_top(b[i + 1:j])
                if len(args) == 2:
                    a0, a1 = args[0].strip(), args[1].strip()
                    if tag.startswith("__ATAN2"):
                        rep = "Arg(%s,%s)" % (a1, a0)
                    else:
                        rep = "Sqrt((%s)*(%s)+(%s)*(%s))" % (a0, a0, a1, a1)
                else:
                    rep = "BAD_ARGS(%s)" % b[i + 1:j]
                b = b[:b.index(tag)] + rep + b[j + 1:]
        return b

    def xcout(self, b, k, var_types):
        def conv(mt):
            parts = [p.strip() for p in split_top(mt.group(1), "<<")][1:]
            fmt, args, todo = [], [], None
            for p in parts:
                if p in ("std::endl", "endl", "'\\n'", "std::flush"):
                    if p != "std::flush":
                        fmt.append("\\n")
                elif p.startswith('"') and p.endswith('"'):
                    fmt.append(p[1:-1].replace("%", "%%"))
                else:
                    spec = self.guess_fmt(p, k, var_types)
                    if spec is None:
                        spec, todo = "%d", "check Print format for '%s'" % p
                    fmt.append(spec)
                    args.append(p)
            call = 'Print("%s"%s);' % ("".join(fmt), "".join("," + a for a in args))
            return call + (" " + TODO + todo if todo else "")
        return sub_cout(conv, b)

    def guess_fmt(self, expr, k, var_types):
        e = expr.strip()
        if re.fullmatch(r"-?\d+", e):
            return "%d"
        if re.fullmatch(r"-?(\d+\.\d*|\.\d+)([eE][+-]?\d+)?[fF]?", e):
            return "%f"
        if re.fullmatch(r"'.'", e):
            return "%c"
        name = re.sub(r"^(this->|self->)", "", e)
        mc = re.fullmatch(r"(\w+)\s*\(.*\)", name, re.S)
        if mc and k is not None:
            m = k.find_method(mc.group(1), self.classes)
            if m and not m.ctor:
                name, var_types = "__ret__", dict(var_types, __ret__=self.xtype(m.ret or "void")[0])
        t = var_types.get(name)
        if t is None and k is not None:
            for f in k.all_fields(self.classes):
                if f.name == name:
                    t = f.ctype
        if t is None:
            return None
        t = t.strip()
        if t == "F64":
            return "%f"
        if t == "U8 *":
            return "%s"
        if t in HC_PRIMS:
            return "%d"
        return None

    DECL_RE = re.compile(
        r"(^|[;{}]\s*|\n\s*)((?:const\s+|static\s+|constexpr\s+)*"
        r"(?:unsigned\s+long\s+long|unsigned\s+long|unsigned\s+int|unsigned\s+short|unsigned\s+char|"
        r"long\s+long|long\s+double|unsigned|std::string|std::size_t|size_t|u?int(?:8|16|32|64)_t|"
        r"std::u?int(?:8|16|32|64)_t|double|float|bool|char|short|long|int|auto|[A-Z]\w*)"
        r"(?:\s*\*+\s*|\s*&\s*|\s+))(\w+)(\s*(?:=|;|\[|,|\())")

    def xdecls(self, b, var_types):
        known = set(self.classes) | self.enum_types

        def one(mt):
            raw = mt.group(2)
            word = re.sub(r"\b(const|static|constexpr)\b", "", raw).replace("*", "").replace("&", "").strip()
            name = mt.group(3)
            tail = mt.group(4)
            if word in ("return", "delete", "new", "else", "case", "goto", "throw"):
                return mt.group(0)
            if word[:1].isupper() and word not in known:
                return mt.group(0)  # some other capitalized word, not a type we know
            if tail.strip() == "(" and word not in known:
                return mt.group(0)  # looks like a call, e.g. "int(x)"
            t, todo = self.xtype(raw.replace("&", ""))
            if word in known and "*" not in raw:
                todo = todo or None
            var_types[name] = t
            note = ""
            if "&" in raw:
                note = " /*was a reference*/"
            sep = "" if t.endswith("*") else " "
            return "%s%s%s%s%s%s" % (mt.group(1), t, sep, name, note, tail)
        return "".join(self.DECL_RE.sub(one, t) if code else t for code, t in code_segments(b))

    def xmembers(self, b, k, var_types, self_name):
        fields = k.all_fields(self.classes)
        names = {f.name for f in fields} - set(var_types)
        statics = {f.name for f in k.fields if f.static}
        for n in sorted(names, key=len, reverse=True):
            b = sub_code(r"(?<![\w.>:])(?<!->)\b%s\b(?!\s*::)" % re.escape(n), "%s->%s" % (self_name, n), b)
        for n in statics:
            b = sub_code(r"(?<![\w.>:])\b%s\b" % re.escape(n), "%s_%s" % (k.name, n), b)
        return b

    def call_args(self, b, start):
        """b[start] == '('; return (args_text, end_index)."""
        j = match_close(b, start, "(", ")")
        return b[start + 1:j], j

    def xcalls(self, b, k, var_types, self_name):
        out_parts = []
        for code, t in code_segments(b):
            out_parts.append(self._xcalls_code(t, k, var_types, self_name) if code else t)
        return "".join(out_parts)

    def _xcalls_code(self, b, k, var_types, self_name):
        # obj->m(  obj.m(  self->m(  bare m(  -- for converted classes only
        pat = re.compile(r"(?:(\b[\w]+)\s*(->|\.)\s*)?\b(\w+)\s*\(")
        i, out = 0, []
        while True:
            mt = pat.search(b, i)
            if not mt:
                out.append(b[i:])
                return "".join(out)
            obj, op, meth = mt.group(1), mt.group(2), mt.group(3)
            target_cls = None
            obj_expr = None
            if obj is not None:
                if obj == self_name and k is not None:
                    target_cls, obj_expr = k, self_name
                else:
                    t = (var_types.get(obj) or "").replace("*", "").strip()
                    if t in self.classes:
                        target_cls = self.classes[t]
                        obj_expr = obj if op == "->" else "&" + obj
            elif k is not None and mt.start() > 0 and b[mt.start() - 1] in ".>":
                target_cls = None
            elif k is not None:
                target_cls, obj_expr = k, self_name
            m = target_cls.find_method(meth, self.classes) if target_cls else None
            if not m:
                out.append(b[i:mt.end()])
                i = mt.end()
                continue
            args, j = self.call_args(b, mt.end() - 1)
            args = self._xcalls_code(args, k, var_types, self_name)
            if any(p[4] for p in m.params):
                parts = split_top(args) if args.strip() else []
                for pi, prm in enumerate(m.params):
                    if prm[4] and pi < len(parts):
                        a = parts[pi].strip()
                        parts[pi] = a[1:].strip() if a.startswith("*") else "&" + a
                args = ",".join(parts)
            if m.static:
                call = "%s(%s)" % (m.hc_name, args)
            else:
                call = "%s(%s%s)" % (m.hc_name, obj_expr, ("," + args) if args.strip() else "")
            out.append(b[i:mt.start()] + call)
            i = j + 1

    def xnewdelete(self, b, var_types):
        def new(mt):
            t = mt.group(1)
            if mt.group(2) == "[":
                tt, _ = self.xtype(t)
                return "CAlloc((%s)*sizeof(%s))" % (mt.group(3), tt)
            if t in self.classes:
                return "%s_New(" % t
            tt, _ = self.xtype(t)
            return "CAlloc(sizeof(%s)); %s new %s(" % (tt, TODO, t)
        b = sub_code(r"\bnew\s+([\w:]+)\s*(\[)\s*([^\]]+)\]", new, b)
        b = sub_code(r"\bnew\s+([\w:]+)\s*(\()", new, b)
        b = sub_code(r"\bnew\s+([\w:]+)\b(?!\s*[(\[])", lambda mt: "%s_New()" % mt.group(1)
                     if mt.group(1) in self.classes else
                     "CAlloc(sizeof(%s))" % self.xtype(mt.group(1))[0], b)

        def delete(mt):
            arr, v = mt.group(1), mt.group(2)
            t = (var_types.get(v.split("->")[-1]) or "").replace("*", "").strip()
            if not arr and t in self.classes:
                return "%s_Del(%s);" % (t, v)
            return "Free(%s);" % v
        b = sub_code(r"\bdelete\s*(\[\s*\])?\s*([\w>\-]+)\s*;", delete, b)
        return b

    def mark_todos(self, b):
        lines = b.split("\n")
        out = []
        for ln in lines:
            if TODO in ln:
                out.append(ln)
                continue
            code_only = "".join(t if c else '""' for c, t in code_segments(ln))
            reasons = []
            for pat, why in LEFTOVER_CHECKS:
                if why and re.search(pat, code_only):
                    reasons.append(why)
                    break
            if "/*range-for*/" in ln:
                reasons.append("range-for - rewrite as an index loop")
            if re.search(r"(?<![\w)\]])\s*&\s*\w+\s*=", code_only) and "&&" not in code_only:
                pass
            if re.search(r"[\w\]\)]\s*(->|\.)\s*\w+\s*\(", code_only) and "Print" not in code_only:
                reasons.append("method call on an object of unknown type")
            if re.search(r"\w\s*\+\s*\"|\"\s*\+\s*\w", code_only):
                reasons.append("string '+' - use MStrPrint/CatPrint")
            if reasons:
                ln = ln + " " + TODO + "; ".join(dict.fromkeys(reasons))
            out.append(ln)
        return "\n".join(out)

    # ---- emit

    def emit(self, sources):
        self.assign_names()
        L = []
        L.append("//Converted by cpp2hc from: %s" % ", ".join(os.path.basename(s) for s in sources))
        L.append("//Needs ROS2-Holy-C loaded first (src/Load.HC).")
        L.append("//Search for TODO(cpp2hc) and fix every one before trusting this file.")
        L.append("")
        for g in self.globals:
            if g:
                L.append(g)
        if self.globals:
            L.append("")

        for cname in self.order:
            k = self.classes[cname]
            conv_bases = [b for b in k.bases if b in self.classes]
            head = "class %s" % k.name
            if conv_bases:
                head += ":%s" % conv_bases[0]
                if len(conv_bases) > 1:
                    head += " " + TODO + "multiple inheritance, only first base kept"
            L.append(head)
            L.append("{")
            own = [f for f in k.fields if not f.static]
            for f in own:
                ct = f.ctype.replace(" " + (f.arr or ""), "") if f.arr else f.ctype
                ln = "  %s%s%s%s;" % (ct, "" if ct.endswith("*") else " ", f.name, f.arr or "")
                if f.todo:
                    ln += " " + TODO + f.todo
                L.append(ln)
            if not own and not conv_bases:
                L.append("  I64 cpp2hc_empty;")
            L.append("};")
            for n in k.notes:
                L.append(n)
            for f in k.fields:
                if f.static:
                    init = (" = " + self.xexpr_simple(f.init)) if f.init else ""
                    L.append("%s %s_%s%s;" % (f.ctype, k.name, f.name, init))
            L.append("")

        # prototypes so methods can call each other in any order
        protos = []
        for cname in self.order:
            for m in self.classes[cname].methods:
                if m.body is not None or m.ctor or m.dtor:
                    protos.append(self.signature(m) + ";")
        for f in self.funcs:
            if f.name != "main":
                protos.append(self.signature(f) + ";")
        need_new = [c for c in self.order if not any(m.ctor for m in self.classes[c].methods)]
        need_del = [c for c in self.order if not any(m.dtor for m in self.classes[c].methods)]
        for c in need_new:
            protos.append("%s *%s_New();" % (c, c))
        for c in need_del:
            protos.append("U0 %s_Del(%s *self);" % (c, c))
        if protos:
            L += ["extern " + p for p in protos]
            L.append("")

        for cname in self.order:
            k = self.classes[cname]
            for m in k.methods:
                if m.pure:
                    L.append("//%s::%s is pure virtual: implement it in a subclass." % (k.name, m.name))
                    continue
                if m.body is None and not m.ctor and not m.dtor:
                    L.append("%sdeclared but never defined: %s::%s" % (TODO, k.name, m.name))
                    continue
                L += self.emit_method(k, m)
                L.append("")
            if cname in need_new:
                L += self.emit_method(k, Method(k, k.name, "", [], "", ctor=True), name=k.name + "_New")
                L.append("")
            if cname in need_del:
                L += self.emit_method(k, Method(k, "~" + k.name, "", [], "", dtor=True), name=k.name + "_Del")
                L.append("")

        for f in self.funcs:
            L += self.emit_method(None, f)
            L.append("")

        if self.exports:
            L.append("//pluginlib registration (was PLUGINLIB_EXPORT_CLASS)")
            for derived, base in self.exports:
                short = derived.split("::")[-1]
                if short not in self.classes:
                    L.append("%sexported class %s was not found in the input" % (TODO, derived))
                    continue
                k = self.classes[short]
                L.append('PluginClassAdd("%s","%s",&%s_New,&%s_Del);' % (derived, base, short, short))
                done = set()
                kk = k
                chain = [k]
                while kk.bases and kk.bases[0] in self.classes:
                    kk = self.classes[kk.bases[0]]
                    chain.append(kk)
                for c in chain:
                    for m in c.methods:
                        if m.ctor or m.dtor or m.static or m.pure or m.body is None or m.name in done:
                            continue
                        if getattr(m, "overload", False):
                            continue
                        done.add(m.name)
                        L.append('PluginMethodAdd("%s","%s",&%s);' % (derived, m.name, m.hc_name))
            L.append("")
        return "\n".join(L).rstrip() + "\n"

    def signature(self, m, name=None):
        name = name or m.hc_name
        params = []
        if m.cls is not None and not m.static and not m.ctor:
            params.append("%s *self" % m.cls.name)
        for t, n, d, _todo, _ref in m.params:
            p = "%s %s" % (t, n)
            if d is not None:
                p += "=" + d
            params.append(p.replace("* ", "*") if t.endswith("*") else p)
        if m.ctor:
            ret = "%s *" % m.cls.name
        elif m.dtor:
            ret = "U0 "
        else:
            r, _ = self.xtype(m.ret or "void")
            ret = r if r.endswith("*") else r + " "
        return "%s%s(%s)" % (ret, name, ",".join(params))

    def emit_method(self, k, m, name=None):
        var_types = {}
        todos = []
        body = m.body or ""
        for t, n, _d, todo, byref in m.params:
            var_types[n] = t
            if todo:
                todos.append("param %s: %s" % (n, todo))
            if byref:
                body = sub_code(r"(?<![\w.>])%s\s*\.(?!\d)" % re.escape(n), n + "->", body)
        if getattr(m, "overload", False):
            todos.append("overloaded in C++, renamed to %s - fix callers" % m.hc_name)
        if m.ret and self.xtype(m.ret)[1]:
            todos.append("return type: " + self.xtype(m.ret)[1])
        L = [self.signature(m, name) + ((" " + TODO + "; ".join(todos)) if todos else "")]
        L.append("{")
        if m.ctor:
            L.append("  %s *self=CAlloc(sizeof(%s));" % (k.name, k.name))
            inits = {}
            for f in k.all_fields(self.classes):
                if f.init is not None:
                    inits[f.name] = f.init
            base_init = []
            for it in split_top(m.init_list):
                it = it.strip()
                mt = re.match(r"([\w:]+)\s*[({](.*)[)}]$", it, re.S)
                if not mt:
                    continue
                nm = mt.group(1)
                if nm.split("::")[-1] in k.bases or "::" in nm:
                    base_init.append(it)
                else:
                    inits[nm] = mt.group(2)
            for it in base_init:
                L.append("  %sbase class init '%s' not converted" % (TODO, it))
            for nm, init in inits.items():
                L.append("  self->%s=%s;" % (nm, self.xbody(init, k, dict(var_types), expr=True).strip()))
            if "return" in body:
                todos.append("constructor has 'return' - must return self")
        x = textwrap.dedent(self.xbody(body, k, var_types).strip("\n").expandtabs(2))
        for ln in x.split("\n"):
            if ln.strip() == "" and (not L or L[-1].strip() == ""):
                continue
            L.append(("  " + ln.rstrip()) if ln.strip() else "")
        while L and L[-1].strip() == "":
            L.pop()
        if m.ctor:
            L.append("  return self;")
        if m.dtor:
            L.append("  Free(self);")
        L.append("}")
        return L


# ---------------------------------------------------------------- report

def report(text):
    lines = text.split("\n")
    code = [ln for ln in lines if ln.strip() and not ln.strip().startswith("//")]
    bad = [(i + 1, ln) for i, ln in enumerate(lines) if "TODO(cpp2hc)" in ln and not ln.startswith("//Search")]
    bad_code = [ln for ln in code if "TODO(cpp2hc)" in ln]
    score = 100.0 if not code else 100.0 * (len(code) - len(bad_code)) / len(code)
    return score, bad


def convert_files(paths, exact_ints=False):
    cv = Converter(exact_ints=exact_ints)
    # headers first so classes exist before out-of-line definitions
    paths = sorted(paths, key=lambda p: 0 if p.endswith((".h", ".hpp", ".hh", ".hxx")) else 1)
    for p in paths:
        with open(p, encoding="utf-8", errors="replace") as fh:
            cv.feed(fh.read(), p)
    return cv.emit(paths)


def convert_text(text, exact_ints=False, name="<input>"):
    cv = Converter(exact_ints=exact_ints)
    cv.feed(text, name)
    return cv.emit([name])


def manifest(xml_text, source_name="plugins.xml"):
    root = ET.fromstring(xml_text)
    libs = [root] if root.tag == "library" else root.findall("library")
    L = ["//Converted by cpp2hc from %s" % source_name,
         "//Declares the plugin classes so PluginList shows them before they are loaded.", ""]
    for lib in libs:
        L.append("//library: %s" % lib.get("path", "?"))
        for cls in lib.findall("class"):
            name = cls.get("type") or cls.get("name")
            base = cls.get("base_class_type", "")
            desc_el = cls.find("description")
            desc = " ".join((desc_el.text or "").split()) if desc_el is not None else ""
            desc = desc.replace("\\", "\\\\").replace('"', '\\"')
            L.append('PluginDescribe("%s","%s","%s");' % (name, base, desc))
    return "\n".join(L) + "\n"


def main(argv=None):
    ap = argparse.ArgumentParser(description="Convert C++ ROS 2 plugins to HolyC (ROS2-Holy-C).")
    sub = ap.add_subparsers(dest="cmd", required=True)
    c = sub.add_parser("convert", help="convert .hpp/.cpp files into one .HC file")
    c.add_argument("inputs", nargs="+")
    c.add_argument("-o", "--output", help="output .HC file (default: stdout)")
    c.add_argument("--strict", action="store_true", help="exit 2 if any TODO(cpp2hc) remains")
    c.add_argument("--exact-ints", action="store_true",
                   help="map int->I32, short->I16 (default maps them to I64, the HolyC norm)")
    c.add_argument("-q", "--quiet", action="store_true", help="no report on stderr")
    mf = sub.add_parser("manifest", help="convert a pluginlib plugins.xml into PluginDescribe calls")
    mf.add_argument("xml")
    mf.add_argument("-o", "--output")
    a = ap.parse_args(argv)

    if a.cmd == "manifest":
        with open(a.xml, encoding="utf-8") as fh:
            out = manifest(fh.read(), os.path.basename(a.xml))
        write_out(out, a.output)
        return 0

    out = convert_files(a.inputs, exact_ints=a.exact_ints)
    write_out(out, a.output)
    score, bad = report(out)
    if not a.quiet:
        sys.stderr.write("cpp2hc: portability score %.0f%% (%d TODO)\n" % (score, len(bad)))
        for ln_no, ln in bad:
            why = ln.split("TODO(cpp2hc):", 1)[1].strip().rstrip("*/").strip()
            sys.stderr.write("  %s:%d: %s\n" % (a.output or "<stdout>", ln_no, why))
    if a.strict and bad:
        return 2
    return 0


def write_out(text, path):
    if path:
        with open(path, "w", encoding="utf-8") as fh:
            fh.write(text)
    else:
        sys.stdout.write(text)


if __name__ == "__main__":
    sys.exit(main())
