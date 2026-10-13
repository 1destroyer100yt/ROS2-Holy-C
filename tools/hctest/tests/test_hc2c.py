import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import hc2c  # noqa: E402


def lint_src(src):
    with tempfile.NamedTemporaryFile("w", suffix=".HC", delete=False) as fh:
        fh.write(src)
    try:
        return hc2c.lint(hc2c.gather(fh.name))
    finally:
        os.unlink(fh.name)


class TestLint(unittest.TestCase):
    def bad(self, line, why):
        errs = lint_src("U0 F()\n{\n  %s\n}\n" % line)
        self.assertTrue(any(why in e for e in errs), "%r not flagged (%s): %s" % (line, why, errs))

    def good(self, line):
        errs = lint_src("U0 F()\n{\n  %s\n}\n" % line)
        self.assertEqual(errs, [], line)

    def test_precedence(self):
        self.bad("x=a&0xFF+1;", "parentheses")
        self.bad("if (a&1==0) x=1;", "parentheses")
        self.bad("x=a<<2+1;", "parentheses")
        self.bad("x=a|b*c;", "parentheses")
        self.good("x=(a&0xFF)+1;")
        self.good("if ((a&1)==0) x=1;")
        self.good("x=-a&b;")
        self.good("x=*p&mask;")
        self.good("if (a && b>3) x=1;")
        self.good("Foo(a&b,c+1);")

    def test_other_rules(self):
        self.bad("x=a?1:2;", "?:")
        self.bad("int y;", "C keyword")
        self.bad("for (I64 i=0;i<3;i++) {}", "for()")
        self.bad('Print("$5");', "lone '$'")
        self.good('Print("$$FG,RED$$hi$$FG$$");')
        self.bad("x=*p(I64 *);", "postfix cast")
        self.good("x=sizeof(I64);")
        self.good('Print("what? int? $$");')
        self.bad('x="a" "b";', "adjacent")
        self.good('Print("a %s b","c d");')

    def test_default_args(self):
        errs = lint_src("I64 F(I64 a=1)\n{\n  return a;\n}\n")
        self.assertTrue(any("default args" in e for e in errs))

    def test_markers_and_dollars(self):
        src = "//@host-skip-begin\nZZZ junk ? int\n//@host-skip-end\n//@host: int host_only=1;\n" \
              'U0 F()\n{\n  Print("$$FG$$");\n}\n'
        with tempfile.NamedTemporaryFile("w", suffix=".HC", delete=False) as fh:
            fh.write(src)
        lines = hc2c.gather(fh.name)
        os.unlink(fh.name)
        self.assertEqual(hc2c.lint(lines), [])
        c = hc2c.to_c(lines)
        self.assertNotIn("ZZZ", c)
        self.assertIn("int host_only=1;", c)
        self.assertIn('Print("$FG$");', c)

    def test_class_to_struct(self):
        c = hc2c.to_c([("x.HC", 1, "class CFoo", False), ("x.HC", 2, "{", False),
                       ("x.HC", 3, "  I64 a;", False), ("x.HC", 4, "};", False)])
        self.assertIn("typedef struct CFoo CFoo; struct CFoo", c)


if __name__ == "__main__":
    unittest.main()
