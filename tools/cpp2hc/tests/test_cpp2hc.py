import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import cpp2hc  # noqa: E402

EX = os.path.join(os.path.dirname(HERE), "examples")


def conv(src, **kw):
    return cpp2hc.convert_text(src, **kw)


def balanced(text):
    depth = 0
    for code, t in cpp2hc.code_segments(text):
        if code:
            depth += t.count("{") - t.count("}")
    return depth == 0


class TestBasics(unittest.TestCase):
    def test_types(self):
        out = conv("class A { int a; double b; float c; bool d; std::string e; uint8_t f; };")
        for line in ("I64 a;", "F64 b;", "F64 c;", "Bool d;", "U8 *e;", "U8 f;"):
            self.assertIn(line, out)

    def test_exact_ints(self):
        out = conv("class A { int a; short b; unsigned int c; };", exact_ints=True)
        self.assertIn("I32 a;", out)
        self.assertIn("I16 b;", out)
        self.assertIn("U32 c;", out)

    def test_methods_get_self(self):
        out = conv("class A { public: int get() { return x_; } int x_; };")
        self.assertIn("I64 A_get(A *self)", out)
        self.assertIn("return self->x_;", out)

    def test_ctor_dtor_and_defaults(self):
        out = conv("class A { public: A() : x_(3) { y_ = 2; } ~A() {} int x_; int y_ = 7; };")
        self.assertIn("A *A_New()", out)
        self.assertIn("A *self=CAlloc(sizeof(A));", out)
        self.assertIn("self->x_=3;", out)
        self.assertIn("self->y_=7;", out)
        self.assertIn("return self;", out)
        self.assertIn("U0 A_Del(A *self)", out)
        self.assertIn("Free(self);", out)

    def test_out_of_line_definition(self):
        out = conv("class A { public: void f(int v); int x_; };\nvoid A::f(int v) { x_ = v; }")
        self.assertIn("U0 A_f(A *self,I64 v)", out)
        self.assertIn("self->x_ = v;", out)
        self.assertNotIn("declared but never defined", out)

    def test_method_call_rewrite(self):
        out = conv("class A { public: int g() { return 1; } int f() { return g() + this->g(); } };")
        self.assertIn("return A_g(self) + A_g(self);", out)

    def test_for_hoist(self):
        out = conv("void f() { for (int i = 0; i < 3; i++) {} }")
        self.assertIn("I64 i;", out)
        self.assertIn("for (i = 0; i < 3; i++)", out)

    def test_casts_convert_numbers(self):
        out = conv("void f(double d) { int a = static_cast<int>(d); double b = (double)a; }")
        self.assertIn("ToI64(d)", out)
        self.assertIn("ToF64(a)", out)

    def test_literals(self):
        out = conv("void f() { bool a = true; int *p = nullptr; float x = 1.5f; double y = M_PI; }")
        self.assertIn("TRUE", out)
        self.assertIn("NULL", out)
        self.assertIn("1.5;", out)
        self.assertIn("pi;", out)

    def test_cout(self):
        out = conv('class A { public: void f() { std::cout << "v=" << v_ << " 50%" << std::endl; } double v_; };')
        self.assertIn('Print("v=%f 50%%\\n",self->v_);', out)

    def test_printf_format_fix(self):
        out = conv('void f(long a, double b) { printf("%ld %lf %zu\\n", a, b, a); }')
        self.assertIn('Print("%d %f %d\\n", a, b, a);', out)

    def test_rclcpp_log(self):
        out = conv('void f() { RCLCPP_WARN(get_logger(), "x=%d", 3); }')
        self.assertIn('RosLogWarn("x=%d", 3);', out)

    def test_math(self):
        out = conv("double f(double y, double x) { return std::atan2(y, x) + std::sqrt(x) + std::hypot(x, y); }")
        self.assertIn("Arg(x,y)", out)
        self.assertIn("Sqrt(x)", out)
        self.assertIn("Sqrt((x)*(x)+(y)*(y))", out)

    def test_new_delete(self):
        out = conv("class P { public: P(int a) {} };\nclass A { public: void f() { p_ = new P(1); delete p_; int *q = new int[4]; delete[] q; } P *p_; };")
        self.assertIn("self->p_ = P_New(1);", out)
        self.assertIn("P_Del(self->p_);", out)
        self.assertIn("CAlloc((4)*sizeof(I64))", out)
        self.assertIn("Free(q);", out)

    def test_class_ref_param_becomes_pointer(self):
        out = conv("class P { public: double d(const P & o) const { return o.x; } double x; };\n"
                   "class A { public: double f(P *a, P *b) { return a->d(*b); } };")
        self.assertIn("F64 P_d(P *self,P *o)", out)
        self.assertIn("return o->x;", out)
        self.assertIn("return P_d(a,b);", out)

    def test_inheritance_when_base_known(self):
        out = conv("class B { public: int x; };\nclass D : public B { public: int y; };")
        self.assertIn("class D:B", out)

    def test_strings_untouched(self):
        out = conv('class A { public: void f() { printf("true this x_ nullptr\\n"); } int x_; };')
        self.assertIn('"true this x_ nullptr\\n"', out)

    def test_namespace_unwrapped_and_export(self):
        out = conv("namespace a { namespace b { class C { public: int f() { return 1; } }; } }\n"
                   "PLUGINLIB_EXPORT_CLASS(a::b::C, base::Thing)")
        self.assertIn('PluginClassAdd("a::b::C","base::Thing",&C_New,&C_Del);', out)
        self.assertIn('PluginMethodAdd("a::b::C","f",&C_f);', out)
        self.assertNotIn("namespace", out)


class TestTodoMarkers(unittest.TestCase):
    """The tool must flag what it can't do instead of producing silent garbage."""

    def flagged(self, src, why):
        out = conv(src)
        hits = [ln for ln in out.split("\n") if "TODO(cpp2hc)" in ln and why in ln]
        self.assertTrue(hits, "expected a TODO mentioning %r in:\n%s" % (why, out))

    def test_vector(self):
        self.flagged("void f() { std::vector<int> v; }", "STL")

    def test_auto(self):
        self.flagged("void f() { auto x = 3; }", "auto")

    def test_lambda(self):
        self.flagged("void f() { g([&](int a) { return a; }); }", "lambda")

    def test_smart_ptr(self):
        self.flagged("void f() { auto p = std::make_shared<int>(3); }", "auto")

    def test_template(self):
        self.flagged("template <typename T> T add(T a, T b) { return a + b; }", "template")

    def test_overload(self):
        self.flagged("class A { public: int f(int a) { return a; } int f(double a) { return 1; } };", "overloaded")

    def test_nonconst_ref(self):
        self.flagged("void f(int & x) { x = 1; }", "reference")

    def test_unknown_method_call(self):
        self.flagged("void f(Thing *t) { t->go(); }", "unknown type")

    def test_throw(self):
        self.flagged("void f() { throw 3; }", "throw")

    def test_ternary(self):
        self.flagged("int f(int a) { return a > 0 ? 1 : 2; }", "?:")

    def test_string_concat(self):
        self.flagged('void f(std::string a) { std::string b = a + "x"; }', "string '+'")


class TestExamples(unittest.TestCase):
    def test_polygon_converts_clean(self):
        d = os.path.join(EX, "polygon")
        out = cpp2hc.convert_files([os.path.join(d, "polygons.cpp"), os.path.join(d, "regular_polygon.hpp")])
        score, bad = cpp2hc.report(out)
        self.assertEqual(bad, [])
        self.assertEqual(score, 100.0)
        self.assertTrue(balanced(out))
        with open(os.path.join(d, "Polygons.HC")) as fh:
            self.assertEqual(out, fh.read(), "examples/polygon/Polygons.HC is stale, re-run cpp2hc")

    def test_hard_example_is_flagged_not_wrong(self):
        d = os.path.join(EX, "hard")
        out = cpp2hc.convert_files([os.path.join(d, "planner.cpp")])
        score, bad = cpp2hc.report(out)
        self.assertTrue(bad)
        self.assertLess(score, 100.0)
        self.assertTrue(balanced(out))

    def test_manifest(self):
        with open(os.path.join(EX, "polygon", "plugins.xml")) as fh:
            out = cpp2hc.manifest(fh.read())
        self.assertIn('PluginDescribe("polygon_plugins::Square","polygon_base::RegularPolygon",'
                      '"This is a square plugin.");', out)

    def test_cli_strict(self):
        d = os.path.join(EX, "hard")
        rc = cpp2hc.main(["convert", os.path.join(d, "planner.cpp"), "-o", os.devnull, "-q", "--strict"])
        self.assertEqual(rc, 2)


if __name__ == "__main__":
    unittest.main()
