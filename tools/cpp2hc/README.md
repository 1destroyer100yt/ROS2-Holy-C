# cpp2hc: C++ ROS 2 plugins → HolyC

ROS 2 plugins (`pluginlib`) are C++ classes loaded from `.so` files. TempleOS can't load
those, so `cpp2hc` rewrites the plugin's C++ source into a HolyC file that registers
itself with `src/Plugin/PluginLib.HC`.

It's **dumb on purpose**. It uses simple, predictable rewrite rules instead of a real C++
compiler. Anything it isn't sure about gets marked on that exact line:

```
auto d = Point_dist(start,goal); //TODO(cpp2hc): auto - write the real type
```

That way you always know what to fix by hand, and it never quietly writes wrong code.

## Use

```
python3 cpp2hc.py convert my_plugin.hpp my_plugin.cpp -o MyPlugin.HC
python3 cpp2hc.py manifest plugins.xml -o Manifest.HC
```

It prints a **portability score**, the share of code lines with no TODO, and lists every TODO:

```
cpp2hc: portability score 93% (5 TODO)
  Planner.HC:77: auto - write the real type
  ...
```

Options:
- `--strict`: exit code 2 if any TODO is left (good for scripts)
- `--exact-ints`: `int`→`I32`, `short`→`I16` (default is `I64`, which is normal in HolyC)
- `-q`: no report

Needs only Python 3.8+, with no other dependencies. It runs on your Linux/Windows/Mac machine,
where the C++ source already lives. The output `.HC` file is what goes to TempleOS.

## What it converts

| C++ | HolyC |
|---|---|
| `class Square : public Base {...}` | `class Square:Base {...}` (when Base is in the input) |
| methods | `Square_area(Square *self, ...)` |
| constructor / destructor / field defaults | `Square_New()` / `Square_Del(self)` |
| `x_`, `this->x_` | `self->x_` |
| `const Point &p` | `Point *p` (call sites get fixed too) |
| `int double float bool char std::string uint8_t size_t ...` | `I64 F64 F64 Bool U8 U8 * U8 U64 ...` |
| `static_cast<int>(d)`, `(double)a` | `ToI64(d)`, `ToF64(a)` |
| `std::cout << ...`, `printf`, `RCLCPP_INFO(...)` | `Print(...)`, `RosLogInfo(...)` |
| `new T(...)`, `delete p`, `new T[n]` | `T_New(...)`, `T_Del(p)`, `CAlloc(n*sizeof(T))` |
| `for (int i = 0; ...)` | `I64 i;` hoisted + `for (i = 0; ...)` |
| `sqrt pow sin cos atan2 hypot min max M_PI ...` | `Sqrt Pow Sin Cos Arg(x,y) ... pi` |
| `nullptr true false 1.5f 10u` | `NULL TRUE FALSE 1.5 10` |
| `PLUGINLIB_EXPORT_CLASS(A, B)` | `PluginInit_<file>()` with `PluginClassAdd(...)` + `PluginMethodAdd(...)`, run automatically when TempleOS loads the file |
| `plugins.xml` | `PluginDescribe(...)` |

Flagged with a TODO: templates, STL containers, smart pointers, `auto`, lambdas,
overloads, operator overloading, non-const references, exceptions, string `+`, `rclcpp::`
APIs, and method calls on types it doesn't know.

## Using a converted plugin
```
PluginLoad("~/MyPlugin.HC");                       //compiles it and registers its classes
CPluginObj *o=PluginCreateAs("my_pkg::MyPlanner","nav_base::GlobalPlanner");
F64 (*area)(U8 *self)=PluginMethod(o,"area");
(*area)(o->self);
PluginDestroy(o);
```
See `examples/PluginNode.HC` for a node that picks its plugin from a parameter.

## Tests

```
cd tools/cpp2hc
python3 -m unittest discover -s tests
```

`examples/polygon` converts with zero TODOs. `examples/hard` shows what flagged output looks like.
