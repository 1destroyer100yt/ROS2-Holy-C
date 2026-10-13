//Run every ROS2-Holy-C unit test. In TempleOS:
//  #include "~/ROS2-Holy-C/tests/Run.HC"
//  RosTestsRun;
//On Linux/CI (no TempleOS needed):
//  python3 tools/hctest/hc2c.py run tests/Run.HC --main RosTestsRun

Cd(__DIR__);;
#include "../Load.HC"
Cd(__DIR__);;
#include "../src/Core/Test.HC"
#include "TestCore.HC"
#include "TestMsg.HC"
#include "TestLayout.HC"
#include "TestGolden.HC"
#include "TestGraph.HC"
#include "TestTools.HC"
#include "../tools/cpp2hc/examples/polygon/Polygons.HC"
#include "TestPlugin.HC"
#include "TestTf.HC"
#include "TestBag.HC"
#include "TestHashGolden.HC"
#include "TestHash.HC"
#include "TestAction.HC"
#include "TestExecMT.HC"
#include "TestRviz.HC"

I64 RosTestsRun()
{
  ros_test_pass=0;
  ros_test_fail=0;
  RosIfaceLoadAll();
  TestCore();
  TestMsg();
  TestLayout();
  TestGolden();
  TestGraph();
  TestTools();
  TestPlugin();
  TestTf();
  TestBag();
  TestHash();
  TestAction();
  TestExecMT();
  TestRviz();
  return RosTestSummary();
}
