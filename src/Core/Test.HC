//Tiny unit test framework. Used by tests/ in TempleOS and on the
//host (tools/hctest). Prints one line per failure and a summary.
//
//  RosTestGroup("Vec");
//  RosTestI64("len after push",v->len,3);
//  ...
//  return RosTestSummary;   //0 when everything passed

I64 ros_test_pass=0;
I64 ros_test_fail=0;
U8 *ros_test_group="";

U0 RosTestGroup(U8 *name)
{
  ros_test_group=name;
}

U0 RosTestFailed(U8 *what)
{
  ros_test_fail++;
  Print("$$FG,RED$$FAIL$$FG$$ %s: %s\n",ros_test_group,what);
}

U0 RosTestTrue(U8 *what,Bool ok)
{
  if (ok)
    ros_test_pass++;
  else
    RosTestFailed(what);
}

U0 RosTestI64(U8 *what,I64 got,I64 want)
{
  if (got==want)
    ros_test_pass++;
  else {
    RosTestFailed(what);
    Print("     got %d (0x%X), want %d (0x%X)\n",got,got,want,want);
  }
}

U0 RosTestF64(U8 *what,F64 got,F64 want,F64 eps)
{
  if (Abs(got-want)<=eps)
    ros_test_pass++;
  else {
    RosTestFailed(what);
    Print("     got %.17g, want %.17g\n",got,want);
  }
}

U0 RosTestStr(U8 *what,U8 *got,U8 *want)
{
  if (got && want && !StrCmp(got,want))
    ros_test_pass++;
  else {
    RosTestFailed(what);
    if (!got) got="(null)";
    if (!want) want="(null)";
    Print("     got \"%s\", want \"%s\"\n",got,want);
  }
}

//Print the totals. Returns the number of failures.
I64 RosTestSummary()
{
  if (ros_test_fail)
    Print("$$FG,RED$$TESTS: %d passed, %d failed$$FG$$\n",ros_test_pass,ros_test_fail);
  else
    Print("$$FG,GREEN$$TESTS: %d passed, 0 failed$$FG$$\n",ros_test_pass);
  return ros_test_fail;
}
