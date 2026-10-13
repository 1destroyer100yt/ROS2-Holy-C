//Small helpers that converted C++ code (cpp2hc output) relies on.

//F64 min/max for converted std::min/std::max.
F64 CppMin(F64 a,F64 b)
{
  if (a<b)
    return a;
  return b;
}

F64 CppMax(F64 a,F64 b)
{
  if (a>b)
    return a;
  return b;
}
