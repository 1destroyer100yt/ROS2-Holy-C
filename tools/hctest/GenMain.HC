//Host entry point for the interface generator (tools/gen_interfaces.sh).
#include "../../src/Load.HC"

I64 RosGenMain()
{
  if (RosIfaceGen("."))
    return 0;
  return 1;
}
