//Services: a server that adds two numbers and a client that asks it.
//  #include "~/ROS2-Holy-C/examples/AddTwoInts.HC"
//  AddTwoInts(2,3);
//On Linux:
//  python3 tools/hctest/hc2c.py run examples/AddTwoInts.HC --main AddTwoIntsHost

Cd(__DIR__);;
#include "../Load.HC"

U0 AddTwoIntsServe(CSrv_example_interfaces_AddTwoInts_Request *req,
	CSrv_example_interfaces_AddTwoInts_Response *resp,CRosNode *node)
{
  U8 *line;
  resp->sum=req->a+req->b;
  line=MStrPrint("Incoming request a: %d b: %d",req->a,req->b);
  RosNodeLog(node,ROS_LOG_INFO,line);
  Free(line);
}

//Runs a server and a client in this task, makes one call.
//Returns the sum (or -1 if the call failed).
I64 AddTwoInts(I64 a,I64 b)
{
  CRosNode *server=RosNodeNew("add_two_ints_server",NULL);
  CRosNode *client=RosNodeNew("add_two_ints_client",NULL);
  CRosExec *e=RosExecNew();
  CRosClient *c;
  CSrv_example_interfaces_AddTwoInts_Request req;
  CSrv_example_interfaces_AddTwoInts_Response *resp;
  U8 *line;
  I64 sum=-1;
  RosExecAdd(e,server);
  RosExecAdd(e,client);
  RosServiceNew(server,"add_two_ints",ros_srv_example_interfaces_AddTwoInts,
	&AddTwoIntsServe,server);
  c=RosClientNew(client,"add_two_ints",ros_srv_example_interfaces_AddTwoInts);
  if (!RosClientWait(c,e,ROS_NS_PER_SEC))
    RosNodeLog(client,ROS_LOG_ERROR,"service not available");
  else {
    req.a=a;
    req.b=b;
    resp=RosClientCallWait(c,e,&req,ROS_NS_PER_SEC);
    if (resp) {
      sum=resp->sum;
      line=MStrPrint("Result of add_two_ints: for %d + %d = %d",a,b,sum);
      RosNodeLog(client,ROS_LOG_INFO,line);
      Free(line);
      RosMsgDel(ros_t_example_interfaces_AddTwoInts_Response,resp);
    }
  }
  RosNodeDel(server);
  RosNodeDel(client);
  RosExecDel(e);
  return sum;
}

I64 AddTwoIntsHost()
{
  RosIfaceLoadAll();
  if (AddTwoInts(2,3)!=5)
    return 1;
  return 0;
}
