//HolyC as the client of real ROS 2 nodes (tools/ros2test/run.sh):
//calls demo_nodes_py's add_two_ints_server and action_tutorials_py's
//fibonacci_action_server, listens to demo_nodes_py's talker, reads a
//bag that `ros2 bag record` wrote and writes one for `ros2 bag info`.

Cd(__DIR__);;
#include "../../Load.HC"

U8 *ClientEnv(U8 *name)
{
  U8 *s=NULL;
//@host: s=(U8 *)getenv((char *)name);
  return s;
}

U8 client_heard[64];

U0 ClientOnChatter(CMsg_std_msgs_String *m,U8 *user)
{
  if (StrLen(m->data)<64)
    StrCpy(client_heard,m->data);
}

U0 ClientCheck(U8 *name,Bool ok,U8 *detail)
{
  if (ok)
    Print("CHECK %s ok\n",name);
  else
    Print("CHECK %s FAIL\n%s\n",name,detail);
}

I64 Ros2Client()
{
  CRtpsPart *pt;
  CRosNode *n;
  CRosBag *bag;
  CRosBagMsg *bm;
  CRosBagChan *bc;
  CMsg_std_msgs_String *str;
  U8 *s,*path;
  I64 i,good;
  Bool ok;
  RosIfaceLoadAll();
  pt=RosDdsStart(0,FALSE);
  if (!pt) {
    Print("can't open RTPS ports\n");
    return 1;
  }
  RosDdsAddPeer(pt,RosIpParse("127.0.0.1"));
  n=RosNodeNew("holyc_client",NULL);
  ros_cli_assist=RosNodeExec(n);
  //a service in Python
  s=NULL;
  for (i=0;i<5 && (!s || RosStrFind(s,"sum: 42")<0);i++) {
    Free(s);
    s=RosServiceCallStr("/add_two_ints","example_interfaces/srv/AddTwoInts","{a: 40, b: 2}");
  }
  ClientCheck("call_python_service",RosStrFind(s,"sum: 42")>=0,s);
  Free(s);
  //an action in Python, with a type we only know from its text
  RosActionDefine("action_tutorials_interfaces/action/Fibonacci",
	"int32 order\n---\nint32[] sequence\n---\nint32[] partial_sequence\n");
  s=RosActionSendGoalStr("/fibonacci","action_tutorials_interfaces/action/Fibonacci",
	"{order: 3}",TRUE,30*ROS_NS_PER_SEC);
  ClientCheck("python_action",RosStrFind(s,"Goal finished with status: SUCCEEDED")>=0 &&
	RosStrFind(s,"partial_sequence:")>=0,s);
  Free(s);
  //the Python talker
  client_heard[0]=0;
  RosSubNew(n,"/chatter",ros_t_std_msgs_String,NULL,&ClientOnChatter,NULL);
  for (i=0;i<1000 && !client_heard[0];i++)
    RosCliSpinOnce(n,10*ROS_NS_PER_MS);
  ClientCheck("hear_python_talker",RosStrStartsWith(client_heard,"Hello World: "),client_heard);
  //node list sees the Python nodes
  s=NULL;
  for (i=0;i<100 && (!s || RosStrFind(s,"/add_two_ints_server\n")<0);i++) {	//discovery
    Free(s);
    RosCliSpinOnce(n,100*ROS_NS_PER_MS);
    s=RosNodeListStr();
  }
  ClientCheck("node_list",RosStrFind(s,"/talker\n")>=0 && RosStrFind(s,"/add_two_ints_server\n")>=0,s);
  Free(s);
  //a bag ros2 recorded
  path=ClientEnv("BAG_IN");
  if (path) {
    bag=RosBagOpen(path);
    good=0;
    if (bag) {
      for (i=0;i<bag->msgs->len;i++) {
        bm=RosVecGetI64(bag->msgs,i);
        bc=RosBagChanById(bag,bm->chan);
        if (bc && bc->t==ros_t_std_msgs_String) {
          str=RosMsgNew(bc->t);
          if (RosCdrDeser(bc->t,bm->data,bm->len,str) && RosStrStartsWith(str->data,"Hello World: "))
            good++;
          RosMsgDel(bc->t,str);
        }
      }
      RosBagFree(bag);
    }
    s=MStrPrint("read %d good messages: %s",good,ros_bag_error);
    ClientCheck("read_ros2_bag",good>=2,s);
    Free(s);
  }
  //a bag for ros2 to read
  path=ClientEnv("BAG_OUT");
  if (path) {
    bag=RosBagCreate(path);
    str=RosMsgNew(ros_t_std_msgs_String);
    for (i=0;i<5;i++) {
      Free(str->data);
      str->data=MStrPrint("from holyc %d",i);
      RosBagWriteMsg(bag,"/holyc_chatter",ros_t_std_msgs_String,1700000000*ROS_NS_PER_SEC+i,str);
    }
    RosMsgDel(ros_t_std_msgs_String,str);
    ok=RosBagClose(bag);
    RosBagFree(bag);
    ClientCheck("write_bag",ok,ros_bag_error);
  }
  ros_cli_assist=NULL;
  RosNodeDel(n);
  RosDdsStop();
  return 0;
}
