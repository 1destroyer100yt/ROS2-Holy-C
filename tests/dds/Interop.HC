//HolyC side of the Cyclone DDS interop test (tools/ddstest/run.sh).
//Uses the full stack: ROS nodes -> Bridge -> RTPS -> UDP.

Cd(__DIR__);;
#include "../../Load.HC"

class CInterop
{
  I64 echoes,from_cy,big;
};

U0 InteropEcho(CMsg_std_msgs_String *m,CInterop *x)
{
  if (RosStrStartsWith(m->data,"echo: holyc "))
    x->echoes++;
}

U0 InteropFromCy(CMsg_std_msgs_String *m,CInterop *x)
{
  if (RosStrStartsWith(m->data,"cyclone "))
    x->from_cy++;
}

U0 InteropAdd(CSrv_example_interfaces_AddTwoInts_Request *req,
	CSrv_example_interfaces_AddTwoInts_Response *resp,U8 *user)
{
  resp->sum=req->a+req->b;
}

U0 InteropBig(CMsg_std_msgs_String *m,CInterop *x)
{
  I64 i,n=StrLen(m->data);
  Bool ok=n==5000;
  for (i=0;ok && i<n;i++)
    if (m->data[i]!='B')
      ok=FALSE;
  if (ok)
    x->big++;
}

//peer_ip: where Cyclone is; mcast: use multicast discovery too.
I64 DdsInteropBody(I64 peer_ip,Bool mcast)
{
  CRtpsPart *pt;
  CRosNode *n;
  CRosExec *e=RosExecNew();
  CRosPub *chat,*big;
  CRosQoS rel,tl;
  CInterop x;
  CMsg_std_msgs_String m;
  I64 end,next=0,count=0,big_sent=0;
  CRtpsWriter *w;
  U8 *s;
  Bool node_ok,topic_ok,srvlist_ok,call_ok;
  MemSet(&x,0,sizeof(CInterop));
  pt=RosDdsStart(0,mcast);
  if (!pt) {
    Print("can't open RTPS ports\n");
    return 1;
  }
  RosDdsAddPeer(pt,peer_ip);
//@host: if (getenv("DDS_DEBUG")) ros_dds_debug=TRUE;
  pt->test_drop_every=7;	//lose every 7th user DATA: reliability must fix it
  n=RosNodeNew("holyc_node",NULL);
  RosExecAdd(e,n);
  RosQoSDefault(&rel);
  RosQoSTransientLocal(&tl,1);
  chat=RosPubNew(n,"chatter",ros_t_std_msgs_String,&rel);
  big=RosPubNew(n,"holyc_big",ros_t_std_msgs_String,&rel);
  RosSubNew(n,"chatter_echo",ros_t_std_msgs_String,&rel,&InteropEcho,&x);
  RosSubNew(n,"from_cyclone",ros_t_std_msgs_String,&rel,&InteropFromCy,&x);
  RosSubNew(n,"big_string",ros_t_std_msgs_String,&tl,&InteropBig,&x);
  RosServiceNew(n,"holyc_add",ros_srv_example_interfaces_AddTwoInts,&InteropAdd,NULL);
  end=RosTimeSteadyNs()+20*ROS_NS_PER_SEC;
  while (RosTimeSteadyNs()<end) {
    RosDdsSpin();
    RosExecSpinSome(e);
    if (RosTimeSteadyNs()>=next) {
      next=RosTimeSteadyNs()+50*ROS_NS_PER_MS;
      count++;
      m.data=MStrPrint("holyc %d",count);
      RosPublish(chat,&m);
      Free(m.data);
      w=big->dds;
      if (w->readers->len && count%10==5 && big_sent<3) {
        m.data=MAlloc(100001);
        MemSet(m.data,'H',100000);
        m.data[100000]=0;
        RosPublish(big,&m);
        Free(m.data);
        big_sent++;
      }
    }
    if (x.echoes>=5 && x.from_cy>=5 && x.big>=1 && count>60)
      break;
    Sleep(1);
  }
  //the CLI sees the other machine
  s=RosNodeListStr();
  node_ok=RosStrFind(s,"/cyclone_node\n")>=0 && RosStrFind(s,"/holyc_node\n")>=0;
  Free(s);
  s=RosTopicListStr(TRUE);
  topic_ok=RosStrFind(s,"/cyclone_only [std_msgs/msg/String]\n")>=0;
  Free(s);
  s=RosServiceListStr(TRUE);
  srvlist_ok=RosStrFind(s,"/add_two_ints [example_interfaces/srv/AddTwoInts]\n")>=0;
  Free(s);
  //call a service that lives in Cyclone, through the CLI
  ros_cli_assist=e;
  s=RosServiceCallStr("/add_two_ints","example_interfaces/srv/AddTwoInts","{a: 20, b: 22}");
  call_ok=RosStrFind(s,"sum: 42")>=0;
  Free(s);
  ros_cli_assist=NULL;
  Print("RESULT holyc: remote participants=%d echoes=%d from_cyclone=%d big_string=%d nodes=%d topics=%d services=%d call=%d\n",
	pt->remote->len,x.echoes,x.from_cy,x.big,node_ok,topic_ok,srvlist_ok,call_ok);
  RosNodeDel(n);
  RosExecDel(e);
  RosDdsStop();
  if (x.echoes>=5 && x.from_cy>=5 && x.big>=1 && node_ok && topic_ok && srvlist_ok && call_ok)
    return 0;
  return 1;
}

//Over the host's sockets, loopback (tools/ddstest/run.sh).
I64 DdsInterop()
{
  RosIfaceLoadAll();
  return DdsInteropBody(RosIpParse("127.0.0.1"),FALSE);
}

//Over OUR IP stack on a TAP device, multicast discovery: everything
//above the network card is what runs on TempleOS (tools/ddstest/run.sh tap).
I64 DdsInteropTap()
{
  CRosNetIf *nif;
  RosIfaceLoadAll();
  nif=RosTapOpen("rhc1",RosIpParse("10.78.0.1"));
  if (!nif) {
    Print("RESULT holyc: can't open TAP\n");
    return 1;
  }
  RosNetUp(nif);
  RosNetConfig(RosIpParse("10.78.0.2"),RosIpParse("255.255.255.0"),0);
  Print("READY\n");
//@host: fflush(stdout);
  return DdsInteropBody(RosIpParse("10.78.0.1"),TRUE);
}
