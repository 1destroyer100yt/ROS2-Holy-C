//HolyC side of the real ROS 2 test (tools/ros2test/run.sh): a node
//with a talker, a listener, a service, an action and parameters, on
//the network for DEMO_SECONDS while the ros2 command line pokes at it.

Cd(__DIR__);;
#include "../../Load.HC"

class CDemo
{
  I64 heard,served;
  CRosGoal *g;
  I64 fi,fn,goals;
  U8 last_heard[64];
};

U0 DemoOnChatter(CMsg_std_msgs_String *m,CDemo *d)
{
  d->heard++;
  if (StrLen(m->data)<64)
    StrCpy(d->last_heard,m->data);
}

U0 DemoAdd(CSrv_example_interfaces_AddTwoInts_Request *req,
	CSrv_example_interfaces_AddTwoInts_Response *resp,CDemo *d)
{
  resp->sum=req->a+req->b;
  d->served++;
}

U0 DemoFibAccepted(CRosGoal *g,CDemo *d)
{
  CAct_example_interfaces_Fibonacci_Goal *goal=g->goal;
  d->g=g;
  d->fn=goal->order;
  d->fi=0;
  d->goals++;
  RosGoalExecute(g);
}

U0 DemoFibTick(CDemo *d)
{
  CAct_example_interfaces_Fibonacci_Feedback fb;
  CRosField *sf=RosTypeField(ros_t_example_interfaces_Fibonacci_Feedback,"sequence");
  I64 j,*v;
  if (!d->g)
    return;
  if (RosGoalIsCanceling(d->g)) {
    RosGoalCanceled(d->g,NULL);
    d->g=NULL;
    return;
  }
  MemSet(&fb,0,sizeof(fb));
  RosSeqResize(sf,&fb,d->fi+1);
  v=fb.sequence.data;
  for (j=0;j<=d->fi;j++) {
    if (j<2)
      v[j]=j;
    else
      v[j]=v[j-1]+v[j-2];
  }
  if (d->fi<d->fn) {
    RosGoalFeedback(d->g,&fb);
    d->fi++;
  } else {
    RosGoalSucceed(d->g,&fb);
    d->g=NULL;
  }
  RosMsgFini(ros_t_example_interfaces_Fibonacci_Feedback,&fb);
}

I64 demo_seconds=30;	//on TempleOS set it before Ros2DemoE1000 (hw.py does)

I64 DemoSeconds()
{
  I64 s=demo_seconds;
  U8 *e=NULL;
//@host: e=(U8 *)getenv("DEMO_SECONDS");
  if (e)
    RosStrToI64(e,&s);
  return s;
}

//Run the demo node: DDS with peer (an IP) as unicast peer.
I64 Ros2DemoBody(I64 peer,Bool mcast)
{
  CRtpsPart *pt;
  CRosNode *n;
  CRosExec *e=RosExecNew();
  CRosPub *chat;
  CRosActionServer *as;
  CMsg_std_msgs_String m;
  CDemo d;
  I64 end,next=0,count=0;
  U8 *stop=NULL;
//@host: stop=(U8 *)getenv("DEMO_STOP");	//a file: quit when it appears
  MemSet(&d,0,sizeof(CDemo));
  pt=RosDdsStart(0,mcast);
  if (!pt) {
    Print("can't open RTPS ports\n");
    return 1;
  }
  RosDdsAddPeer(pt,peer);
  n=RosNodeNew("holyc_demo",NULL);
  RosExecAdd(e,n);
  RosParamDeclare(n,"greeting","Hello from HolyC");
  chat=RosPubNew(n,"chatter",ros_t_std_msgs_String,NULL);
  RosSubNew(n,"ros_chatter",ros_t_std_msgs_String,NULL,&DemoOnChatter,&d);
  RosServiceNew(n,"add_two_ints",ros_srv_example_interfaces_AddTwoInts,&DemoAdd,&d);
  as=RosActionServerNew(n,"fibonacci",ros_act_example_interfaces_Fibonacci,NULL,NULL,
	&DemoFibAccepted,&d);
  RosTimerNew(n,50*ROS_NS_PER_MS,&DemoFibTick,&d);
  Print("READY\n");
//@host: fflush(stdout);
  end=RosTimeSteadyNs()+DemoSeconds()*ROS_NS_PER_SEC;
  while (RosTimeSteadyNs()<end && ros_ok) {
    RosDdsSpin();
    RosExecSpinSome(e);
    if (RosTimeSteadyNs()>=next) {
      next=RosTimeSteadyNs()+100*ROS_NS_PER_MS;
      count++;
      m.data=MStrPrint("%s: %d",RosParamStr(n,"greeting"),count);
      RosPublish(chat,&m);
      Free(m.data);
    }
    Sleep(1);
//@host: if (stop && count%10==0 && !access((char *)stop,F_OK)) break;
  }
  Print("RESULT holyc: heard=%d last='%s' served=%d goals=%d remote=%d\n",d.heard,
	d.last_heard,d.served,d.goals,pt->remote->len);
  RosActionServerDel(as);
  RosNodeDel(n);
  RosExecDel(e);
  RosDdsStop();
  return 0;
}

//On this machine, over the host's sockets (tools/ros2test/run.sh).
I64 Ros2Demo()
{
  RosIfaceLoadAll();
  return Ros2DemoBody(RosIpParse("127.0.0.1"),FALSE);
}

//The whole thing on our own stack and e1000 driver, as /init of a
//Linux guest in QEMU (tools/e1000test/run.sh ros2): what TempleOS runs.
I64 Ros2DemoE1000()
{
  CRosNetIf *nif;
  I64 r=1;
//@host: mkdir("/proc",0555); mount("proc","/proc","proc",0,NULL); iopl(3);
  RosIfaceLoadAll();
  nif=E1000Open();
  if (nif) {
    RosNetUp(nif);
    if (RosNetDhcp(30*ROS_NS_PER_SEC)) {
      Print("DHCP %s\n",RosIpStr(ros_netif->ip));
      r=Ros2DemoBody(ros_netif->gw,TRUE);
    } else
      Print("RESULT holyc: no DHCP answer\n");
  } else
    Print("RESULT holyc: no e1000 found\n");
//@host: fflush(stdout); sync(); reboot(RB_POWER_OFF);
  return r;
}
