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

I64 DdsInterop()
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
  RosIfaceLoadAll();
  MemSet(&x,0,sizeof(CInterop));
  pt=RosDdsStart(0,FALSE);
  if (!pt) {
    Print("can't open RTPS ports\n");
    return 1;
  }
  RosDdsAddPeer(pt,RosIpParse("127.0.0.1"));
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
  Print("RESULT holyc: remote participants=%d echoes=%d from_cyclone=%d big_string=%d -> %s\n",
	pt->remote->len,x.echoes,x.from_cy,x.big,"");
  RosNodeDel(n);
  RosExecDel(e);
  RosDdsStop();
  if (x.echoes>=5 && x.from_cy>=5 && x.big>=1)
    return 0;
  return 1;
}
