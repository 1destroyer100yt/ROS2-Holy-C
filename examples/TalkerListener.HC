//The classic ROS 2 demo: a talker publishes "Hello World: N" on
///chatter twice a second, a listener prints what it hears.
//
//In TempleOS each node runs in its own task (they could be on
//different cores too):
//  #include "~/ROS2-Holy-C/examples/TalkerListener.HC"
//  TalkerListener;        //starts both tasks
//  RosShutdown;           //stops them
//On Linux (one executor, 2.2 seconds):
//  python3 tools/hctest/hc2c.py run examples/TalkerListener.HC --main TalkerListenerHost

Cd(__DIR__);;
#include "../Load.HC"

class CTalker
{
  CRosNode *node;
  CRosPub *pub;
  I64 count;
};

U0 TalkerTick(CTalker *t)
{
  CMsg_std_msgs_String *m=RosPubLoan(t->pub);
  U8 *line;
  t->count++;
  m->data=MStrPrint("Hello World: %d",t->count);
  line=MStrPrint("Publishing: '%s'",m->data);
  RosNodeLog(t->node,ROS_LOG_INFO,line);
  Free(line);
  RosPublishLoaned(t->pub,m);
}

CTalker *TalkerNew()
{
  CTalker *t=CAlloc(sizeof(CTalker));
  t->node=RosNodeNew("talker",NULL);
  t->pub=RosPubNew(t->node,"chatter",ros_t_std_msgs_String,NULL);
  RosTimerNew(t->node,500*ROS_NS_PER_MS,&TalkerTick,t);
  return t;
}

U0 ListenerHeard(CMsg_std_msgs_String *m,CRosNode *node)
{
  U8 *line=MStrPrint("I heard: [%s]",m->data);
  RosNodeLog(node,ROS_LOG_INFO,line);
  Free(line);
}

CRosNode *ListenerNew()
{
  CRosNode *n=RosNodeNew("listener",NULL);
  RosSubNew(n,"chatter",ros_t_std_msgs_String,NULL,&ListenerHeard,n);
  return n;
}

U0 TalkerTask(U8 *data)
{
  CTalker *t=TalkerNew();
  RosSpin(t->node);	//until RosShutdown
  RosNodeDel(t->node);
  Free(t);
}

U0 ListenerTask(U8 *data)
{
  CRosNode *n=ListenerNew();
  RosSpin(n);
  RosNodeDel(n);
}

//@host-skip-begin
U0 TalkerListener()
{
  ros_ok=TRUE;
  Spawn(&ListenerTask,NULL,"listener");
  Spawn(&TalkerTask,NULL,"talker");
}
//@host-skip-end

I64 TalkerListenerHost()
{
  CTalker *t;
  CRosNode *l;
  CRosExec *e=RosExecNew();
  RosIfaceLoadAll();
  t=TalkerNew();
  l=ListenerNew();
  RosExecAdd(e,t->node);
  RosExecAdd(e,l);
  RosExecSpinFor(e,2200*ROS_NS_PER_MS);
  RosNodeDel(t->node);
  RosNodeDel(l);
  RosExecDel(e);
  if (t->count!=4)
    return 1;
  Free(t);
  return 0;
}
