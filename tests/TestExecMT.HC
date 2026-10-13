//Tests for the multi-threaded executor (src/Graph/Exec.HC).

#define MT_NODES	4
#define MT_PER_NODE	40

class CTestMT
{
  CRosExec *e;
  I64 lock,inside,max_inside,done,overlap;
  I64 busy[MT_NODES];	//callbacks running in each node right now
};

class CTestMTSub
{
  CTestMT *mt;
  I64 node;
};

U0 TestMTOnMsg(CMsg_std_msgs_Int64 *m,CTestMTSub *s)
{
  CTestMT *mt=s->mt;
  RosLockGet(&mt->lock);
  mt->inside++;
  if (mt->inside>mt->max_inside)
    mt->max_inside=mt->inside;
  mt->busy[s->node]++;
  if (mt->busy[s->node]>1)
    mt->overlap++;	//two callbacks of one node at once: not allowed
  RosLockPut(&mt->lock);
  RosSleepNs(2*ROS_NS_PER_MS);	//"work"
  RosLockGet(&mt->lock);
  mt->busy[s->node]--;
  mt->inside--;
  mt->done++;
  if (mt->done==MT_NODES*MT_PER_NODE*2)
    RosExecStop(mt->e);
  RosLockPut(&mt->lock);
}

U0 TestExecMT()
{
  CRosNode *nodes[MT_NODES],*pn;
  CRosPub *pubs[MT_NODES];
  CTestMTSub subs[MT_NODES*2];
  CTestMT mt;
  CRosQoS q;
  CMsg_std_msgs_Int64 m;
  U8 *name;
  I64 i,j,t0,took;
  RosTestGroup("Multi-threaded executor");
  MemSet(&mt,0,sizeof(CTestMT));
  mt.e=RosExecNew();
  RosQoSDefault(&q);
  q.depth=1000;
  pn=RosNodeNew("mt_pub",NULL);
  for (i=0;i<MT_NODES;i++) {
    name=MStrPrint("mt_node%d",i);
    nodes[i]=RosNodeNew(name,NULL);
    Free(name);
    RosExecAdd(mt.e,nodes[i]);
    name=MStrPrint("mt_topic%d",i);
    pubs[i]=RosPubNew(pn,name,ros_t_std_msgs_Int64,&q);
    for (j=0;j<2;j++) {	//two subscriptions per node: they must take turns
      subs[i*2+j].mt=&mt;
      subs[i*2+j].node=i;
      RosSubNew(nodes[i],name,ros_t_std_msgs_Int64,&q,&TestMTOnMsg,&subs[i*2+j]);
    }
    Free(name);
  }
  for (j=0;j<MT_PER_NODE;j++)
    for (i=0;i<MT_NODES;i++) {
      m.data=j;
      RosPublish(pubs[i],&m);
    }
  t0=RosTimeSteadyNs();
  RosExecSpinMT(mt.e,MT_NODES);
  took=RosTimeSteadyNs()-t0;
  RosTestI64("every callback ran",mt.done,MT_NODES*MT_PER_NODE*2);
  RosTestI64("one callback per node at a time",mt.overlap,0);
  RosTestTrue("nodes ran in parallel",mt.max_inside>=2);
  RosTestTrue("faster than one thread",took<MT_NODES*MT_PER_NODE*2*2*ROS_NS_PER_MS*3/4);
  RosTestI64("workers finished",mt.e->workers,0);
  for (i=0;i<MT_NODES;i++)
    RosNodeDel(nodes[i]);
  RosNodeDel(pn);
  RosExecDel(mt.e);
}
