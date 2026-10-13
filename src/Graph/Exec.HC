//Exec: run callbacks (timers, subscriptions, services, client
//responses) for a set of nodes. One executor per task: callbacks run
//in the task that spins.
//
//  CRosExec *e=RosExecNew();
//  RosExecAdd(e,node);
//  RosExecSpin(e);			//until RosShutdown() / e->stop
//
//Or just RosSpin(node) for one node.

CRosExec *RosExecNew()
{
  CRosExec *e=CAlloc(sizeof(CRosExec));
  e->nodes=RosVecNew(8);
  return e;
}

U0 RosExecDel(CRosExec *e)
{
  I64 i;
  CRosNode *n;
  if (!e)
    return;
  for (i=0;i<e->nodes->len;i++) {
    n=RosVecGetI64(e->nodes,i);
    n->exec=NULL;
  }
  RosVecDel(e->nodes);
  Free(e);
}

U0 RosExecAdd(CRosExec *e,CRosNode *n)
{
  if (n->exec==e)
    return;
  if (n->exec)
    RosVecRemovePtr(n->exec->nodes,n);
  n->exec=e;
  RosVecPushI64(e->nodes,n);
}

U0 RosExecRemove(CRosExec *e,CRosNode *n)
{
  if (RosVecRemovePtr(e->nodes,n))
    n->exec=NULL;
}

//Run everything that's ready right now, once each (messages that
//arrive during this call wait for the next one). Returns how many
//callbacks ran.
I64 RosExecSpinSome(CRosExec *e)
{
  I64 i,j,k,n_ready,work=0;
  CRosNode *n;
  CRosSub *s;
  for (i=0;i<e->nodes->len;i++) {
    n=RosVecGetI64(e->nodes,i);
    for (j=0;j<n->timers->len;j++)
      if (RosTimerExec(RosVecGetI64(n->timers,j)))
        work++;
    for (j=0;j<n->subs->len;j++) {
      s=RosVecGetI64(n->subs,j);
      n_ready=RosSubPending(s);
      for (k=0;k<n_ready && j<n->subs->len;k++)
        if (RosSubExecOne(s))
          work++;
    }
    for (j=0;j<n->srvs->len;j++)
      while (j<n->srvs->len && RosServiceExecOne(RosVecGetI64(n->srvs,j)))
        work++;
    for (j=0;j<n->clis->len;j++)
      while (j<n->clis->len && RosClientExecOne(RosVecGetI64(n->clis,j)))
        work++;
  }
  return work;
}

//Nanoseconds until the next timer of these nodes is due (max cap).
I64 RosExecNextTimer(CRosExec *e,I64 cap)
{
  I64 i,j,left,best=cap;
  CRosNode *n;
  for (i=0;i<e->nodes->len;i++) {
    n=RosVecGetI64(e->nodes,i);
    for (j=0;j<n->timers->len;j++) {
      left=RosTimerLeft(RosVecGetI64(n->timers,j));
      if (left>=0 && left<best)
        best=left;
    }
  }
  return best;
}

//Wait up to timeout_ns for work, run what's ready. TRUE if anything ran.
Bool RosExecSpinOnce(CRosExec *e,I64 timeout_ns)
{
  I64 end=RosTimeSteadyNs()+timeout_ns,left,wait;
  while (TRUE) {
    if (RosExecSpinSome(e))
      return TRUE;
    left=end-RosTimeSteadyNs();
    if (left<=0 || !ros_ok || e->stop)
      return FALSE;
    //messages can come from other tasks at any time: nap in small steps
    wait=RosExecNextTimer(e,left);
    if (wait>ROS_NS_PER_MS)
      wait=ROS_NS_PER_MS;
    if (wait>0)
      RosSleepNs(wait);
    else
      Yield();
  }
  return FALSE;
}

//Spin until RosShutdown() or e->stop.
U0 RosExecSpin(CRosExec *e)
{
  e->stop=FALSE;
  while (ros_ok && !e->stop)
    RosExecSpinOnce(e,100*ROS_NS_PER_MS);
}

//Spin for a while.
U0 RosExecSpinFor(CRosExec *e,I64 ns)
{
  I64 end=RosTimeSteadyNs()+ns,left;
  while (ros_ok && !e->stop) {
    left=end-RosTimeSteadyNs();
    if (left<=0)
      break;
    RosExecSpinOnce(e,left);
  }
}

//Spin until *flag is TRUE or the timeout. TRUE if the flag got set.
Bool RosExecSpinUntil(CRosExec *e,Bool *flag,I64 timeout_ns)
{
  I64 end=RosTimeSteadyNs()+timeout_ns,left;
  while (!*flag && ros_ok) {
    left=end-RosTimeSteadyNs();
    if (left<=0)
      break;
    RosExecSpinOnce(e,left);
  }
  return *flag;
}

//One-node shortcuts: each node gets its own executor on demand.
CRosExec *RosNodeExec(CRosNode *n)
{
  if (!n->exec)
    RosExecAdd(RosExecNew(),n);
  return n->exec;
}

U0 RosSpin(CRosNode *n)
{
  RosExecSpin(RosNodeExec(n));
}

I64 RosSpinSome(CRosNode *n)
{
  return RosExecSpinSome(RosNodeExec(n));
}

//---- blocking service call

class CRosSyncCall
{
  CRosType *t;
  U8 *resp;
  Bool done;
};

U0 RosSyncCallDone(U8 *resp,I64 seq,CRosSyncCall *sc)
{
  sc->resp=RosMsgDup(sc->t,resp);
  sc->done=TRUE;
}

//Call and wait (spinning e) for the answer. Returns the response
//(yours: RosMsgDel(st->resp,...)) or NULL on timeout / no service.
//e must be able to run the client's node, and the server's node too
//if it lives in this task.
U8 *RosClientCallWait(CRosClient *c,CRosExec *e,U8 *req,I64 timeout_ns)
{
  CRosSyncCall sc;
  I64 seq;
  sc.t=c->st->resp;
  sc.resp=NULL;
  sc.done=FALSE;
  seq=RosClientCall(c,req,&RosSyncCallDone,&sc);
  if (seq<0)
    return NULL;
  if (!RosExecSpinUntil(e,&sc.done,timeout_ns)) {
    RosClientCancel(c,seq);
    RosGraphErr("service call timed out",c->name);
    return NULL;
  }
  return sc.resp;
}

//Wait (spinning e) until the client's service exists.
Bool RosClientWait(CRosClient *c,CRosExec *e,I64 timeout_ns)
{
  I64 end=RosTimeSteadyNs()+timeout_ns;
  while (!RosClientReady(c)) {
    if (RosTimeSteadyNs()>=end || !ros_ok)
      return FALSE;
    RosExecSpinOnce(e,10*ROS_NS_PER_MS);
  }
  return TRUE;
}
