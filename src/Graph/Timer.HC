//Timer: call cb(user) every period. Runs in the node's executor.
//
//  U0 Tick(U8 *user) {...}
//  RosTimerNew(node,500*ROS_NS_PER_MS,&Tick,NULL);

CRosTimer *RosTimerNew(CRosNode *n,I64 period_ns,U0 (*cb)(U8 *user),U8 *user)
{
  CRosTimer *t=CAlloc(sizeof(CRosTimer));
  if (period_ns<1)
    period_ns=1;
  t->node=n;
  t->period_ns=period_ns;
  t->next_ns=RosTimeSteadyNs()+period_ns;
  t->cb=cb;
  t->user=user;
  RosGraphLock();
  RosVecPushI64(n->timers,t);
  RosGraphUnlock();
  return t;
}

U0 RosTimerCancel(CRosTimer *t)
{
  t->canceled=TRUE;
}

U0 RosTimerReset(CRosTimer *t)
{
  t->canceled=FALSE;
  t->next_ns=RosTimeSteadyNs()+t->period_ns;
}

//Nanoseconds until it fires (0 if due, -1 if canceled).
I64 RosTimerLeft(CRosTimer *t)
{
  I64 left;
  if (t->canceled)
    return -1;
  left=t->next_ns-RosTimeSteadyNs();
  if (left<0)
    return 0;
  return left;
}

//Fire if due. TRUE if it ran. If we fell more than a period behind,
//skip the missed ticks instead of firing a burst (like rclcpp).
Bool RosTimerExec(CRosTimer *t)
{
  I64 now=RosTimeSteadyNs();
  if (t->canceled || now<t->next_ns)
    return FALSE;
  t->next_ns+=t->period_ns;
  if (t->next_ns<=now)
    t->next_ns=now+t->period_ns;
  t->calls++;
  (*t->cb)(t->user);
  return TRUE;
}

U0 RosTimerDel(CRosTimer *t)
{
  if (!t)
    return;
  RosGraphLock();
  RosVecRemovePtr(t->node->timers,t);
  RosGraphUnlock();
  Free(t);
}
