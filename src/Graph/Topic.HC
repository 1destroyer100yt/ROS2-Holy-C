//Topic: publishers and subscriptions.
//
//  CRosPub *pub=RosPubNew(node,"chatter",ros_t_std_msgs_String,NULL);
//  CMsg_std_msgs_String *m=RosPubLoan(pub);   //zero copy
//  m->data=StrNew("hi");
//  RosPublishLoaned(pub,m);
//
//  U0 OnMsg(CMsg_std_msgs_String *m,U8 *user) {...}
//  RosSubNew(node,"chatter",ros_t_std_msgs_String,NULL,&OnMsg,NULL);
//
//Callbacks run in the executor that spins the subscription's node.
//Messages are shared between subscribers: treat them as read-only.

U8 ros_graph_error[128];

U0 RosGraphErr(U8 *what,U8 *name)
{
  RosNameErr(what,name);
  StrCpy(ros_graph_error,ros_name_error);
}

extern U8 *RosNodeResolve(CRosNode *n,U8 *name);

//Queue one message on a subscription (graph lock held).
U0 RosSubPush(CRosSub *s,CRosShared *sh)
{
  CRosShared *old,*item[1];
  I64 depth=s->qos.depth;
  if (s->qos.history==ROS_QOS_KEEP_ALL || depth<1)
    depth=ROS_KEEP_ALL_MAX;
  RosSharedRetain(sh);
  RosLockGet(&s->lock);
  if (RosRingCount(s->q)>=depth) {
    if (s->qos.history==ROS_QOS_KEEP_ALL) {	//full: refuse the new one
      RosLockPut(&s->lock);
      RosSharedRelease(sh);
      s->dropped++;
      return;
    }
    RosRingPop(s->q,&old);	//keep last: drop the oldest
    RosSharedRelease(old);
    s->dropped++;
  }
  item[0]=sh;
  RosRingPush(s->q,item);
  RosLockPut(&s->lock);
}

//Can pub feed sub? Counts incompatible pairs once.
Bool RosPubSubMatch(CRosPub *p,CRosSub *s)
{
  return RosQoSCompatible(&p->qos,&s->qos);
}

CRosPub *RosPubNew(CRosNode *n,U8 *topic,CRosType *t,CRosQoS *qos)
{
  U8 *fqn=RosNodeResolve(n,topic);
  CRosPub *p;
  CRosTopic *tp;
  CRosSub *s;
  I64 i;
  if (!fqn)
    return NULL;
  RosGraphLock();
  tp=RosTopicGet(fqn,t);
  if (tp->type!=t) {
    RosTopicGc(tp);
    RosGraphUnlock();
    RosGraphErr("topic already has another type",fqn);
    Free(fqn);
    return NULL;
  }
  p=CAlloc(sizeof(CRosPub));
  p->node=n;
  p->topic=tp;
  if (qos)
    MemCpy(&p->qos,qos,sizeof(CRosQoS));
  else
    RosQoSDefault(&p->qos);
  p->history=RosVecNew(8);
  p->id=ros_graph->next_id++;
  RosVecPushI64(tp->pubs,p);
  RosVecPushI64(n->pubs,p);
  for (i=0;i<tp->subs->len;i++) {
    s=RosVecGetI64(tp->subs,i);
    if (!RosPubSubMatch(p,s)) {
      p->incompatible++;
      s->incompatible++;
    }
  }
  RosGraphUnlock();
  Free(fqn);
  return p;
}

//A message you fill in and hand to RosPublishLoaned (no copy).
//If you don't publish it, give it back with RosMsgRelease.
U8 *RosPubLoan(CRosPub *p)
{
  return RosSharedMsg(RosSharedNew(p->topic->type));
}

//Publish a loaned message. Ownership passes to ROS.
U0 RosPublishLoaned(CRosPub *p,U8 *msg)
{
  CRosShared *sh=RosSharedOf(msg),*old;
  CRosSub *s;
  I64 i,keep;
  sh->stamp_ns=RosTimeSteadyNs();
  sh->pub_id=p->id;
  RosGraphLock();
  for (i=0;i<p->topic->subs->len;i++) {
    s=RosVecGetI64(p->topic->subs,i);
    if (RosPubSubMatch(p,s))
      RosSubPush(s,sh);
  }
  if (p->qos.durability==ROS_QOS_TRANSIENT_LOCAL) {
    keep=p->qos.depth;
    if (keep<1)
      keep=1;
    RosSharedRetain(sh);
    RosVecPushI64(p->history,sh);
    while (p->history->len>keep) {
      old=RosVecGetI64(p->history,0);
      RosVecRemove(p->history,0);
      RosSharedRelease(old);
    }
  }
  p->sent++;
  RosGraphUnlock();
  RosSharedRelease(sh);
}

//Publish a copy of msg (msg stays yours).
U0 RosPublish(CRosPub *p,U8 *msg)
{
  CRosType *t=p->topic->type;
  CRosShared *sh=CAlloc(sizeof(CRosShared)+t->size);
  sh->refs=1;
  sh->type=t;
  MemCpy(RosSharedMsg(sh),msg,t->size);
  RosMsgCopyFix(t,RosSharedMsg(sh));
  RosPublishLoaned(p,RosSharedMsg(sh));
}

//How many subscriptions this publisher reaches.
I64 RosPubSubCount(CRosPub *p)
{
  I64 i,n=0;
  RosGraphLock();
  for (i=0;i<p->topic->subs->len;i++)
    if (RosPubSubMatch(p,RosVecGetI64(p->topic->subs,i)))
      n++;
  RosGraphUnlock();
  return n;
}

U0 RosPubDel(CRosPub *p)
{
  I64 i;
  if (!p)
    return;
  RosGraphLock();
  RosVecRemovePtr(p->topic->pubs,p);
  RosVecRemovePtr(p->node->pubs,p);
  for (i=0;i<p->history->len;i++)
    RosSharedRelease(RosVecGetI64(p->history,i));
  RosVecDel(p->history);
  RosTopicGc(p->topic);
  RosGraphUnlock();
  Free(p);
}

//cb may be NULL: then read messages yourself with RosSubTake.
CRosSub *RosSubNew(CRosNode *n,U8 *topic,CRosType *t,CRosQoS *qos,
	U0 (*cb)(U8 *msg,U8 *user),U8 *user)
{
  U8 *fqn=RosNodeResolve(n,topic);
  CRosSub *s;
  CRosTopic *tp;
  CRosPub *p;
  I64 i,j,depth;
  if (!fqn)
    return NULL;
  RosGraphLock();
  tp=RosTopicGet(fqn,t);
  if (tp->type!=t) {
    RosTopicGc(tp);
    RosGraphUnlock();
    RosGraphErr("topic already has another type",fqn);
    Free(fqn);
    return NULL;
  }
  s=CAlloc(sizeof(CRosSub));
  s->node=n;
  s->topic=tp;
  if (qos)
    MemCpy(&s->qos,qos,sizeof(CRosQoS));
  else
    RosQoSDefault(&s->qos);
  depth=s->qos.depth;
  if (s->qos.history==ROS_QOS_KEEP_ALL || depth<1)
    depth=ROS_KEEP_ALL_MAX;
  s->q=RosRingNew(8,depth);
  s->cb=cb;
  s->user=user;
  s->id=ros_graph->next_id++;
  RosVecPushI64(tp->subs,s);
  RosVecPushI64(n->subs,s);
  for (i=0;i<tp->pubs->len;i++) {
    p=RosVecGetI64(tp->pubs,i);
    if (!RosPubSubMatch(p,s)) {
      p->incompatible++;
      s->incompatible++;
    } else if (s->qos.durability==ROS_QOS_TRANSIENT_LOCAL)
      for (j=0;j<p->history->len;j++)	//late joiner gets the history
        RosSubPush(s,RosVecGetI64(p->history,j));
  }
  RosGraphUnlock();
  Free(fqn);
  return s;
}

//Next queued message (yours: RosMsgRelease it), or NULL. Messages
//older than the QoS lifespan are thrown away.
U8 *RosSubTake(CRosSub *s)
{
  CRosShared *sh;
  while (TRUE) {
    RosLockGet(&s->lock);
    if (!RosRingPop(s->q,&sh)) {
      RosLockPut(&s->lock);
      return NULL;
    }
    RosLockPut(&s->lock);
    if (s->qos.lifespan_ns && RosTimeSteadyNs()-sh->stamp_ns>s->qos.lifespan_ns) {
      s->expired++;
      RosSharedRelease(sh);
      continue;
    }
    s->received++;
    return RosSharedMsg(sh);
  }
}

I64 RosSubPending(CRosSub *s)
{
  I64 n;
  RosLockGet(&s->lock);
  n=RosRingCount(s->q);
  RosLockPut(&s->lock);
  return n;
}

I64 RosSubPubCount(CRosSub *s)
{
  I64 i,n=0;
  RosGraphLock();
  for (i=0;i<s->topic->pubs->len;i++)
    if (RosPubSubMatch(RosVecGetI64(s->topic->pubs,i),s))
      n++;
  RosGraphUnlock();
  return n;
}

U0 RosSubDel(CRosSub *s)
{
  CRosShared *sh;
  if (!s)
    return;
  RosGraphLock();
  RosVecRemovePtr(s->topic->subs,s);
  RosVecRemovePtr(s->node->subs,s);
  RosTopicGc(s->topic);
  RosGraphUnlock();
  while (RosRingPop(s->q,&sh))
    RosSharedRelease(sh);
  RosRingDel(s->q);
  Free(s);
}

//Run the callback for one queued message. TRUE if there was one.
Bool RosSubExecOne(CRosSub *s)
{
  U8 *m;
  if (!s->cb)
    return FALSE;
  m=RosSubTake(s);
  if (!m)
    return FALSE;
  (*s->cb)(m,s->user);
  RosMsgRelease(m);
  return TRUE;
}
