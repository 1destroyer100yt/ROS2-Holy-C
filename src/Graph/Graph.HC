//Graph: QoS, the classes for nodes/topics/publishers/subscriptions/
//timers/services/clients, shared (zero-copy) messages, and the
//global registry every task sees.

//---- QoS (same names and rules as ROS 2)

#define ROS_QOS_KEEP_LAST	1
#define ROS_QOS_KEEP_ALL	2
#define ROS_QOS_RELIABLE	1
#define ROS_QOS_BEST_EFFORT	2
#define ROS_QOS_VOLATILE	1
#define ROS_QOS_TRANSIENT_LOCAL	2

#define ROS_KEEP_ALL_MAX	1000	//KEEP_ALL is capped at this many queued

class CRosQoS
{
  I64 history,depth,reliability,durability;
  I64 lifespan_ns;	//0 = forever
};

U0 RosQoSSet(CRosQoS *q,I64 history,I64 depth,I64 reliability,I64 durability)
{
  q->history=history;
  q->depth=depth;
  q->reliability=reliability;
  q->durability=durability;
  q->lifespan_ns=0;
}

//rclcpp's default: keep last 10, reliable, volatile.
U0 RosQoSDefault(CRosQoS *q)
{
  RosQoSSet(q,ROS_QOS_KEEP_LAST,10,ROS_QOS_RELIABLE,ROS_QOS_VOLATILE);
}

//Sensor data: keep last 5, best effort.
U0 RosQoSSensorData(CRosQoS *q)
{
  RosQoSSet(q,ROS_QOS_KEEP_LAST,5,ROS_QOS_BEST_EFFORT,ROS_QOS_VOLATILE);
}

//"Latched": late subscribers get the last depth messages.
U0 RosQoSTransientLocal(CRosQoS *q,I64 depth)
{
  RosQoSSet(q,ROS_QOS_KEEP_LAST,depth,ROS_QOS_RELIABLE,ROS_QOS_TRANSIENT_LOCAL);
}

//Parameter events / parameter services.
U0 RosQoSParameters(CRosQoS *q)
{
  RosQoSSet(q,ROS_QOS_KEEP_LAST,1000,ROS_QOS_RELIABLE,ROS_QOS_VOLATILE);
}

//Can a publisher with qos p talk to a subscription with qos s?
//(Best effort can't feed reliable; volatile can't feed transient local.)
Bool RosQoSCompatible(CRosQoS *p,CRosQoS *s)
{
  if (p->reliability==ROS_QOS_BEST_EFFORT && s->reliability==ROS_QOS_RELIABLE)
    return FALSE;
  if (p->durability==ROS_QOS_VOLATILE && s->durability==ROS_QOS_TRANSIENT_LOCAL)
    return FALSE;
  return TRUE;
}

//---- classes

class CRosExec
{
  CRosVec *nodes;	//CRosNode *
  Bool stop;
};

class CRosNode
{
  U8 *name,*ns,*fqn;
  I64 id;
  CRosVec *pubs,*subs,*timers,*srvs,*clis;	//of CRosPub * etc.
  CRosVec *remaps;	//from,to pairs for this node
  CRosMap *params;	//name -> CRosParam *
  CRosVec *param_order;	//names in declaration order (not owned)
  CRosMap *overrides;	//name -> value text (from YAML / launch)
  Bool (*param_cb)(CRosNode *n,U8 *name,U8 *val,U8 *user);
  U8 *param_cb_user;
  U8 *rosout_pub;	//CRosPub *
  U8 *param_events_pub;	//CRosPub *
  CRosExec *exec;	//executor it's spinning in, if any
  U0 (*on_del)(CRosNode *n,U8 *user);	//called first thing in RosNodeDel
  U8 *on_del_user;
  I64 log_level;
  Bool dead;
};

class CRosTopic
{
  U8 *name;
  CRosType *type;
  CRosVec *pubs,*subs;
};

class CRosPub
{
  CRosNode *node;
  CRosTopic *topic;
  CRosQoS qos;
  CRosVec *history;	//CRosShared * kept for transient local
  I64 id,sent,incompatible;
};

class CRosSub
{
  CRosNode *node;
  CRosTopic *topic;
  CRosQoS qos;
  CRosRing *q;		//CRosShared *
  I64 lock;
  U0 (*cb)(U8 *msg,U8 *user);
  U8 *user;
  I64 id,received,dropped,expired,incompatible;
};

class CRosTimer
{
  CRosNode *node;
  I64 period_ns,next_ns;
  U0 (*cb)(U8 *user);
  U8 *user;
  Bool canceled;
  I64 calls;
};

class CRosService
{
  CRosNode *node;
  U8 *name;
  CRosSrvType *st;
  U0 (*cb)(U8 *req,U8 *resp,U8 *user);
  U8 *user;
  CRosVec *queue;	//CRosSrvReq *
  I64 lock,handled;
};

class CRosClient
{
  CRosNode *node;
  U8 *name;
  CRosSrvType *st;
  CRosVec *pending;	//CRosPending *
  I64 lock,id,next_seq;
};

class CRosSrvReq	//a request waiting at a service
{
  I64 cli_id,seq;
  U8 *req;
};

class CRosPending	//a request a client is waiting on
{
  I64 seq;
  U8 *req,*resp;
  U0 (*cb)(U8 *resp,I64 seq,U8 *user);
  U8 *user;
  Bool done;
};

class CRosGraph
{
  CRosMap *topics;	//name -> CRosTopic *
  CRosMap *services;	//name -> CRosService *
  CRosVec *nodes;	//CRosNode *
  CRosVec *clients;	//CRosClient *
  I64 lock,next_id;
};

CRosGraph *ros_graph=NULL;
Bool ros_ok=TRUE;	//RosShutdown() clears it; spinning stops

CRosGraph *RosGraph()
{
  if (!ros_graph) {
    ros_graph=CAlloc(sizeof(CRosGraph));
    ros_graph->topics=RosMapNew();
    ros_graph->services=RosMapNew();
    ros_graph->nodes=RosVecNew(8);
    ros_graph->clients=RosVecNew(8);
    ros_graph->next_id=1;
  }
  return ros_graph;
}

U0 RosGraphLock()
{
  RosLockGet(&RosGraph()->lock);
}

U0 RosGraphUnlock()
{
  RosLockPut(&ros_graph->lock);
}

U0 RosShutdown()
{
  ros_ok=FALSE;
}

//Remove the first p from v (by pointer). TRUE if found.
Bool RosVecRemovePtr(CRosVec *v,U8 *p)
{
  I64 i;
  U8 *q;
  for (i=0;i<v->len;i++) {
    q=RosVecGetI64(v,i);
    if (q==p) {
      RosVecRemove(v,i);
      return TRUE;
    }
  }
  return FALSE;
}

//---- shared messages
//A published message lives in one block with a reference count in
//front of it. Every subscriber gets the same pointer: no copies.

class CRosShared
{
  I64 refs;
  CRosType *type;
  I64 stamp_ns;		//steady time when published (for lifespan)
  I64 pub_id;
};

U8 *RosSharedMsg(CRosShared *s)
{
  U8 *p=s;
  return p+sizeof(CRosShared);
}

CRosShared *RosSharedOf(U8 *msg)
{
  return msg-sizeof(CRosShared);
}

CRosShared *RosSharedNew(CRosType *t)
{
  CRosShared *s=CAlloc(sizeof(CRosShared)+t->size);
  s->refs=1;
  s->type=t;
  RosMsgInit(t,RosSharedMsg(s));
  return s;
}

U0 RosSharedRetain(CRosShared *s)
{
  RosRefAdd(&s->refs,1);
}

U0 RosSharedRelease(CRosShared *s)
{
  if (!s)
    return;
  if (!RosRefAdd(&s->refs,-1)) {
    RosMsgFini(s->type,RosSharedMsg(s));
    Free(s);
  }
}

//Done with a message from RosSubTake / RosPubLoan you didn't publish.
U0 RosMsgRelease(U8 *msg)
{
  if (msg)
    RosSharedRelease(RosSharedOf(msg));
}

//---- graph lookups (call with the graph lock held)

CRosTopic *RosTopicGet(U8 *name,CRosType *t)
{
  CRosGraph *g=RosGraph();
  CRosTopic *tp=RosMapGet(g->topics,name);
  if (!tp) {
    tp=CAlloc(sizeof(CRosTopic));
    tp->name=StrNew(name);
    tp->type=t;
    tp->pubs=RosVecNew(8);
    tp->subs=RosVecNew(8);
    RosMapSet(g->topics,name,tp);
  }
  return tp;
}

//Drop a topic nobody uses any more.
U0 RosTopicGc(CRosTopic *tp)
{
  if (tp->pubs->len || tp->subs->len)
    return;
  RosMapRemove(ros_graph->topics,tp->name);
  RosVecDel(tp->pubs);
  RosVecDel(tp->subs);
  Free(tp->name);
  Free(tp);
}

CRosClient *RosClientById(I64 id)
{
  I64 i;
  CRosClient *c;
  for (i=0;i<ros_graph->clients->len;i++) {
    c=RosVecGetI64(ros_graph->clients,i);
    if (c->id==id)
      return c;
  }
  return NULL;
}
