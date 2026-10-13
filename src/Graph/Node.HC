//Node: a named participant in the ROS graph.
//
//  CRosNode *n=RosNodeNew("talker","/");
//  RosNodeRemap(n,"chatter","/news");	//like -r chatter:=/news
//  ...
//  RosNodeDel(n);
//
//Each node gets /rosout logging, parameter services and
///parameter_events, like in ROS 2. Set ros_node_builtins=FALSE before
//making a node to skip those.

Bool ros_node_builtins=TRUE;

extern U0 RosParamNodeInit(CRosNode *n);
extern U0 RosParamNodeFini(CRosNode *n);

//Expand + remap a topic/service name for this node. New string or NULL.
U8 *RosNodeResolve(CRosNode *n,U8 *name)
{
  U8 *fqn=RosNameExpand(name,n->name,n->ns),*a;
  if (!fqn) {
    StrCpy(ros_graph_error,ros_name_error);
    return NULL;
  }
  a=RosNameRemapWith(n->remaps,fqn,n->name,n->ns);
  Free(fqn);
  fqn=RosNameRemapWith(ros_global_remaps,a,n->name,n->ns);
  Free(a);
  return fqn;
}

//Add a remap rule for names this node creates from now on.
U0 RosNodeRemap(CRosNode *n,U8 *from,U8 *to)
{
  RosVecPushI64(n->remaps,StrNew(from));
  RosVecPushI64(n->remaps,StrNew(to));
}

//ns may be NULL or "" for the root namespace. NULL if the name is bad.
CRosNode *RosNodeNew(U8 *name,U8 *ns)
{
  CRosNode *n;
  CRosQoS q;
  U8 *nsx;
  CRosStrBuf *sb;
  if (!RosNodeNameOk(name)) {
    StrCpy(ros_graph_error,ros_name_error);
    return NULL;
  }
  if (!ns || !ns[0])
    nsx=StrNew("/");
  else if (ns[0]!='/') {
    sb=RosStrBufNew();
    RosStrBufAddC(sb,'/');
    RosStrBufAdd(sb,ns);
    nsx=RosStrBufTake(sb);
  } else
    nsx=StrNew(ns);
  if (!RosFqnOk(nsx)) {
    StrCpy(ros_graph_error,ros_name_error);
    Free(nsx);
    return NULL;
  }
  n=CAlloc(sizeof(CRosNode));
  n->name=StrNew(name);
  n->ns=nsx;
  n->fqn=RosNameJoin(nsx,name);
  n->pubs=RosVecNew(8);
  n->subs=RosVecNew(8);
  n->timers=RosVecNew(8);
  n->srvs=RosVecNew(8);
  n->clis=RosVecNew(8);
  n->remaps=RosVecNew(8);
  n->params=RosMapNew();
  n->param_order=RosVecNew(8);
  n->overrides=RosMapNew();
  n->log_level=ros_log_level;
  RosGraphLock();
  n->id=ros_graph->next_id++;
  RosVecPushI64(ros_graph->nodes,n);
  RosGraphUnlock();
  if (ros_node_builtins) {
    RosQoSTransientLocal(&q,1000);
    q.lifespan_ns=10*ROS_NS_PER_SEC;
    n->rosout_pub=RosPubNew(n,"/rosout",ros_t_rcl_interfaces_Log,&q);
  }
  RosParamNodeInit(n);
  return n;
}

//Log through this node: prints and publishes on /rosout.
U0 RosNodeLog(CRosNode *n,I64 level,U8 *text)
{
  CMsg_rcl_interfaces_Log *m;
  I64 now;
  level=RosLogClamp(level);
  if (level<n->log_level)
    return;
  RosLogEmit(level,n->name,text);
  if (!n->rosout_pub)
    return;
  m=RosPubLoan(n->rosout_pub);
  now=RosTimeSystemNs();
  m->stamp.sec=RosTimeSec(now);
  m->stamp.nanosec=RosTimeNsec(now);
  m->level=(level+1)*10;	//DEBUG=10 INFO=20 WARN=30 ERROR=40 FATAL=50
  m->name=StrNew(n->name);
  m->msg=StrNew(text);
  m->file=StrNew("");
  m->function=StrNew("");
  RosPublishLoaned(n->rosout_pub,m);
}

//@host-skip-begin
U0 RosNodeLogJoin(CRosNode *n,I64 level,U8 *fmt,I64 argc,I64 *argv)
{
  U8 *buf=StrPrintJoin(NULL,fmt,argc,argv);
  RosNodeLog(n,level,buf);
  Free(buf);
}
U0 RosNodeDebug(CRosNode *n,U8 *fmt,...) {RosNodeLogJoin(n,ROS_LOG_DEBUG,fmt,argc,argv);}
U0 RosNodeInfo(CRosNode *n,U8 *fmt,...)  {RosNodeLogJoin(n,ROS_LOG_INFO,fmt,argc,argv);}
U0 RosNodeWarn(CRosNode *n,U8 *fmt,...)  {RosNodeLogJoin(n,ROS_LOG_WARN,fmt,argc,argv);}
U0 RosNodeError(CRosNode *n,U8 *fmt,...) {RosNodeLogJoin(n,ROS_LOG_ERROR,fmt,argc,argv);}
//@host-skip-end

//Run cb(n,user) when the node is deleted, before its publishers etc.
//go away: the place to free state you hung on the node. One per node.
U0 RosNodeOnDel(CRosNode *n,U0 (*cb)(CRosNode *n,U8 *user),U8 *user)
{
  n->on_del=cb;
  n->on_del_user=user;
}

//Destroy a node and everything it made.
U0 RosNodeDel(CRosNode *n)
{
  if (!n)
    return;
  if (n->on_del)
    (*n->on_del)(n,n->on_del_user);
  RosParamNodeFini(n);
  while (n->pubs->len)
    RosPubDel(RosVecGetI64(n->pubs,n->pubs->len-1));
  while (n->subs->len)
    RosSubDel(RosVecGetI64(n->subs,n->subs->len-1));
  while (n->timers->len)
    RosTimerDel(RosVecGetI64(n->timers,n->timers->len-1));
  while (n->srvs->len)
    RosServiceDel(RosVecGetI64(n->srvs,n->srvs->len-1));
  while (n->clis->len)
    RosClientDel(RosVecGetI64(n->clis,n->clis->len-1));
  RosGraphLock();
  RosVecRemovePtr(ros_graph->nodes,n);
  RosGraphUnlock();
  if (n->exec)
    RosVecRemovePtr(n->exec->nodes,n);
  RosVecDel(n->pubs);
  RosVecDel(n->subs);
  RosVecDel(n->timers);
  RosVecDel(n->srvs);
  RosVecDel(n->clis);
  RosStrVecDel(n->remaps);
  RosVecDel(n->param_order);
  RosMapDel(n->params);
  RosMapDel(n->overrides);
  Free(n->name);
  Free(n->ns);
  Free(n->fqn);
  Free(n);
}
