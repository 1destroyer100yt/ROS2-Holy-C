//Cli: the ros2 command line, as HolyC functions. The TempleOS shell
//is a HolyC prompt, so these are typed straight in:
//
//  RosNodeList;                       ros2 node list
//  RosNodeInfo("/talker");            ros2 node info /talker
//  RosTopicList;  RosTopicListT;      ros2 topic list [-t]
//  RosTopicInfo("/chatter");          ros2 topic info /chatter
//  RosTopicEcho("/chatter");          ros2 topic echo /chatter (any key stops)
//  RosTopicPub("/cmd_vel","geometry_msgs/msg/Twist","{linear: {x: 0.5}}",1.0);
//  RosTopicHz("/scan");               ros2 topic hz /scan (5 s)
//  RosServiceList;  RosServiceCall("/add_two_ints","example_interfaces/srv/AddTwoInts","{a: 1, b: 2}");
//  RosParamList("/talker"); RosParamGet("/talker","rate"); RosParamSetCli("/talker","rate","5.0");
//  RosParamDump("/talker");
//  RosInterfaceList; RosInterfaceShow("geometry_msgs/msg/Twist"); RosInterfaceProto(...);
//
//Each has a ...Str version that returns the text instead (yours to Free).

CRosExec *ros_cli_assist=NULL;	//also spun by CLI waits (single-task tests)
I64 ros_cli_count=0;

//Any key pressed? (TempleOS; always FALSE on the host)
Bool RosCliKey()
{
//@host-skip-begin
  if (ScanKey)
    return TRUE;
//@host-skip-end
  return FALSE;
}

CRosNode *RosCliNodeNew()
{
  CRosNode *n;
  U8 *name;
  Bool b=ros_node_builtins;
  ros_cli_count++;
  name=MStrPrint("_ros2cli_%d",ros_cli_count);
  ros_node_builtins=FALSE;
  n=RosNodeNew(name,NULL);
  ros_node_builtins=b;
  Free(name);
  return n;
}

U0 RosCliNodeDel(CRosNode *n)
{
  CRosExec *e=n->exec;
  RosNodeDel(n);
  RosExecDel(e);
}

//Spin the CLI node (and the assist executor) for up to ns.
Bool RosCliSpinOnce(CRosNode *n,I64 ns)
{
  Bool did=FALSE;
  if (ros_cli_assist && RosExecSpinSome(ros_cli_assist))
    did=TRUE;
  if (RosExecSpinOnce(RosNodeExec(n),ns))
    did=TRUE;
  return did;
}

U8 *RosCliCall(CRosNode *n,CRosClient *c,U8 *req,I64 timeout_ns)
{
  CRosSyncCall sc;
  I64 seq,end=RosTimeSteadyNs()+timeout_ns;
  sc.t=c->st->resp;
  sc.resp=NULL;
  sc.done=FALSE;
  seq=RosClientCall(c,req,&RosSyncCallDone,&sc);
  if (seq<0)
    return NULL;
  while (!sc.done && RosTimeSteadyNs()<end && ros_ok)
    RosCliSpinOnce(n,ROS_NS_PER_MS);
  if (!sc.done)
    RosClientCancel(c,seq);
  return sc.resp;
}

U0 RosCliPrint(U8 *s)
{
  if (s) {
    Print("%s",s);
    Free(s);
  }
}

//---- graph lookups

CRosNode *RosNodeFind(U8 *fqn)
{
  I64 i;
  CRosNode *n,*hit=NULL;
  RosGraphLock();
  for (i=0;i<ros_graph->nodes->len;i++) {
    n=RosVecGetI64(ros_graph->nodes,i);
    if (!StrCmp(n->fqn,fqn))
      hit=n;
  }
  RosGraphUnlock();
  return hit;
}

//Remote (other machines') topics or services: name -> type name.
U8 *RosCliRemoteType(U8 *name,Bool services)
{
  CRosVec *names=RosVecNew(8),*types=RosVecNew(8);
  U8 *hit=NULL;
  I64 i;
  if (services && ros_hook_remote_services)
    (*ros_hook_remote_services)(names,types);
  if (!services && ros_hook_remote_topics)
    (*ros_hook_remote_topics)(names,types);
  for (i=0;i<names->len;i++)
    if (!hit && !StrCmp(RosVecGetI64(names,i),name))
      hit=StrNew(RosVecGetI64(types,i));
  RosStrVecDel(names);
  RosStrVecDel(types);
  return hit;
}

CRosType *RosTopicTypeOf(U8 *name)
{
  CRosTopic *tp;
  CRosType *t=NULL;
  U8 *rt;
  RosGraphLock();
  tp=RosMapGet(RosGraph()->topics,name);
  if (tp)
    t=tp->type;
  RosGraphUnlock();
  if (!t) {
    rt=RosCliRemoteType(name,FALSE);
    if (rt) {
      t=RosTypeFind(rt);
      Free(rt);
    }
  }
  return t;
}

//Sorted, de-duplicated (in place).
U0 RosStrVecSortUnique(CRosVec *v)
{
  I64 i=1;
  RosStrVecSort(v);
  while (i<v->len) {
    if (!StrCmp(RosVecGetI64(v,i),RosVecGetI64(v,i-1))) {
      Free(RosVecGetI64(v,i));
      RosVecRemove(v,i);
    } else
      i++;
  }
}

CRosVec *RosGraphNames(CRosMap *m)
{
  CRosVec *v=RosVecNew(8);
  I64 it=0;
  U8 *k,*val;
  RosGraphLock();
  while (RosMapNext(m,&it,&k,&val))
    RosVecPushI64(v,StrNew(k));
  RosGraphUnlock();
  RosStrVecSort(v);
  return v;
}

//---- nodes

U8 *RosNodeListStr()
{
  CRosStrBuf *sb=RosStrBufNew();
  CRosVec *v=RosVecNew(8);
  CRosNode *n;
  I64 i;
  RosGraphLock();
  for (i=0;i<RosGraph()->nodes->len;i++) {
    n=RosVecGetI64(ros_graph->nodes,i);
    if (n->name[0]!='_')	//hidden, like rcl
      RosVecPushI64(v,StrNew(n->fqn));
  }
  RosGraphUnlock();
  if (ros_hook_remote_nodes)
    (*ros_hook_remote_nodes)(v);
  RosStrVecSortUnique(v);
  for (i=0;i<v->len;i++) {
    RosStrBufAdd(sb,RosVecGetI64(v,i));
    RosStrBufAddC(sb,'\n');
  }
  RosStrVecDel(v);
  return RosStrBufTake(sb);
}

U0 RosNodeList()
{
  RosCliPrint(RosNodeListStr());
}

//Add "    name: type" lines for a list of endpoints (sorted).
U0 RosCliEndpoints(CRosStrBuf *sb,U8 *title,CRosVec *items,I64 kind)
{
  CRosVec *lines=RosVecNew(8);
  CRosPub *p;
  CRosSub *s;
  CRosService *sv;
  CRosClient *c;
  I64 i;
  for (i=0;i<items->len;i++) {
    if (kind==0) {
      p=RosVecGetI64(items,i);
      RosVecPushI64(lines,MStrPrint("    %s: %s\n",p->topic->name,p->topic->type->name));
    } else if (kind==1) {
      s=RosVecGetI64(items,i);
      RosVecPushI64(lines,MStrPrint("    %s: %s\n",s->topic->name,s->topic->type->name));
    } else if (kind==2) {
      sv=RosVecGetI64(items,i);
      RosVecPushI64(lines,MStrPrint("    %s: %s\n",sv->name,sv->st->name));
    } else {
      c=RosVecGetI64(items,i);
      RosVecPushI64(lines,MStrPrint("    %s: %s\n",c->name,c->st->name));
    }
  }
  RosStrVecSort(lines);
  RosStrBufAdd(sb,title);
  for (i=0;i<lines->len;i++)
    if (i==0 || StrCmp(RosVecGetI64(lines,i),RosVecGetI64(lines,i-1)))
      RosStrBufAdd(sb,RosVecGetI64(lines,i));
  RosStrVecDel(lines);
}

U8 *RosNodeInfoStr(U8 *fqn)
{
  CRosNode *n=RosNodeFind(fqn);
  CRosStrBuf *sb;
  if (!n)
    return MStrPrint("Unable to find node '%s'\n",fqn);
  sb=RosStrBufNew();
  RosStrBufAdd(sb,n->fqn);
  RosStrBufAddC(sb,'\n');
  RosGraphLock();
  RosCliEndpoints(sb,"  Subscribers:\n",n->subs,1);
  RosCliEndpoints(sb,"  Publishers:\n",n->pubs,0);
  RosCliEndpoints(sb,"  Service Servers:\n",n->srvs,2);
  RosCliEndpoints(sb,"  Service Clients:\n",n->clis,3);
  RosGraphUnlock();
  return RosStrBufTake(sb);
}

U0 RosNodeInfo(U8 *fqn)
{
  RosCliPrint(RosNodeInfoStr(fqn));
}

//---- topics

//Local names plus other machines' (sorted, unique).
CRosVec *RosCliAllNames(CRosMap *local,Bool services)
{
  CRosVec *v=RosGraphNames(local),*names=RosVecNew(8),*types=RosVecNew(8);
  I64 i;
  if (services && ros_hook_remote_services)
    (*ros_hook_remote_services)(names,types);
  if (!services && ros_hook_remote_topics)
    (*ros_hook_remote_topics)(names,types);
  for (i=0;i<names->len;i++)
    RosVecPushI64(v,StrNew(RosVecGetI64(names,i)));
  RosStrVecDel(names);
  RosStrVecDel(types);
  RosStrVecSortUnique(v);
  return v;
}

U8 *RosTopicListStr(Bool types)
{
  CRosVec *v=RosCliAllNames(RosGraph()->topics,FALSE);
  CRosStrBuf *sb=RosStrBufNew();
  CRosType *t;
  I64 i;
  for (i=0;i<v->len;i++) {
    RosStrBufAdd(sb,RosVecGetI64(v,i));
    if (types) {
      t=RosTopicTypeOf(RosVecGetI64(v,i));
      RosStrBufAdd(sb," [");
      if (t)
        RosStrBufAdd(sb,t->name);
      RosStrBufAddC(sb,']');
    }
    RosStrBufAddC(sb,'\n');
  }
  RosStrVecDel(v);
  return RosStrBufTake(sb);
}

U0 RosTopicList()
{
  RosCliPrint(RosTopicListStr(FALSE));
}

U0 RosTopicListT()
{
  RosCliPrint(RosTopicListStr(TRUE));
}

U8 *RosQoSStr(CRosQoS *q)
{
  U8 *h="KEEP_LAST",*r="RELIABLE",*d="VOLATILE";
  if (q->history==ROS_QOS_KEEP_ALL)
    h="KEEP_ALL";
  if (q->reliability==ROS_QOS_BEST_EFFORT)
    r="BEST_EFFORT";
  if (q->durability==ROS_QOS_TRANSIENT_LOCAL)
    d="TRANSIENT_LOCAL";
  return MStrPrint("    Reliability: %s\n    History (Depth): %s (%d)\n    Durability: %s\n",
	r,h,q->depth,d);
}

U8 *RosTopicInfoStr(U8 *name,Bool verbose)
{
  CRosTopic *tp;
  CRosStrBuf *sb;
  CRosPub *p;
  CRosSub *s;
  U8 *q;
  I64 i;
  RosGraphLock();
  tp=RosMapGet(RosGraph()->topics,name);
  if (!tp) {
    RosGraphUnlock();
    return MStrPrint("Unknown topic '%s'\n",name);
  }
  sb=RosStrBufNew();
  RosStrBufAdd(sb,"Type: ");
  RosStrBufAdd(sb,tp->type->name);
  RosStrBufAdd(sb,"\nPublisher count: ");
  RosStrBufAddI64(sb,tp->pubs->len);
  RosStrBufAdd(sb,"\nSubscription count: ");
  RosStrBufAddI64(sb,tp->subs->len);
  RosStrBufAddC(sb,'\n');
  if (verbose) {
    for (i=0;i<tp->pubs->len;i++) {
      p=RosVecGetI64(tp->pubs,i);
      RosStrBufAdd(sb,"\nNode name: ");
      RosStrBufAdd(sb,p->node->fqn);
      RosStrBufAdd(sb,"\nEndpoint type: PUBLISHER\nQoS profile:\n");
      q=RosQoSStr(&p->qos);
      RosStrBufAdd(sb,q);
      Free(q);
    }
    for (i=0;i<tp->subs->len;i++) {
      s=RosVecGetI64(tp->subs,i);
      RosStrBufAdd(sb,"\nNode name: ");
      RosStrBufAdd(sb,s->node->fqn);
      RosStrBufAdd(sb,"\nEndpoint type: SUBSCRIPTION\nQoS profile:\n");
      q=RosQoSStr(&s->qos);
      RosStrBufAdd(sb,q);
      Free(q);
    }
  }
  RosGraphUnlock();
  return RosStrBufTake(sb);
}

U0 RosTopicInfo(U8 *name)
{
  RosCliPrint(RosTopicInfoStr(name,FALSE));
}

U0 RosTopicInfoV(U8 *name)
{
  RosCliPrint(RosTopicInfoStr(name,TRUE));
}

//Echo up to count messages (0 = until a key / shutdown / timeout).
//field (or NULL) prints only that part, like --field. Returns how
//many were printed.
I64 RosTopicEchoN(U8 *name,U8 *field,I64 count,I64 timeout_ns)
{
  CRosType *t=RosTopicTypeOf(name);
  CRosNode *n;
  CRosSub *s;
  CRosQoS q;
  U8 *m,*y;
  I64 got=0,end;
  if (!t) {
    Print("Topic '%s' doesn't exist (or has no type yet)\n",name);
    return 0;
  }
  n=RosCliNodeNew();
  RosQoSSensorData(&q);	//best effort reads from every publisher
  q.depth=100;
  s=RosSubNew(n,name,t,&q,NULL,NULL);
  end=RosTimeSteadyNs()+timeout_ns;
  while (ros_ok && (!count || got<count) && RosTimeSteadyNs()<end && !RosCliKey()) {
    m=RosSubTake(s);
    if (!m) {
      RosCliSpinOnce(n,ROS_NS_PER_MS);
      continue;
    }
    if (field) {
      y=RosMsgGet(t,m,field);
      if (y) {
        Print("%s\n---\n",y);
        Free(y);
      } else
        Print("bad field: %s\n",ros_msg_error);
    } else
      RosMsgPrint(t,m);
    RosMsgRelease(m);
    got++;
  }
  RosCliNodeDel(n);
  return got;
}

U0 RosTopicEcho(U8 *name)
{
  RosTopicEchoN(name,NULL,0,0x7FFFFFFFFFFFFFFF);
}

//Fill message m of type t from "{a: 1, b: {c: 2}}" or "a=1 b.c=2".
Bool RosMsgFill(CRosType *t,U8 *m,U8 *values)
{
  CRosVec *items=RosYamlFlowFlatten(values);
  CRosYamlItem *it;
  I64 i;
  Bool ok=TRUE;
  if (!items) {
    RosMsgErr("can't read values",values);
    return FALSE;
  }
  for (i=0;ok && i<items->len;i++) {
    it=RosVecGetI64(items,i);
    ok=RosMsgSet(t,m,it->path,it->val);
  }
  RosYamlFree(items);
  return ok;
}

//Publish values on a topic times times (0 = until a key) at rate_hz.
//Returns how many were published.
I64 RosTopicPubN(U8 *name,U8 *type_name,U8 *values,F64 rate_hz,I64 times)
{
  CRosType *t=RosTypeFind(type_name);
  CRosNode *n;
  CRosPub *p;
  U8 *m;
  I64 sent=0,period,next;
  if (!t) {
    Print("Unknown message type '%s'\n",type_name);
    return 0;
  }
  m=RosMsgNew(t);
  if (!RosMsgFill(t,m,values)) {
    Print("Bad values: %s\n",ros_msg_error);
    RosMsgDel(t,m);
    return 0;
  }
  n=RosCliNodeNew();
  p=RosPubNew(n,name,t,NULL);
  if (!p) {
    Print("%s\n",ros_graph_error);
    RosMsgDel(t,m);
    RosCliNodeDel(n);
    return 0;
  }
  if (rate_hz<=0.0)
    rate_hz=1.0;
  period=ToI64(1000000000.0/rate_hz);
  next=RosTimeSteadyNs();
  while (ros_ok && (!times || sent<times) && !RosCliKey()) {
    if (RosTimeSteadyNs()>=next) {
      RosPublish(p,m);
      sent++;
      Print("publishing #%d\n",sent);
      next+=period;
    }
    RosCliSpinOnce(n,ROS_NS_PER_MS);
  }
  RosMsgDel(t,m);
  RosCliNodeDel(n);
  return sent;
}

U0 RosTopicPub(U8 *name,U8 *type_name,U8 *values,F64 rate_hz)
{
  RosTopicPubN(name,type_name,values,rate_hz,0);
}

U0 RosTopicPubOnce(U8 *name,U8 *type_name,U8 *values)
{
  RosTopicPubN(name,type_name,values,1.0,1);
}

//Round to 3 decimals (for display).
F64 Round1k(F64 x)
{
  return Floor(x*1000.0+0.5)/1000.0;
}

class CRosHz
{
  I64 last,count;
  F64 sum,sum2,min,max;
};

U0 RosHzOnMsg(U8 *m,CRosHz *h)
{
  I64 now=RosTimeSteadyNs();
  F64 dt;
  if (h->last) {
    dt=RosTimeToSec(now-h->last);
    h->sum+=dt;
    h->sum2+=dt*dt;
    if (!h->count || dt<h->min)
      h->min=dt;
    if (dt>h->max)
      h->max=dt;
    h->count++;
  }
  h->last=now;
}

//Measure the publish rate for a while. Returns the text report.
U8 *RosTopicHzStr(U8 *name,I64 window_ns)
{
  CRosType *t=RosTopicTypeOf(name);
  CRosNode *n;
  CRosHz h;
  CRosQoS q;
  CRosStrBuf *sb;
  F64 mean,var;
  I64 end;
  if (!t)
    return MStrPrint("Topic '%s' doesn't exist\n",name);
  MemSet(&h,0,sizeof(CRosHz));
  n=RosCliNodeNew();
  RosQoSSensorData(&q);
  q.depth=1000;
  RosSubNew(n,name,t,&q,&RosHzOnMsg,&h);
  end=RosTimeSteadyNs()+window_ns;
  while (ros_ok && RosTimeSteadyNs()<end && !RosCliKey())
    RosCliSpinOnce(n,ROS_NS_PER_MS);
  RosCliNodeDel(n);
  if (!h.count)
    return StrNew("no new messages\n");
  mean=h.sum/h.count;
  var=h.sum2/h.count-mean*mean;
  if (var<0.0)
    var=0.0;
  sb=RosStrBufNew();
  RosStrBufAdd(sb,"average rate: ");
  RosStrBufAddF64(sb,Round1k(1.0/mean));
  RosStrBufAdd(sb,"\n\tmin: ");
  RosStrBufAddF64(sb,Round1k(h.min));
  RosStrBufAdd(sb,"s max: ");
  RosStrBufAddF64(sb,Round1k(h.max));
  RosStrBufAdd(sb,"s std dev: ");
  RosStrBufAddF64(sb,Round1k(Sqrt(var)));
  RosStrBufAdd(sb,"s window: ");
  RosStrBufAddI64(sb,h.count+1);
  RosStrBufAddC(sb,'\n');
  return RosStrBufTake(sb);
}

U0 RosTopicHz(U8 *name)
{
  RosCliPrint(RosTopicHzStr(name,5*ROS_NS_PER_SEC));
}

//---- services

U8 *RosServiceListStr(Bool types)
{
  CRosVec *v=RosCliAllNames(RosGraph()->services,TRUE);
  CRosStrBuf *sb=RosStrBufNew();
  CRosService *s;
  U8 *rt;
  I64 i;
  for (i=0;i<v->len;i++) {
    RosStrBufAdd(sb,RosVecGetI64(v,i));
    if (types) {
      RosGraphLock();
      s=RosMapGet(ros_graph->services,RosVecGetI64(v,i));
      if (s) {
        RosStrBufAdd(sb," [");
        RosStrBufAdd(sb,s->st->name);
        RosStrBufAddC(sb,']');
      }
      RosGraphUnlock();
      if (!s) {
        rt=RosCliRemoteType(RosVecGetI64(v,i),TRUE);
        if (rt) {
          RosStrBufAdd(sb," [");
          RosStrBufAdd(sb,rt);
          RosStrBufAddC(sb,']');
          Free(rt);
        }
      }
    }
    RosStrBufAddC(sb,'\n');
  }
  RosStrVecDel(v);
  return RosStrBufTake(sb);
}

U0 RosServiceList()
{
  RosCliPrint(RosServiceListStr(FALSE));
}

U0 RosServiceListT()
{
  RosCliPrint(RosServiceListStr(TRUE));
}

U8 *RosServiceCallStr(U8 *name,U8 *type_name,U8 *values)
{
  CRosSrvType *st=RosSrvFind(type_name);
  CRosNode *n;
  CRosClient *c;
  CRosStrBuf *sb;
  U8 *req,*resp,*y;
  I64 end;
  if (!st)
    return MStrPrint("Unknown service type '%s'\n",type_name);
  req=RosMsgNew(st->req);
  if (!RosMsgFill(st->req,req,values)) {
    RosMsgDel(st->req,req);
    return MStrPrint("Bad values: %s\n",ros_msg_error);
  }
  n=RosCliNodeNew();
  c=RosClientNew(n,name,st);
  sb=RosStrBufNew();
  end=RosTimeSteadyNs()+2*ROS_NS_PER_SEC;	//give network discovery a moment
  while (c && !RosClientReady(c) && c->dds && RosTimeSteadyNs()<end && ros_ok)
    RosCliSpinOnce(n,10*ROS_NS_PER_MS);
  if (!c || !RosClientReady(c))
    RosStrBufAdd(sb,"waiting for service to become available...\nservice not available\n");
  else {
    RosStrBufAdd(sb,"requester: making request:\n");
    y=RosMsgToYaml(st->req,req);
    RosStrBufAdd(sb,y);
    Free(y);
    resp=RosCliCall(n,c,req,5*ROS_NS_PER_SEC);
    if (!resp)
      RosStrBufAdd(sb,"\nno response (timed out)\n");
    else {
      RosStrBufAdd(sb,"\nresponse:\n");
      y=RosMsgToYaml(st->resp,resp);
      RosStrBufAdd(sb,y);
      Free(y);
      RosMsgDel(st->resp,resp);
    }
  }
  RosMsgDel(st->req,req);
  RosCliNodeDel(n);
  return RosStrBufTake(sb);
}

U0 RosServiceCall(U8 *name,U8 *type_name,U8 *values)
{
  RosCliPrint(RosServiceCallStr(name,type_name,values));
}

//---- parameters (through the parameter services, like ros2 param)

//Call node_fqn/<srv>. Returns the response or NULL (message in *err).
U8 *RosParamCall(CRosNode *n,U8 *node_fqn,U8 *srv,CRosSrvType *st,U8 *req)
{
  U8 *name=MStrPrint("%s/%s",node_fqn,srv),*resp=NULL;
  CRosClient *c=RosClientNew(n,name,st);
  Free(name);
  if (c && RosClientReady(c))
    resp=RosCliCall(n,c,req,5*ROS_NS_PER_SEC);
  return resp;
}

U8 *RosParamListStr(U8 *node_fqn)
{
  CRosNode *n=RosCliNodeNew();
  CRosType *t=ros_t_rcl_interfaces_ListParameters_Request;
  CSrv_rcl_interfaces_ListParameters_Response *resp;
  U8 *req=RosMsgNew(t),**names;
  CRosVec *v;
  CRosStrBuf *sb=RosStrBufNew();
  I64 i;
  resp=RosParamCall(n,node_fqn,"list_parameters",ros_srv_rcl_interfaces_ListParameters,req);
  if (!resp)
    RosStrBufAdd(sb,"Node not found or not answering\n");
  else {
    v=RosVecNew(8);
    names=resp->result.names.data;
    for (i=0;i<resp->result.names.len;i++)
      RosVecPushI64(v,StrNew(names[i]));
    RosStrVecSort(v);
    for (i=0;i<v->len;i++) {
      RosStrBufAdd(sb,"  ");
      RosStrBufAdd(sb,RosVecGetI64(v,i));
      RosStrBufAddC(sb,'\n');
    }
    RosStrVecDel(v);
    RosMsgDel(ros_t_rcl_interfaces_ListParameters_Response,resp);
  }
  RosMsgDel(t,req);
  RosCliNodeDel(n);
  return RosStrBufTake(sb);
}

U0 RosParamList(U8 *node_fqn)
{
  RosCliPrint(RosParamListStr(node_fqn));
}

//"Double value is: 1.5" etc., like ros2 param get.
U8 *RosPvDescribe(CMsg_rcl_interfaces_ParameterValue *pv)
{
  U8 *v=RosPvToText(pv),*s,*what;
  switch (pv->type) {
    case ROS_PT_BOOL: what="Boolean value is"; break;
    case ROS_PT_INTEGER: what="Integer value is"; break;
    case ROS_PT_DOUBLE: what="Double value is"; break;
    case ROS_PT_STRING:
      Free(v);
      return MStrPrint("String value is: %s\n",pv->string_value);
    case ROS_PT_BYTE_ARRAY: what="Byte values are"; break;
    case ROS_PT_BOOL_ARRAY: what="Boolean values are"; break;
    case ROS_PT_INTEGER_ARRAY: what="Integer values are"; break;
    case ROS_PT_DOUBLE_ARRAY: what="Double values are"; break;
    case ROS_PT_STRING_ARRAY: what="String values are"; break;
    default:
      Free(v);
      return StrNew("Parameter not set.\n");
  }
  s=MStrPrint("%s: %s\n",what,v);
  Free(v);
  return s;
}

U8 *RosParamGetStr(U8 *node_fqn,U8 *name)
{
  CRosNode *n=RosCliNodeNew();
  CRosType *t=ros_t_rcl_interfaces_GetParameters_Request;
  CSrv_rcl_interfaces_GetParameters_Response *resp;
  U8 *req=RosMsgNew(t),*s;
  CMsg_rcl_interfaces_ParameterValue *vals;
  RosElemSetS(RosSeqAppend(t,req,"names"),StrNew(name));
  resp=RosParamCall(n,node_fqn,"get_parameters",ros_srv_rcl_interfaces_GetParameters,req);
  if (!resp)
    s=StrNew("Node not found or not answering\n");
  else {
    vals=resp->values.data;
    if (resp->values.len)
      s=RosPvDescribe(vals);
    else
      s=StrNew("Parameter not set.\n");
    RosMsgDel(ros_t_rcl_interfaces_GetParameters_Response,resp);
  }
  RosMsgDel(t,req);
  RosCliNodeDel(n);
  return s;
}

U0 RosParamGet(U8 *node_fqn,U8 *name)
{
  RosCliPrint(RosParamGetStr(node_fqn,name));
}

U8 *RosParamSetStr(U8 *node_fqn,U8 *name,U8 *value)
{
  CRosNode *n=RosCliNodeNew();
  CRosType *t=ros_t_rcl_interfaces_SetParameters_Request;
  CSrv_rcl_interfaces_SetParameters_Response *resp;
  CMsg_rcl_interfaces_Parameter *p;
  CMsg_rcl_interfaces_ParameterValue *pv=RosPvFromText(value);
  CMsg_rcl_interfaces_SetParametersResult *r;
  U8 *req,*s;
  if (!pv) {
    RosCliNodeDel(n);
    return MStrPrint("Bad value: %s\n",ros_param_error);
  }
  req=RosMsgNew(t);
  p=RosSeqAppend(t,req,"parameters");
  p->name=StrNew(name);
  RosMsgCopy(ros_t_rcl_interfaces_ParameterValue,&p->value,pv);
  RosPvDel(pv);
  resp=RosParamCall(n,node_fqn,"set_parameters",ros_srv_rcl_interfaces_SetParameters,req);
  if (!resp)
    s=StrNew("Node not found or not answering\n");
  else {
    r=resp->results.data;
    if (resp->results.len && r->successful)
      s=StrNew("Set parameter successful\n");
    else if (resp->results.len)
      s=MStrPrint("Setting parameter failed: %s\n",r->reason);
    else
      s=StrNew("Setting parameter failed\n");
    RosMsgDel(ros_t_rcl_interfaces_SetParameters_Response,resp);
  }
  RosMsgDel(t,req);
  RosCliNodeDel(n);
  return s;
}

U0 RosParamSetCli(U8 *node_fqn,U8 *name,U8 *value)
{
  RosCliPrint(RosParamSetStr(node_fqn,name,value));
}

//All parameters of a node as a params YAML file.
U8 *RosParamDumpStr(U8 *node_fqn)
{
  CRosNode *n=RosCliNodeNew();
  CRosType *lt=ros_t_rcl_interfaces_ListParameters_Request;
  CRosType *gt=ros_t_rcl_interfaces_GetParameters_Request;
  CSrv_rcl_interfaces_ListParameters_Response *lresp;
  CSrv_rcl_interfaces_GetParameters_Response *gresp=NULL;
  CMsg_rcl_interfaces_ParameterValue *vals;
  U8 *lreq=RosMsgNew(lt),*greq=RosMsgNew(gt),**names,*v;
  CRosVec *sorted=RosVecNew(8);
  CRosStrBuf *sb=RosStrBufNew();
  I64 i;
  lresp=RosParamCall(n,node_fqn,"list_parameters",ros_srv_rcl_interfaces_ListParameters,lreq);
  if (lresp) {
    names=lresp->result.names.data;
    for (i=0;i<lresp->result.names.len;i++)
      RosVecPushI64(sorted,StrNew(names[i]));
    RosStrVecSort(sorted);
    for (i=0;i<sorted->len;i++)
      RosElemSetS(RosSeqAppend(gt,greq,"names"),StrNew(RosVecGetI64(sorted,i)));
    gresp=RosParamCall(n,node_fqn,"get_parameters",ros_srv_rcl_interfaces_GetParameters,greq);
  }
  if (!gresp)
    RosStrBufAdd(sb,"# node not found or not answering\n");
  else {
    RosStrBufAdd(sb,node_fqn);
    RosStrBufAdd(sb,":\n  ros__parameters:\n");
    vals=gresp->values.data;
    for (i=0;i<sorted->len && i<gresp->values.len;i++) {
      RosStrBufAdd(sb,"    ");
      RosStrBufAdd(sb,RosVecGetI64(sorted,i));
      RosStrBufAdd(sb,": ");
      v=RosPvToText(&vals[i]);
      RosStrBufAdd(sb,v);
      Free(v);
      RosStrBufAddC(sb,'\n');
    }
    RosMsgDel(ros_t_rcl_interfaces_GetParameters_Response,gresp);
  }
  if (lresp)
    RosMsgDel(ros_t_rcl_interfaces_ListParameters_Response,lresp);
  RosMsgDel(lt,lreq);
  RosMsgDel(gt,greq);
  RosStrVecDel(sorted);
  RosCliNodeDel(n);
  return RosStrBufTake(sb);
}

U0 RosParamDump(U8 *node_fqn)
{
  RosCliPrint(RosParamDumpStr(node_fqn));
}

//---- interfaces

U8 *RosInterfaceListStr()
{
  CRosVec *msgs=RosVecNew(8),*srvs=RosVecNew(8);
  CRosStrBuf *sb=RosStrBufNew();
  I64 i,it=0;
  U8 *k,*v;
  if (ros_types)
    while (RosMapNext(ros_types,&it,&k,&v))
      if (RosStrFind(k,"/srv/")<0)
        RosVecPushI64(msgs,StrNew(k));
  it=0;
  if (ros_srv_types)
    while (RosMapNext(ros_srv_types,&it,&k,&v))
      RosVecPushI64(srvs,StrNew(k));
  RosStrVecSort(msgs);
  RosStrVecSort(srvs);
  RosStrBufAdd(sb,"Messages:\n");
  for (i=0;i<msgs->len;i++) {
    RosStrBufAdd(sb,"    ");
    RosStrBufAdd(sb,RosVecGetI64(msgs,i));
    RosStrBufAddC(sb,'\n');
  }
  RosStrBufAdd(sb,"Services:\n");
  for (i=0;i<srvs->len;i++) {
    RosStrBufAdd(sb,"    ");
    RosStrBufAdd(sb,RosVecGetI64(srvs,i));
    RosStrBufAddC(sb,'\n');
  }
  RosStrVecDel(msgs);
  RosStrVecDel(srvs);
  return RosStrBufTake(sb);
}

U0 RosInterfaceList()
{
  RosCliPrint(RosInterfaceListStr());
}

U8 *RosInterfaceShowStr(U8 *name)
{
  CRosType *t=RosTypeFind(name);
  CRosSrvType *st=RosSrvFind(name);
  if (st)
    return MStrPrint("%s---\n%s",st->req->def,st->resp->def);
  if (t)
    return StrNew(t->def);
  return MStrPrint("Unknown interface '%s'\n",name);
}

U0 RosInterfaceShow(U8 *name)
{
  RosCliPrint(RosInterfaceShowStr(name));
}

//A message with default values as YAML (what to type for RosTopicPub).
U8 *RosInterfaceProtoStr(U8 *name)
{
  CRosType *t=RosTypeFind(name);
  U8 *m,*y;
  if (!t)
    return MStrPrint("Unknown message type '%s'\n",name);
  m=RosMsgNew(t);
  y=RosMsgToYaml(t,m);
  RosMsgDel(t,m);
  return y;
}

U0 RosInterfaceProto(U8 *name)
{
  RosCliPrint(RosInterfaceProtoStr(name));
}
