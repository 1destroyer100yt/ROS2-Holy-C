//Srv: services and clients.
//
//  U0 Add(CSrv_example_interfaces_AddTwoInts_Request *req,
//	CSrv_example_interfaces_AddTwoInts_Response *resp,U8 *user)
//  { resp->sum=req->a+req->b; }
//  RosServiceNew(node,"add_two_ints",ros_srv_example_interfaces_AddTwoInts,&Add,NULL);
//
//  CRosClient *c=RosClientNew(node,"add_two_ints",ros_srv_example_interfaces_AddTwoInts);
//  RosClientCall(c,req,&OnSum,NULL);	//OnSum(resp,seq,user) runs later
//
//The service callback runs in the server node's executor, the
//response callback in the client node's executor.

CRosService *RosServiceNew(CRosNode *n,U8 *name,CRosSrvType *st,
	U0 (*cb)(U8 *req,U8 *resp,U8 *user),U8 *user)
{
  U8 *fqn=RosNodeResolve(n,name);
  CRosService *s;
  if (!fqn)
    return NULL;
  if (!st) {
    RosGraphErr("unknown service type",name);
    Free(fqn);
    return NULL;
  }
  RosGraphLock();
  if (RosMapHas(ros_graph->services,fqn)) {
    RosGraphUnlock();
    RosGraphErr("service name already in use",fqn);
    Free(fqn);
    return NULL;
  }
  s=CAlloc(sizeof(CRosService));
  s->node=n;
  s->name=fqn;
  s->st=st;
  s->cb=cb;
  s->user=user;
  s->queue=RosVecNew(8);
  RosMapSet(ros_graph->services,fqn,s);
  RosVecPushI64(n->srvs,s);
  RosGraphUnlock();
  if (ros_hook_srv_new)
    (*ros_hook_srv_new)(s);
  return s;
}

U0 RosServiceDel(CRosService *s)
{
  I64 i;
  CRosSrvReq *r;
  if (!s)
    return;
  if (ros_hook_srv_del && s->dds)
    (*ros_hook_srv_del)(s);
  RosGraphLock();
  RosMapRemove(ros_graph->services,s->name);
  RosVecRemovePtr(s->node->srvs,s);
  RosGraphUnlock();
  for (i=0;i<s->queue->len;i++) {
    r=RosVecGetI64(s->queue,i);
    RosMsgDel(s->st->req,r->req);
    Free(r->remote);
    Free(r);
  }
  RosVecDel(s->queue);
  Free(s->name);
  Free(s);
}

CRosClient *RosClientNew(CRosNode *n,U8 *name,CRosSrvType *st)
{
  U8 *fqn=RosNodeResolve(n,name);
  CRosClient *c;
  if (!fqn)
    return NULL;
  if (!st) {
    RosGraphErr("unknown service type",name);
    Free(fqn);
    return NULL;
  }
  c=CAlloc(sizeof(CRosClient));
  c->node=n;
  c->name=fqn;
  c->st=st;
  c->pending=RosVecNew(8);
  c->next_seq=1;
  RosGraphLock();
  c->id=ros_graph->next_id++;
  RosVecPushI64(ros_graph->clients,c);
  RosVecPushI64(n->clis,c);
  RosGraphUnlock();
  if (ros_hook_cli_new)
    (*ros_hook_cli_new)(c);
  return c;
}

U0 RosPendingFree(CRosClient *c,CRosPending *p)
{
  RosMsgDel(c->st->req,p->req);
  RosMsgDel(c->st->resp,p->resp);
  Free(p);
}

U0 RosClientDel(CRosClient *c)
{
  I64 i;
  if (!c)
    return;
  if (ros_hook_cli_del && c->dds)
    (*ros_hook_cli_del)(c);
  RosGraphLock();
  RosVecRemovePtr(ros_graph->clients,c);
  RosVecRemovePtr(c->node->clis,c);
  RosGraphUnlock();
  for (i=0;i<c->pending->len;i++)
    RosPendingFree(c,RosVecGetI64(c->pending,i));
  RosVecDel(c->pending);
  Free(c->name);
  Free(c);
}

//Is there a service with a matching type to talk to (here or, with
//networking on, on another machine)?
Bool RosClientReady(CRosClient *c)
{
  CRosService *s;
  Bool ok;
  RosGraphLock();
  s=RosMapGet(ros_graph->services,c->name);
  ok=s && s->st==c->st;
  RosGraphUnlock();
  if (!ok && ros_hook_cli_ready && c->dds)
    ok=(*ros_hook_cli_ready)(c);
  return ok;
}

CRosPending *RosClientAddPending(CRosClient *c,U8 *req,U0 (*cb)(U8 *resp,I64 seq,U8 *user),
	U8 *user)
{
  CRosPending *p=CAlloc(sizeof(CRosPending));
  RosLockGet(&c->lock);
  p->seq=c->next_seq++;
  p->req=RosMsgDup(c->st->req,req);
  p->cb=cb;
  p->user=user;
  RosVecPushI64(c->pending,p);
  RosLockPut(&c->lock);
  return p;
}

extern U0 RosClientCancel(CRosClient *c,I64 seq);

//Send a request (copied). cb(resp,seq,user) runs when the answer
//comes back; resp is only valid during the callback. Returns the
//sequence number, or -1 if no matching service exists right now.
I64 RosClientCall(CRosClient *c,U8 *req,U0 (*cb)(U8 *resp,I64 seq,U8 *user),U8 *user)
{
  CRosService *s;
  CRosSrvReq *r;
  CRosPending *p;
  I64 seq;
  RosGraphLock();
  s=RosMapGet(ros_graph->services,c->name);
  if (!s || s->st!=c->st) {
    RosGraphUnlock();
    if (ros_hook_cli_call && c->dds && ros_hook_cli_ready && (*ros_hook_cli_ready)(c)) {
      p=RosClientAddPending(c,req,cb,user);
      if ((*ros_hook_cli_call)(c,p->seq,req))
        return p->seq;
      RosClientCancel(c,p->seq);
    }
    RosGraphErr("no service to call",c->name);
    return -1;
  }
  RosLockGet(&c->lock);
  seq=c->next_seq++;
  p=CAlloc(sizeof(CRosPending));
  p->seq=seq;
  p->req=RosMsgDup(c->st->req,req);
  p->cb=cb;
  p->user=user;
  RosVecPushI64(c->pending,p);
  RosLockPut(&c->lock);
  r=CAlloc(sizeof(CRosSrvReq));
  r->cli_id=c->id;
  r->seq=seq;
  r->req=RosMsgDup(s->st->req,req);
  RosLockGet(&s->lock);
  RosVecPushI64(s->queue,r);
  RosLockPut(&s->lock);
  RosGraphUnlock();
  return seq;
}

//Send resp (taken over) to whoever asked r, and free r.
U0 RosServiceDeliver(CRosService *s,CRosSrvReq *r,U8 *resp)
{
  CRosClient *c;
  CRosPending *p;
  I64 i;
  if (r->remote) {	//asked over the network: answer there
    if (ros_hook_srv_reply)
      (*ros_hook_srv_reply)(s,r,resp);
    RosMsgDel(s->st->resp,resp);
    Free(r->remote);
    Free(r);
    return;
  }
  //hand the response to the client, if it's still there
  RosGraphLock();
  c=RosClientById(r->cli_id);
  if (c) {
    RosLockGet(&c->lock);
    for (i=0;i<c->pending->len;i++) {
      p=RosVecGetI64(c->pending,i);
      if (p->seq==r->seq && !p->done) {
        p->resp=resp;
        p->done=TRUE;
        resp=NULL;
        break;
      }
    }
    RosLockPut(&c->lock);
  }
  RosGraphUnlock();
  if (resp)
    RosMsgDel(s->st->resp,resp);
  Free(r);
}

//Inside a service callback: don't answer now. Returns a ticket for
//RosServiceReplyLater (the callback's resp is thrown away).
U8 *RosServiceDefer(CRosService *s)
{
  s->deferred=TRUE;
  return s->cur;
}

//Answer a deferred request with resp (copied).
U0 RosServiceReplyLater(CRosService *s,U8 *ticket,U8 *resp)
{
  RosServiceDeliver(s,ticket,RosMsgDup(s->st->resp,resp));
}

//Drop a deferred request without answering (the service is going away).
U0 RosServiceDropLater(CRosService *s,U8 *ticket)
{
  CRosSrvReq *r=ticket;
  Free(r->remote);
  Free(r);
}

//Handle one waiting request. TRUE if there was one.
Bool RosServiceExecOne(CRosService *s)
{
  CRosSrvReq *r=NULL;
  U8 *resp;
  RosLockGet(&s->lock);
  if (s->queue->len) {
    r=RosVecGetI64(s->queue,0);
    RosVecRemove(s->queue,0);
  }
  RosLockPut(&s->lock);
  if (!r)
    return FALSE;
  resp=RosMsgNew(s->st->resp);
  s->cur=r;
  s->deferred=FALSE;
  (*s->cb)(r->req,resp,s->user);
  s->cur=NULL;
  s->handled++;
  RosMsgDel(s->st->req,r->req);
  r->req=NULL;
  if (s->deferred) {
    s->deferred=FALSE;
    RosMsgDel(s->st->resp,resp);
    return TRUE;
  }
  RosServiceDeliver(s,r,resp);
  return TRUE;
}

//Run the callback for one answered request. TRUE if there was one.
Bool RosClientExecOne(CRosClient *c)
{
  CRosPending *p=NULL,*q;
  I64 i;
  RosLockGet(&c->lock);
  for (i=0;i<c->pending->len;i++) {
    q=RosVecGetI64(c->pending,i);
    if (q->done) {
      p=q;
      RosVecRemove(c->pending,i);
      break;
    }
  }
  RosLockPut(&c->lock);
  if (!p)
    return FALSE;
  if (p->cb)
    (*p->cb)(p->resp,p->seq,p->user);
  RosPendingFree(c,p);
  return TRUE;
}

//Forget a request you no longer care about.
U0 RosClientCancel(CRosClient *c,I64 seq)
{
  I64 i;
  CRosPending *p;
  RosLockGet(&c->lock);
  for (i=0;i<c->pending->len;i++) {
    p=RosVecGetI64(c->pending,i);
    if (p->seq==seq) {
      RosVecRemove(c->pending,i);
      RosPendingFree(c,p);
      break;
    }
  }
  RosLockPut(&c->lock);
}
