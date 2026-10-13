//Action: ROS 2 actions (long-running goals with feedback), built from
//three services and two topics under <name>/_action/, exactly as rcl
//does, so rclcpp/rclpy action clients and servers talk to ours:
//  send_goal   (pkg/action/Name_SendGoal)   accept or reject a goal
//  cancel_goal (action_msgs/srv/CancelGoal)
//  get_result  (pkg/action/Name_GetResult)  answered once the goal ends
//  feedback    (pkg/action/Name_FeedbackMessage)
//  status      (action_msgs/msg/GoalStatusArray, transient local)
//
//Server:
//  Bool OnGoal(CRosActionServer *as,CAct_example_interfaces_Fibonacci_Goal *g,U8 *user)
//  { return g->order<50; }                       //accept?
//  U0 OnAccepted(CRosGoal *g,U8 *user) { ... }   //start working (timer, task...)
//  as=RosActionServerNew(node,"fibonacci",ros_act_example_interfaces_Fibonacci,
//	&OnGoal,NULL,&OnAccepted,NULL);          //NULL cancel callback = always allow
//  RosGoalFeedback(g,fb);  RosGoalSucceed(g,result);  (or Abort / Canceled)
//
//Client:
//  ac=RosActionClientNew(node,"fibonacci",ros_act_example_interfaces_Fibonacci);
//  gh=RosActionSendGoal(ac,goal,&OnFeedback,&OnResult,user);
//  RosActionCancel(gh);
//
//Callbacks run in the node's executor. Delete action servers and
//clients before their node.

#define ROS_GOAL_UNKNOWN	0
#define ROS_GOAL_ACCEPTED	1
#define ROS_GOAL_EXECUTING	2
#define ROS_GOAL_CANCELING	3
#define ROS_GOAL_SUCCEEDED	4
#define ROS_GOAL_CANCELED	5
#define ROS_GOAL_ABORTED	6
#define ROS_ACTION_RESULT_NS	900000000000	//keep results 15 min, like rcl

class CRosActionServer
{
  CRosNode *node;
  U8 *name;		//fully qualified action name
  CRosActionType *at;
  CRosService *send_goal,*cancel,*get_result;
  CRosPub *feedback,*status;
  CRosTimer *expire;
  CRosVec *goals;	//CRosGoal *
  Bool (*goal_cb)(U8 *as,U8 *goal,U8 *user);	//(CRosActionServer *,Name_Goal *,user)
  Bool (*cancel_cb)(U8 *g,U8 *user);		//(CRosGoal *,user)
  U0 (*accepted_cb)(U8 *g,U8 *user);		//(CRosGoal *,user)
  U8 *user;
  I64 result_ns;
};

class CRosGoal		//a goal on the server
{
  CRosActionServer *as;
  U8 uuid[16];
  I64 status,stamp,ended;
  U8 *goal;		//Name_Goal
  U8 *result;		//Name_Result, once it ended
  CRosVec *waiting;	//get_result tickets to answer when it ends
  U8 *user;		//yours
};

class CRosActionClient
{
  CRosNode *node;
  U8 *name;
  CRosActionType *at;
  CRosClient *send_goal,*cancel,*get_result;
  CRosSub *feedback,*status;
  CRosVec *goals;	//CRosGoalHandle *
};

class CRosGoalHandle	//a goal on the client
{
  CRosActionClient *ac;
  U8 uuid[16];
  I64 status,stamp;
  Bool answered,accepted,done;
  U8 *result;		//Name_Result once done
  U0 (*feedback_cb)(U8 *gh,U8 *feedback,U8 *user);	//(CRosGoalHandle *,Name_Feedback *,user)
  U0 (*result_cb)(U8 *gh,I64 status,U8 *result,U8 *user);
  U8 *user;
};

U8 *RosActionStatusName(I64 s)
{
  switch (s) {
    case ROS_GOAL_ACCEPTED:	return "ACCEPTED";
    case ROS_GOAL_EXECUTING:	return "EXECUTING";
    case ROS_GOAL_CANCELING:	return "CANCELING";
    case ROS_GOAL_SUCCEEDED:	return "SUCCEEDED";
    case ROS_GOAL_CANCELED:	return "CANCELED";
    case ROS_GOAL_ABORTED:	return "ABORTED";
  }
  return "UNKNOWN";
}

Bool RosGoalTerminal(I64 s)
{
  return s>=ROS_GOAL_SUCCEEDED;
}

//Pointer to field name of message m of type t.
U8 *RosFieldAt(CRosType *t,U8 *m,U8 *name)
{
  return m+RosTypeField(t,name)->offset;
}

//"/fibonacci" + "send_goal" -> "/fibonacci/_action/send_goal" (new)
U8 *RosActionSub(U8 *fqn,U8 *what)
{
  if (!StrCmp(fqn,"/"))
    return MStrPrint("/_action/%s",what);
  return MStrPrint("%s/_action/%s",fqn,what);
}

U0 RosStampFill(CMsg_builtin_interfaces_Time *t,I64 ns)
{
  t->sec=RosTimeSec(ns);
  t->nanosec=RosTimeNsec(ns);
}

Bool RosUuidZero(U8 *u)
{
  I64 i;
  for (i=0;i<16;i++)
    if (u[i])
      return FALSE;
  return TRUE;
}

//---- server

CRosGoal *RosActionGoalFind(CRosActionServer *as,U8 *uuid)
{
  I64 i;
  CRosGoal *g;
  for (i=0;i<as->goals->len;i++) {
    g=RosVecGetI64(as->goals,i);
    if (!MemCmp(g->uuid,uuid,16))
      return g;
  }
  return NULL;
}

//Publish every goal's state on <name>/_action/status.
U0 RosActionPublishStatus(CRosActionServer *as)
{
  CMsg_action_msgs_GoalStatusArray *m=RosPubLoan(as->status);
  CMsg_action_msgs_GoalStatus *st;
  CRosGoal *g;
  I64 i;
  RosSeqResize(RosTypeField(ros_t_action_msgs_GoalStatusArray,"status_list"),m,as->goals->len);
  st=m->status_list.data;
  for (i=0;i<as->goals->len;i++) {
    g=RosVecGetI64(as->goals,i);
    MemCpy(st[i].goal_info.goal_id.uuid,g->uuid,16);
    RosStampFill(&st[i].goal_info.stamp,g->stamp);
    st[i].status=g->status;
  }
  RosPublishLoaned(as->status,m);
}

U0 RosActionGoalFree(CRosGoal *g)
{
  CRosActionType *at=g->as->at;
  I64 i;
  for (i=0;i<g->waiting->len;i++)
    RosServiceDropLater(g->as->get_result,RosVecGetI64(g->waiting,i));
  RosVecDel(g->waiting);
  RosMsgDel(at->goal,g->goal);
  RosMsgDel(at->result,g->result);
  Free(g);
}

U0 RosActionOnSendGoal(U8 *req,U8 *resp,CRosActionServer *as)
{
  CRosType *rt=as->at->send_goal->req,*pt=as->at->send_goal->resp;
  U8 *uuid=RosFieldAt(rt,req,"goal_id"),*goal=RosFieldAt(rt,req,"goal");
  I64 *acc=RosFieldAt(pt,resp,"accepted");
  CRosGoal *g;
  Bool ok=TRUE;
  if (RosActionGoalFind(as,uuid))	//same id twice: reject
    ok=FALSE;
  if (ok && as->goal_cb)
    ok=(*as->goal_cb)(as,goal,as->user);
  *acc=ok;
  if (!ok)
    return;
  g=CAlloc(sizeof(CRosGoal));
  g->as=as;
  MemCpy(g->uuid,uuid,16);
  g->status=ROS_GOAL_ACCEPTED;
  g->stamp=RosClockNow(ROS_CLOCK_ROS);
  g->goal=RosMsgDup(as->at->goal,goal);
  g->waiting=RosVecNew(8);
  RosVecPushI64(as->goals,g);
  RosStampFill(RosFieldAt(pt,resp,"stamp"),g->stamp);
  RosActionPublishStatus(as);
  if (as->accepted_cb)
    (*as->accepted_cb)(g,as->user);
}

//Cancel policy from action_msgs/srv/CancelGoal.
U0 RosActionOnCancel(CSrv_action_msgs_CancelGoal_Request *req,
	CSrv_action_msgs_CancelGoal_Response *resp,CRosActionServer *as)
{
  CRosField *f=RosTypeField(ros_t_action_msgs_CancelGoal_Response,"goals_canceling");
  CMsg_action_msgs_GoalInfo *gi;
  CRosGoal *g;
  U8 *id=req->goal_info.goal_id.uuid;
  I64 i,before=RosTimeMake(req->goal_info.stamp.sec,req->goal_info.stamp.nanosec),n=0;
  Bool any_id=RosUuidZero(id),match,terminal_match=FALSE,id_found=FALSE,ok;
  for (i=0;i<as->goals->len;i++) {
    g=RosVecGetI64(as->goals,i);
    match=FALSE;
    if (!any_id && !MemCmp(g->uuid,id,16)) {
      match=TRUE;
      id_found=TRUE;
    }
    if (any_id && !before)
      match=TRUE;
    if (before && g->stamp<=before)
      match=TRUE;
    if (!match)
      continue;
    if (RosGoalTerminal(g->status)) {
      terminal_match=TRUE;
      continue;
    }
    ok=TRUE;
    if (g->status!=ROS_GOAL_CANCELING && as->cancel_cb)
      ok=(*as->cancel_cb)(g,as->user);
    if (!ok)
      continue;
    g->status=ROS_GOAL_CANCELING;
    RosSeqResize(f,resp,n+1);
    gi=resp->goals_canceling.data;
    MemCpy(gi[n].goal_id.uuid,g->uuid,16);
    RosStampFill(&gi[n].stamp,g->stamp);
    n++;
  }
  if (n)
    resp->return_code=ROS_action_msgs_CancelGoal_Response_ERROR_NONE;
  else if (!any_id && !id_found)
    resp->return_code=ROS_action_msgs_CancelGoal_Response_ERROR_UNKNOWN_GOAL_ID;
  else if (terminal_match)
    resp->return_code=ROS_action_msgs_CancelGoal_Response_ERROR_GOAL_TERMINATED;
  else
    resp->return_code=ROS_action_msgs_CancelGoal_Response_ERROR_REJECTED;
  if (n)
    RosActionPublishStatus(as);
}

//Fill a GetResult response for g.
U0 RosActionFillResult(CRosGoal *g,U8 *resp)
{
  CRosType *pt=g->as->at->get_result->resp;
  I64 *st=RosFieldAt(pt,resp,"status");
  *st=g->status;
  RosMsgCopy(g->as->at->result,RosFieldAt(pt,resp,"result"),g->result);
}

U0 RosActionOnGetResult(U8 *req,U8 *resp,CRosActionServer *as)
{
  CRosType *rt=as->at->get_result->req,*pt=as->at->get_result->resp;
  CRosGoal *g=RosActionGoalFind(as,RosFieldAt(rt,req,"goal_id"));
  I64 *st=RosFieldAt(pt,resp,"status");
  if (!g) {
    *st=ROS_GOAL_UNKNOWN;
    return;
  }
  if (RosGoalTerminal(g->status)) {
    RosActionFillResult(g,resp);
    return;
  }
  RosVecPushI64(g->waiting,RosServiceDefer(as->get_result));
}

//Forget goals whose results are older than result_ns.
U0 RosActionExpire(CRosActionServer *as)
{
  I64 i=0,now=RosTimeSteadyNs();
  CRosGoal *g;
  Bool changed=FALSE;
  while (i<as->goals->len) {
    g=RosVecGetI64(as->goals,i);
    if (RosGoalTerminal(g->status) && now-g->ended>=as->result_ns) {
      RosVecRemove(as->goals,i);
      RosActionGoalFree(g);
      changed=TRUE;
    } else
      i++;
  }
  if (changed)
    RosActionPublishStatus(as);
}

//goal_cb (NULL = accept all), cancel_cb (NULL = allow all) and
//accepted_cb (start the work) get user.
CRosActionServer *RosActionServerNew(CRosNode *n,U8 *name,CRosActionType *at,
	Bool (*goal_cb)(U8 *as,U8 *goal,U8 *user),
	Bool (*cancel_cb)(U8 *g,U8 *user),
	U0 (*accepted_cb)(U8 *g,U8 *user),U8 *user)
{
  CRosActionServer *as;
  U8 *fqn,*s;
  CRosQoS q;
  if (!at) {
    RosGraphErr("unknown action type",name);
    return NULL;
  }
  fqn=RosNodeResolve(n,name);
  if (!fqn)
    return NULL;
  as=CAlloc(sizeof(CRosActionServer));
  as->node=n;
  as->name=fqn;
  as->at=at;
  as->goal_cb=goal_cb;
  as->cancel_cb=cancel_cb;
  as->accepted_cb=accepted_cb;
  as->user=user;
  as->result_ns=ROS_ACTION_RESULT_NS;
  as->goals=RosVecNew(8);
  s=RosActionSub(fqn,"send_goal");
  as->send_goal=RosServiceNew(n,s,at->send_goal,&RosActionOnSendGoal,as);
  Free(s);
  s=RosActionSub(fqn,"cancel_goal");
  as->cancel=RosServiceNew(n,s,ros_srv_action_msgs_CancelGoal,&RosActionOnCancel,as);
  Free(s);
  s=RosActionSub(fqn,"get_result");
  as->get_result=RosServiceNew(n,s,at->get_result,&RosActionOnGetResult,as);
  Free(s);
  RosQoSDefault(&q);
  s=RosActionSub(fqn,"feedback");
  as->feedback=RosPubNew(n,s,at->feedback_msg,&q);
  Free(s);
  RosQoSTransientLocal(&q,1);
  s=RosActionSub(fqn,"status");
  as->status=RosPubNew(n,s,ros_t_action_msgs_GoalStatusArray,&q);
  Free(s);
  if (!as->send_goal || !as->cancel || !as->get_result || !as->feedback || !as->status) {
    RosServiceDel(as->send_goal);
    RosServiceDel(as->cancel);
    RosServiceDel(as->get_result);
    RosPubDel(as->feedback);
    RosPubDel(as->status);
    RosVecDel(as->goals);
    Free(as->name);
    Free(as);
    return NULL;
  }
  as->expire=RosTimerNew(n,ROS_NS_PER_SEC,&RosActionExpire,as);
  return as;
}

U0 RosActionServerDel(CRosActionServer *as)
{
  I64 i;
  if (!as)
    return;
  RosTimerDel(as->expire);
  for (i=0;i<as->goals->len;i++)
    RosActionGoalFree(RosVecGetI64(as->goals,i));
  RosVecDel(as->goals);
  RosServiceDel(as->send_goal);
  RosServiceDel(as->cancel);
  RosServiceDel(as->get_result);
  RosPubDel(as->feedback);
  RosPubDel(as->status);
  Free(as->name);
  Free(as);
}

//The goal is being worked on.
U0 RosGoalExecute(CRosGoal *g)
{
  if (g->status!=ROS_GOAL_ACCEPTED)
    return;
  g->status=ROS_GOAL_EXECUTING;
  RosActionPublishStatus(g->as);
}

//Has a client asked to cancel it?
Bool RosGoalIsCanceling(CRosGoal *g)
{
  return g->status==ROS_GOAL_CANCELING;
}

//Send feedback (a Name_Feedback message, copied).
U0 RosGoalFeedback(CRosGoal *g,U8 *feedback)
{
  CRosType *ft=g->as->at->feedback_msg;
  U8 *m=RosPubLoan(g->as->feedback);
  MemCpy(RosFieldAt(ft,m,"goal_id"),g->uuid,16);
  RosMsgCopy(g->as->at->feedback,RosFieldAt(ft,m,"feedback"),feedback);
  RosPublishLoaned(g->as->feedback,m);
}

//End the goal with status and result (copied; NULL = empty result).
//Answers every get_result waiting for it.
U0 RosGoalEnd(CRosGoal *g,I64 status,U8 *result)
{
  CRosActionServer *as=g->as;
  CRosType *pt=as->at->get_result->resp;
  U8 *resp;
  I64 i;
  if (RosGoalTerminal(g->status))
    return;
  g->status=status;
  g->ended=RosTimeSteadyNs();
  if (result)
    g->result=RosMsgDup(as->at->result,result);
  else
    g->result=RosMsgNew(as->at->result);
  if (g->waiting->len) {
    resp=RosMsgNew(pt);
    RosActionFillResult(g,resp);
    for (i=0;i<g->waiting->len;i++)
      RosServiceReplyLater(as->get_result,RosVecGetI64(g->waiting,i),resp);
    RosVecClear(g->waiting);
    RosMsgDel(pt,resp);
  }
  RosActionPublishStatus(as);
}

U0 RosGoalSucceed(CRosGoal *g,U8 *result)
{
  RosGoalEnd(g,ROS_GOAL_SUCCEEDED,result);
}

U0 RosGoalAbort(CRosGoal *g,U8 *result)
{
  RosGoalEnd(g,ROS_GOAL_ABORTED,result);
}

U0 RosGoalCanceled(CRosGoal *g,U8 *result)
{
  RosGoalEnd(g,ROS_GOAL_CANCELED,result);
}

//---- client

CRosGoalHandle *RosActionHandleFind(CRosActionClient *ac,U8 *uuid)
{
  I64 i;
  CRosGoalHandle *gh;
  for (i=0;i<ac->goals->len;i++) {
    gh=RosVecGetI64(ac->goals,i);
    if (!MemCmp(gh->uuid,uuid,16))
      return gh;
  }
  return NULL;
}

U0 RosActionOnFeedback(U8 *m,CRosActionClient *ac)
{
  CRosType *ft=ac->at->feedback_msg;
  CRosGoalHandle *gh=RosActionHandleFind(ac,RosFieldAt(ft,m,"goal_id"));
  if (gh && !gh->done && gh->feedback_cb)
    (*gh->feedback_cb)(gh,RosFieldAt(ft,m,"feedback"),gh->user);
}

U0 RosActionOnStatus(CMsg_action_msgs_GoalStatusArray *m,CRosActionClient *ac)
{
  CMsg_action_msgs_GoalStatus *st=m->status_list.data;
  CRosGoalHandle *gh;
  I64 i;
  for (i=0;i<m->status_list.len;i++) {
    gh=RosActionHandleFind(ac,st[i].goal_info.goal_id.uuid);
    if (gh && !gh->done)
      gh->status=st[i].status;
  }
}

U0 RosActionOnResult(U8 *resp,I64 seq,CRosGoalHandle *gh)
{
  CRosActionClient *ac=gh->ac;
  CRosType *pt=ac->at->get_result->resp;
  I64 *st=RosFieldAt(pt,resp,"status");
  gh->status=*st;
  gh->done=TRUE;
  gh->result=RosMsgDup(ac->at->result,RosFieldAt(pt,resp,"result"));
  if (gh->result_cb)
    (*gh->result_cb)(gh,gh->status,gh->result,gh->user);
}

U0 RosActionOnGoalResponse(U8 *resp,I64 seq,CRosGoalHandle *gh)
{
  CRosActionClient *ac=gh->ac;
  CRosType *pt=ac->at->send_goal->resp,*rt=ac->at->get_result->req;
  CMsg_builtin_interfaces_Time *t=RosFieldAt(pt,resp,"stamp");
  I64 *acc=RosFieldAt(pt,resp,"accepted");
  U8 *req;
  gh->answered=TRUE;
  gh->accepted=*acc!=0;
  if (!gh->accepted) {
    gh->done=TRUE;
    gh->status=ROS_GOAL_UNKNOWN;
    if (gh->result_cb)
      (*gh->result_cb)(gh,ROS_GOAL_UNKNOWN,NULL,gh->user);
    return;
  }
  gh->stamp=RosTimeMake(t->sec,t->nanosec);
  if (gh->status==ROS_GOAL_UNKNOWN)
    gh->status=ROS_GOAL_ACCEPTED;
  req=RosMsgNew(rt);
  MemCpy(RosFieldAt(rt,req,"goal_id"),gh->uuid,16);
  RosClientCall(ac->get_result,req,&RosActionOnResult,gh);
  RosMsgDel(rt,req);
}

CRosActionClient *RosActionClientNew(CRosNode *n,U8 *name,CRosActionType *at)
{
  CRosActionClient *ac;
  U8 *fqn,*s;
  CRosQoS q;
  if (!at) {
    RosGraphErr("unknown action type",name);
    return NULL;
  }
  fqn=RosNodeResolve(n,name);
  if (!fqn)
    return NULL;
  ac=CAlloc(sizeof(CRosActionClient));
  ac->node=n;
  ac->name=fqn;
  ac->at=at;
  ac->goals=RosVecNew(8);
  s=RosActionSub(fqn,"send_goal");
  ac->send_goal=RosClientNew(n,s,at->send_goal);
  Free(s);
  s=RosActionSub(fqn,"cancel_goal");
  ac->cancel=RosClientNew(n,s,ros_srv_action_msgs_CancelGoal);
  Free(s);
  s=RosActionSub(fqn,"get_result");
  ac->get_result=RosClientNew(n,s,at->get_result);
  Free(s);
  RosQoSDefault(&q);
  q.depth=100;
  s=RosActionSub(fqn,"feedback");
  ac->feedback=RosSubNew(n,s,at->feedback_msg,&q,&RosActionOnFeedback,ac);
  Free(s);
  RosQoSTransientLocal(&q,1);
  s=RosActionSub(fqn,"status");
  ac->status=RosSubNew(n,s,ros_t_action_msgs_GoalStatusArray,&q,&RosActionOnStatus,ac);
  Free(s);
  return ac;
}

U0 RosActionClientDel(CRosActionClient *ac)
{
  I64 i;
  CRosGoalHandle *gh;
  if (!ac)
    return;
  RosClientDel(ac->send_goal);
  RosClientDel(ac->cancel);
  RosClientDel(ac->get_result);
  RosSubDel(ac->feedback);
  RosSubDel(ac->status);
  for (i=0;i<ac->goals->len;i++) {
    gh=RosVecGetI64(ac->goals,i);
    RosMsgDel(ac->at->result,gh->result);
    Free(gh);
  }
  RosVecDel(ac->goals);
  Free(ac->name);
  Free(ac);
}

//Is there an action server to talk to?
Bool RosActionServerReady(CRosActionClient *ac)
{
  return RosClientReady(ac->send_goal) && RosClientReady(ac->cancel) &&
	RosClientReady(ac->get_result);
}

//Send a goal (a Name_Goal message, copied). feedback_cb(gh,feedback,user)
//runs for each feedback; result_cb(gh,status,result,user) once at the
//end (status UNKNOWN and result NULL if the goal was rejected). NULL if
//there's no server. The handle belongs to ac (freed with it).
CRosGoalHandle *RosActionSendGoal(CRosActionClient *ac,U8 *goal,
	U0 (*feedback_cb)(U8 *gh,U8 *feedback,U8 *user),
	U0 (*result_cb)(U8 *gh,I64 status,U8 *result,U8 *user),U8 *user)
{
  CRosType *rt=ac->at->send_goal->req;
  CRosGoalHandle *gh=CAlloc(sizeof(CRosGoalHandle));
  U8 *req=RosMsgNew(rt);
  U64 r=0;
  I64 i;
  gh->ac=ac;
  for (i=0;i<16;i++) {	//random UUID (version 4)
    if (!(i&7))
      r=RandU64();
    gh->uuid[i]=r&255;
    r=r>>8;
  }
  gh->uuid[6]=(gh->uuid[6]&0x0F)|0x40;
  gh->uuid[8]=(gh->uuid[8]&0x3F)|0x80;
  gh->feedback_cb=feedback_cb;
  gh->result_cb=result_cb;
  gh->user=user;
  MemCpy(RosFieldAt(rt,req,"goal_id"),gh->uuid,16);
  RosMsgCopy(ac->at->goal,RosFieldAt(rt,req,"goal"),goal);
  RosVecPushI64(ac->goals,gh);
  if (RosClientCall(ac->send_goal,req,&RosActionOnGoalResponse,gh)<0) {
    RosVecRemovePtr(ac->goals,gh);
    Free(gh);
    gh=NULL;
  }
  RosMsgDel(rt,req);
  return gh;
}

//Ask the server to cancel the goal. The result callback still runs
//(usually with status CANCELED).
Bool RosActionCancel(CRosGoalHandle *gh)
{
  CSrv_action_msgs_CancelGoal_Request req;
  MemSet(&req,0,sizeof(CSrv_action_msgs_CancelGoal_Request));
  MemCpy(req.goal_info.goal_id.uuid,gh->uuid,16);
  return RosClientCall(gh->ac->cancel,&req,NULL,NULL)>=0;
}

//"0f3c...-..." text of a goal id (new).
U8 *RosUuidStr(U8 *u)
{
  CRosStrBuf *sb=RosStrBufNew();
  U8 *hex="0123456789abcdef";
  I64 i;
  for (i=0;i<16;i++) {
    if (i==4 || i==6 || i==8 || i==10)
      RosStrBufAddC(sb,'-');
    RosStrBufAddC(sb,hex[u[i]>>4]);
    RosStrBufAddC(sb,hex[u[i]&15]);
  }
  return RosStrBufTake(sb);
}
