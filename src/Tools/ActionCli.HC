//ActionCli: ros2 action, as HolyC functions.
//
//  RosActionList;  RosActionListT;          ros2 action list [-t]
//  RosActionInfo("/fibonacci");             ros2 action info /fibonacci
//  RosActionSendGoalCli("/fibonacci","example_interfaces/action/Fibonacci","{order: 5}",TRUE);
//                                           ros2 action send_goal [--feedback]
//(any key while waiting for the result cancels the goal)

#define ROS_ACTION_SEND_GOAL_SUFFIX	"/_action/send_goal"

//Action names (from the send_goal services there are, here and on
//other machines), sorted.
CRosVec *RosActionNames()
{
  CRosVec *v=RosCliAllNames(RosGraph()->services,TRUE),*out=RosVecNew(8);
  I64 i,n,k=StrLen(ROS_ACTION_SEND_GOAL_SUFFIX);
  U8 *s;
  for (i=0;i<v->len;i++) {
    s=RosVecGetI64(v,i);
    n=StrLen(s);
    if (n>k && RosStrEndsWith(s,ROS_ACTION_SEND_GOAL_SUFFIX))
      RosVecPushI64(out,RosStrSub(s,0,n-k));
  }
  RosStrVecDel(v);
  return out;
}

//"pkg/action/Name" of an action (new), or NULL if we can't tell.
U8 *RosActionTypeOf(U8 *name)
{
  U8 *srv=MStrPrint("%s%s",name,ROS_ACTION_SEND_GOAL_SUFFIX),*t=NULL;
  CRosService *s;
  I64 n;
  RosGraphLock();
  s=RosMapGet(RosGraph()->services,srv);
  if (s)
    t=StrNew(s->st->name);
  RosGraphUnlock();
  if (!t)
    t=RosCliRemoteType(srv,TRUE);
  Free(srv);
  if (t && RosStrEndsWith(t,"_SendGoal")) {
    n=StrLen(t);
    t[n-9]=0;
  }
  return t;
}

U8 *RosActionListStr(Bool types)
{
  CRosVec *v=RosActionNames();
  CRosStrBuf *sb=RosStrBufNew();
  U8 *t;
  I64 i;
  for (i=0;i<v->len;i++) {
    RosStrBufAdd(sb,RosVecGetI64(v,i));
    if (types) {
      t=RosActionTypeOf(RosVecGetI64(v,i));
      if (t) {
        RosStrBufAdd(sb," [");
        RosStrBufAdd(sb,t);
        RosStrBufAddC(sb,']');
        Free(t);
      }
    }
    RosStrBufAddC(sb,'\n');
  }
  RosStrVecDel(v);
  return RosStrBufTake(sb);
}

U0 RosActionList()
{
  RosCliPrint(RosActionListStr(FALSE));
}

U0 RosActionListT()
{
  RosCliPrint(RosActionListStr(TRUE));
}

//Who serves and who uses an action (nodes on this machine).
U8 *RosActionInfoStr(U8 *name)
{
  U8 *srv=MStrPrint("%s%s",name,ROS_ACTION_SEND_GOAL_SUFFIX),*t=RosActionTypeOf(name);
  CRosStrBuf *sb=RosStrBufNew();
  CRosVec *clis=RosVecNew(8);
  CRosService *s;
  CRosClient *c;
  I64 i;
  RosStrBufAdd(sb,"Action: ");
  RosStrBufAdd(sb,name);
  RosGraphLock();
  for (i=0;i<ros_graph->clients->len;i++) {
    c=RosVecGetI64(ros_graph->clients,i);
    if (!StrCmp(c->name,srv))
      RosVecPushI64(clis,StrNew(c->node->fqn));
  }
  s=RosMapGet(ros_graph->services,srv);
  RosStrBufAdd(sb,"\nAction clients: ");
  RosStrBufAddI64(sb,clis->len);
  RosStrBufAddC(sb,'\n');
  for (i=0;i<clis->len;i++) {
    RosStrBufAdd(sb,"    ");
    RosStrBufAdd(sb,RosVecGetI64(clis,i));
    if (t) {
      RosStrBufAdd(sb," [");
      RosStrBufAdd(sb,t);
      RosStrBufAddC(sb,']');
    }
    RosStrBufAddC(sb,'\n');
  }
  RosStrBufAdd(sb,"Action servers: ");
  if (s) {
    RosStrBufAdd(sb,"1\n    ");
    RosStrBufAdd(sb,s->node->fqn);
    if (t) {
      RosStrBufAdd(sb," [");
      RosStrBufAdd(sb,t);
      RosStrBufAddC(sb,']');
    }
    RosStrBufAddC(sb,'\n');
  } else
    RosStrBufAdd(sb,"0\n");
  RosGraphUnlock();
  RosStrVecDel(clis);
  Free(srv);
  Free(t);
  return RosStrBufTake(sb);
}

U0 RosActionInfo(U8 *name)
{
  RosCliPrint(RosActionInfoStr(name));
}

class CRosCliGoal
{
  CRosStrBuf *sb;
  CRosActionType *at;
  Bool show_feedback,done;
};

//Add yaml with every line indented by n spaces.
U0 RosStrBufAddIndented(CRosStrBuf *sb,U8 *yaml,I64 n)
{
  Bool start=TRUE;
  I64 i;
  while (*yaml) {
    if (start)
      for (i=0;i<n;i++)
        RosStrBufAddC(sb,' ');
    RosStrBufAddC(sb,*yaml);
    start=*yaml=='\n';
    yaml++;
  }
}

U0 RosCliGoalFeedback(CRosGoalHandle *gh,U8 *fb,CRosCliGoal *cg)
{
  U8 *y;
  if (!cg->show_feedback)
    return;
  y=RosMsgToYaml(cg->at->feedback,fb);
  RosStrBufAdd(cg->sb,"Feedback:\n");
  RosStrBufAddIndented(cg->sb,y,4);
  RosStrBufAddC(cg->sb,'\n');
  Free(y);
}

U0 RosCliGoalResult(CRosGoalHandle *gh,I64 status,U8 *result,CRosCliGoal *cg)
{
  U8 *y;
  cg->done=TRUE;
  if (!result)
    return;
  y=RosMsgToYaml(cg->at->result,result);
  RosStrBufAdd(cg->sb,"Result:\n");
  RosStrBufAddIndented(cg->sb,y,4);
  RosStrBufAdd(cg->sb,"\nGoal finished with status: ");
  RosStrBufAdd(cg->sb,RosActionStatusName(status));
  RosStrBufAddC(cg->sb,'\n');
  Free(y);
}

//Like ros2 action send_goal [--feedback]: send, wait for the result
//(up to timeout_ns) and return what it printed.
U8 *RosActionSendGoalStr(U8 *name,U8 *type_name,U8 *values,Bool feedback,I64 timeout_ns)
{
  CRosActionType *at=RosActionFind(type_name);
  CRosNode *n;
  CRosActionClient *ac;
  CRosGoalHandle *gh;
  CRosCliGoal cg;
  U8 *goal,*y,*id;
  I64 end;
  Bool canceled=FALSE;
  if (!at)
    return MStrPrint("Unknown action type '%s'\n",type_name);
  goal=RosMsgNew(at->goal);
  if (!RosMsgFill(at->goal,goal,values)) {
    RosMsgDel(at->goal,goal);
    return MStrPrint("Bad values: %s\n",ros_msg_error);
  }
  MemSet(&cg,0,sizeof(CRosCliGoal));
  cg.sb=RosStrBufNew();
  cg.at=at;
  cg.show_feedback=feedback;
  n=RosCliNodeNew();
  ac=RosActionClientNew(n,name,at);
  RosStrBufAdd(cg.sb,"Waiting for an action server to become available...\n");
  end=RosTimeSteadyNs()+2*ROS_NS_PER_SEC;
  while (ac && !RosActionServerReady(ac) && RosTimeSteadyNs()<end && ros_ok)
    RosCliSpinOnce(n,10*ROS_NS_PER_MS);
  gh=NULL;
  if (ac && RosActionServerReady(ac)) {
    RosStrBufAdd(cg.sb,"Sending goal:\n");
    y=RosMsgToYaml(at->goal,goal);
    RosStrBufAddIndented(cg.sb,y,5);
    RosStrBufAddC(cg.sb,'\n');
    Free(y);
    gh=RosActionSendGoal(ac,goal,&RosCliGoalFeedback,&RosCliGoalResult,&cg);
  }
  if (!gh)
    RosStrBufAdd(cg.sb,"Action server not available\n");
  else {
    end=RosTimeSteadyNs()+timeout_ns;
    while (!gh->answered && RosTimeSteadyNs()<end && ros_ok)
      RosCliSpinOnce(n,10*ROS_NS_PER_MS);
    if (!gh->answered)
      RosStrBufAdd(cg.sb,"No answer from the action server\n");
    else if (!gh->accepted)
      RosStrBufAdd(cg.sb,"Goal was rejected.\n");
    else {
      id=RosUuidStr(gh->uuid);
      RosStrBufAdd(cg.sb,"Goal accepted with ID: ");
      RosStrBufAdd(cg.sb,id);
      RosStrBufAdd(cg.sb,"\n\n");
      Free(id);
      while (!cg.done && RosTimeSteadyNs()<end && ros_ok) {
        if (!canceled && RosCliKey()) {
          RosActionCancel(gh);
          canceled=TRUE;
        }
        RosCliSpinOnce(n,10*ROS_NS_PER_MS);
      }
      if (!cg.done) {
        RosActionCancel(gh);
        RosCliSpinOnce(n,10*ROS_NS_PER_MS);
        RosStrBufAdd(cg.sb,"Timed out waiting for the result (goal canceled)\n");
      }
    }
  }
  RosActionClientDel(ac);
  RosCliNodeDel(n);
  RosMsgDel(at->goal,goal);
  return RosStrBufTake(cg.sb);
}

U0 RosActionSendGoalCli(U8 *name,U8 *type_name,U8 *values,Bool feedback)
{
  RosCliPrint(RosActionSendGoalStr(name,type_name,values,feedback,1000*ROS_NS_PER_SEC));
}
