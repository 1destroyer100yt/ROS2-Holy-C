//Tests for src/Graph/Action.HC.

class CTestFib	//a Fibonacci server that does one step per timer tick
{
  CRosGoal *g;
  I64 i,n,a,b;
  CRosTimer *t;
  I64 goals,cancels;
  Bool allow_cancel;
};

class CTestFibCli
{
  I64 feedbacks,results,last_status,last_len,last_value;
};

Bool TestFibOnGoal(CRosActionServer *as,CAct_example_interfaces_Fibonacci_Goal *goal,CTestFib *f)
{
  f->goals++;
  return goal->order>=0 && goal->order<=40;	//reject silly ones
}

Bool TestFibOnCancel(CRosGoal *g,CTestFib *f)
{
  f->cancels++;
  return f->allow_cancel;
}

U0 TestFibOnAccepted(CRosGoal *g,CTestFib *f)
{
  CAct_example_interfaces_Fibonacci_Goal *goal=g->goal;
  f->g=g;
  f->n=goal->order;
  f->i=0;
  f->a=0;
  f->b=1;
  RosGoalExecute(g);
}

U0 TestFibStep(CTestFib *f)
{
  CAct_example_interfaces_Fibonacci_Feedback fb;
  CAct_example_interfaces_Fibonacci_Result res;
  CRosField *sf=RosTypeField(ros_t_example_interfaces_Fibonacci_Feedback,"sequence");
  I64 j,*v,c;
  if (!f->g)
    return;
  if (RosGoalIsCanceling(f->g)) {
    RosGoalCanceled(f->g,NULL);
    f->g=NULL;
    return;
  }
  MemSet(&fb,0,sizeof(fb));
  RosSeqResize(sf,&fb,f->i+1);
  v=fb.sequence.data;
  c=0;
  for (j=0;j<=f->i;j++) {	//the sequence so far
    if (j==0)
      v[j]=0;
    else if (j==1)
      v[j]=1;
    else
      v[j]=v[j-1]+v[j-2];
  }
  if (f->i<f->n) {
    RosGoalFeedback(f->g,&fb);
    f->i++;
  } else {
    MemSet(&res,0,sizeof(res));
    RosMsgCopy(ros_t_example_interfaces_Fibonacci_Result,&res,&fb);	//same layout
    RosGoalSucceed(f->g,&res);
    RosMsgFini(ros_t_example_interfaces_Fibonacci_Result,&res);
    f->g=NULL;
  }
  RosMsgFini(ros_t_example_interfaces_Fibonacci_Feedback,&fb);
}

U0 TestFibCliFeedback(CRosGoalHandle *gh,CAct_example_interfaces_Fibonacci_Feedback *fb,
	CTestFibCli *c)
{
  c->feedbacks++;
}

U0 TestFibCliResult(CRosGoalHandle *gh,I64 status,CAct_example_interfaces_Fibonacci_Result *r,
	CTestFibCli *c)
{
  I64 *v;
  c->results++;
  c->last_status=status;
  c->last_len=-1;
  if (r) {
    c->last_len=r->sequence.len;
    v=r->sequence.data;
    if (r->sequence.len)
      c->last_value=v[r->sequence.len-1];
  }
}

U0 TestActionSpin(CRosExec *e,CTestFibCli *c,I64 want_results)
{
  I64 i;
  for (i=0;i<2000 && c->results<want_results;i++)
    RosExecSpinSome(e);
}

U0 TestActionBasics()
{
  CRosNode *sn=RosNodeNew("fib_server",NULL),*cn=RosNodeNew("fib_client",NULL);
  CRosExec *e=RosExecNew();
  CRosActionServer *as;
  CRosActionClient *ac;
  CRosGoalHandle *gh,*gh2;
  CTestFib f;
  CTestFibCli c;
  CAct_example_interfaces_Fibonacci_Goal goal;
  RosTestGroup("Actions");
  MemSet(&f,0,sizeof(f));
  MemSet(&c,0,sizeof(c));
  RosExecAdd(e,sn);
  RosExecAdd(e,cn);
  ac=RosActionClientNew(cn,"fibonacci",ros_act_example_interfaces_Fibonacci);
  RosTestTrue("no server yet",!RosActionServerReady(ac));
  as=RosActionServerNew(sn,"fibonacci",ros_act_example_interfaces_Fibonacci,
	&TestFibOnGoal,&TestFibOnCancel,&TestFibOnAccepted,&f);
  RosTestTrue("server made",as!=NULL);
  RosTestStr("server name",as->name,"/fibonacci");
  RosTestTrue("services exist",RosMapHas(RosGraph()->services,"/fibonacci/_action/send_goal") &&
	RosMapHas(RosGraph()->services,"/fibonacci/_action/cancel_goal") &&
	RosMapHas(RosGraph()->services,"/fibonacci/_action/get_result"));
  RosTestTrue("topics exist",RosMapHas(RosGraph()->topics,"/fibonacci/_action/feedback") &&
	RosMapHas(RosGraph()->topics,"/fibonacci/_action/status"));
  RosTestTrue("server ready",RosActionServerReady(ac));
  f.t=RosTimerNew(sn,0,&TestFibStep,&f);	//a step per spin
  MemSet(&goal,0,sizeof(goal));
  goal.order=10;
  gh=RosActionSendGoal(ac,&goal,&TestFibCliFeedback,&TestFibCliResult,&c);
  RosTestTrue("goal sent",gh!=NULL);
  TestActionSpin(e,&c,1);
  RosTestTrue("accepted",gh->answered && gh->accepted);
  RosTestI64("result arrived",c.results,1);
  RosTestI64("status succeeded",c.last_status,ROS_GOAL_SUCCEEDED);
  RosTestI64("feedback count",c.feedbacks,10);
  RosTestI64("result length",c.last_len,11);
  RosTestI64("fib(10)",c.last_value,55);
  RosTestTrue("handle done",gh->done && gh->status==ROS_GOAL_SUCCEEDED);
  //rejected goal
  goal.order=1000;
  gh2=RosActionSendGoal(ac,&goal,NULL,&TestFibCliResult,&c);
  TestActionSpin(e,&c,2);
  RosTestTrue("rejected",gh2->answered && !gh2->accepted && gh2->done);
  RosTestI64("rejected: status unknown",c.last_status,ROS_GOAL_UNKNOWN);
  RosTestI64("rejected: no result",c.last_len,-1);
  //cancel while running (refused, then allowed)
  goal.order=40;
  gh=RosActionSendGoal(ac,&goal,&TestFibCliFeedback,&TestFibCliResult,&c);
  while (!gh->answered)
    RosExecSpinSome(e);
  RosTestTrue("cancel sent",RosActionCancel(gh));
  RosExecSpinSome(e);
  RosExecSpinSome(e);
  RosTestI64("cancel callback ran",f.cancels,1);
  RosTestTrue("refused cancel: still running",!gh->done);
  f.allow_cancel=TRUE;
  RosActionCancel(gh);
  TestActionSpin(e,&c,3);
  RosTestI64("canceled",c.last_status,ROS_GOAL_CANCELED);
  RosTestI64("server goals kept for results",as->goals->len,2);
  //results expire
  as->result_ns=0;
  RosActionExpire(as);
  RosTestI64("expired",as->goals->len,0);
  RosTimerDel(f.t);
  RosActionClientDel(ac);
  RosActionServerDel(as);
  RosNodeDel(sn);
  RosNodeDel(cn);
  RosExecDel(e);
}

U0 TestActionCancelPolicy()
{
  CRosNode *n=RosNodeNew("cancel_policy",NULL);
  CRosActionServer *as;
  CSrv_action_msgs_CancelGoal_Request req;
  CSrv_action_msgs_CancelGoal_Response *resp;
  CAct_example_interfaces_Fibonacci_SendGoal_Request *sg;
  CAct_example_interfaces_Fibonacci_SendGoal_Response *sr;
  I64 i;
  CRosGoal *g;
  U8 *s;
  RosTestGroup("Action cancel policy");
  as=RosActionServerNew(n,"/policy",ros_act_example_interfaces_Fibonacci,NULL,NULL,NULL,NULL);
  sg=RosMsgNew(ros_t_example_interfaces_Fibonacci_SendGoal_Request);
  for (i=0;i<3;i++) {	//three goals, straight through the callback
    sr=RosMsgNew(ros_t_example_interfaces_Fibonacci_SendGoal_Response);
    sg->goal_id.uuid[0]=i+1;
    RosActionOnSendGoal(sg,sr,as);
    RosTestTrue("accepted by default",sr->accepted!=0);
    RosMsgDel(ros_t_example_interfaces_Fibonacci_SendGoal_Response,sr);
  }
  sr=RosMsgNew(ros_t_example_interfaces_Fibonacci_SendGoal_Response);
  RosActionOnSendGoal(sg,sr,as);
  RosTestTrue("same id twice: rejected",!sr->accepted);
  RosMsgDel(ros_t_example_interfaces_Fibonacci_SendGoal_Response,sr);
  RosMsgDel(ros_t_example_interfaces_Fibonacci_SendGoal_Request,sg);
  g=RosVecGetI64(as->goals,0);
  g->stamp=100;
  g=RosVecGetI64(as->goals,1);
  g->stamp=200;
  g=RosVecGetI64(as->goals,2);
  g->stamp=300;
  RosGoalSucceed(g,NULL);
  MemSet(&req,0,sizeof(req));
  req.goal_info.goal_id.uuid[0]=9;
  resp=RosMsgNew(ros_t_action_msgs_CancelGoal_Response);
  RosActionOnCancel(&req,resp,as);
  RosTestI64("unknown id",resp->return_code,ROS_action_msgs_CancelGoal_Response_ERROR_UNKNOWN_GOAL_ID);
  RosMsgDel(ros_t_action_msgs_CancelGoal_Response,resp);
  req.goal_info.goal_id.uuid[0]=3;
  resp=RosMsgNew(ros_t_action_msgs_CancelGoal_Response);
  RosActionOnCancel(&req,resp,as);
  RosTestI64("terminal goal",resp->return_code,ROS_action_msgs_CancelGoal_Response_ERROR_GOAL_TERMINATED);
  RosMsgDel(ros_t_action_msgs_CancelGoal_Response,resp);
  req.goal_info.goal_id.uuid[0]=0;	//zero id + stamp: goals at or before it
  req.goal_info.stamp.nanosec=150;
  resp=RosMsgNew(ros_t_action_msgs_CancelGoal_Response);
  RosActionOnCancel(&req,resp,as);
  RosTestI64("by time: ok",resp->return_code,ROS_action_msgs_CancelGoal_Response_ERROR_NONE);
  RosTestI64("by time: one goal",resp->goals_canceling.len,1);
  RosMsgDel(ros_t_action_msgs_CancelGoal_Response,resp);
  req.goal_info.stamp.nanosec=0;	//zero id, zero stamp: everything
  resp=RosMsgNew(ros_t_action_msgs_CancelGoal_Response);
  RosActionOnCancel(&req,resp,as);
  RosTestI64("all: two (one is done)",resp->goals_canceling.len,2);
  RosMsgDel(ros_t_action_msgs_CancelGoal_Response,resp);
  g=RosVecGetI64(as->goals,0);
  RosTestTrue("canceling",RosGoalIsCanceling(g));
  s=RosUuidStr(g->uuid);
  RosTestStr("uuid text",s,"01000000-0000-0000-0000-000000000000");
  Free(s);
  RosActionServerDel(as);
  RosNodeDel(n);
}


U0 TestActionCli()
{
  CRosNode *sn=RosNodeNew("fib_cli_server",NULL);
  CRosExec *e=RosExecNew();
  CRosActionServer *as;
  CTestFib f;
  U8 *s;
  RosTestGroup("Action CLI");
  MemSet(&f,0,sizeof(f));
  RosExecAdd(e,sn);
  as=RosActionServerNew(sn,"fibonacci",ros_act_example_interfaces_Fibonacci,
	&TestFibOnGoal,&TestFibOnCancel,&TestFibOnAccepted,&f);
  f.t=RosTimerNew(sn,0,&TestFibStep,&f);
  s=RosActionListStr(TRUE);
  RosTestStr("action list -t",s,"/fibonacci [example_interfaces/action/Fibonacci]\n");
  Free(s);
  s=RosTopicListStr(FALSE);
  RosTestTrue("action topics are hidden",RosStrFind(s,"_action")<0);
  Free(s);
  s=RosServiceListStr(FALSE);
  RosTestTrue("action services are hidden",RosStrFind(s,"_action")<0);
  Free(s);
  s=RosActionInfoStr("/fibonacci");
  RosTestStr("action info",s,"Action: /fibonacci\nAction clients: 0\nAction servers: 1\n    /fib_cli_server [example_interfaces/action/Fibonacci]\n");
  Free(s);
  ros_cli_assist=e;
  s=RosActionSendGoalStr("/fibonacci","example_interfaces/action/Fibonacci","{order: 2}",TRUE,
	5*ROS_NS_PER_SEC);
  RosTestTrue("send_goal: accepted",RosStrFind(s,"Goal accepted with ID: ")>=0);
  RosTestTrue("send_goal: goal shown",RosStrStartsWith(s,"Waiting for an action server to become available...\nSending goal:\n     order: 2\n\n"));
  RosTestTrue("send_goal: feedback",RosStrFind(s,"Feedback:\n    sequence:\n    - 0\n    - 1\n\n")>=0);
  RosTestTrue("send_goal: result",RosStrFind(s,"Result:\n    sequence:\n    - 0\n    - 1\n    - 1\n\nGoal finished with status: SUCCEEDED\n")>=0);
  Free(s);
  s=RosActionSendGoalStr("/fibonacci","example_interfaces/action/Fibonacci","{order: 99}",FALSE,
	5*ROS_NS_PER_SEC);
  RosTestTrue("send_goal: rejected",RosStrFind(s,"Goal was rejected.\n")>=0);
  Free(s);
  s=RosActionSendGoalStr("/nobody","example_interfaces/action/Fibonacci","{order: 1}",FALSE,
	ROS_NS_PER_SEC);
  RosTestTrue("send_goal: no server",RosStrFind(s,"Action server not available\n")>=0);
  Free(s);
  ros_cli_assist=NULL;
  s=RosInterfaceShowStr("example_interfaces/action/Fibonacci");
  RosTestStr("interface show",s,"# Goal\nint32 order\n---\n# Result\nint32[] sequence\n---\n# Feedback\nint32[] sequence\n");
  Free(s);
  s=RosInterfaceListStr();
  RosTestTrue("interface list: actions",RosStrFind(s,"Actions:\n    example_interfaces/action/Fibonacci\n    turtlesim/action/RotateAbsolute\n")>=0);
  RosTestTrue("interface list: no action parts in messages",RosStrFind(s,"Fibonacci_Goal")<0);
  Free(s);
  RosTimerDel(f.t);
  RosActionServerDel(as);
  RosNodeDel(sn);
  RosExecDel(e);
}

class CTestRot
{
  I64 feedbacks,status;
  F64 delta;
  Bool done;
};

U0 TestRotFeedback(CRosGoalHandle *gh,CAct_turtlesim_RotateAbsolute_Feedback *fb,CTestRot *r)
{
  r->feedbacks++;
}

U0 TestRotResult(CRosGoalHandle *gh,I64 status,CAct_turtlesim_RotateAbsolute_Result *res,
	CTestRot *r)
{
  r->done=TRUE;
  r->status=status;
  if (res)
    r->delta=res->delta;
}

U0 TestActionTurtle()
{
  CRosNode *sn=RosNodeNew("turtlesim",NULL),*cn=RosNodeNew("rot_client",NULL);
  CRosExec *e=RosExecNew();
  CTurtleSim *ts;
  CTurtle *t;
  CRosActionClient *ac;
  CAct_turtlesim_RotateAbsolute_Goal goal;
  CMsg_geometry_msgs_Twist tw;
  CTestRot r;
  I64 i;
  RosTestGroup("Turtlesim rotate_absolute");
  RosExecAdd(e,sn);
  RosExecAdd(e,cn);
  ts=TurtleSimNew(sn);
  t=TurtleFind(ts,"turtle1");
  ac=RosActionClientNew(cn,"/turtle1/rotate_absolute",ros_act_turtlesim_RotateAbsolute);
  RosTestTrue("server there",RosActionServerReady(ac));
  MemSet(&r,0,sizeof(r));
  goal.theta=pi/2.0;
  RosActionSendGoal(ac,&goal,&TestRotFeedback,&TestRotResult,&r);
  for (i=0;i<400 && !r.done;i++) {
    RosExecSpinSome(e);
    TurtleStep(ts,0.016);
  }
  RosExecSpinSome(e);
  RosTestTrue("done",r.done);
  RosTestI64("succeeded",r.status,ROS_GOAL_SUCCEEDED);
  RosTestTrue("turned there",Abs(t->theta-pi/2.0)<0.03);
  RosTestTrue("delta",Abs(r.delta+pi/2.0)<0.03);
  RosTestTrue("feedback while turning",r.feedbacks>50);
  //a velocity command aborts a running goal
  MemSet(&r,0,sizeof(r));
  goal.theta=-pi/2.0;
  RosActionSendGoal(ac,&goal,NULL,&TestRotResult,&r);
  for (i=0;i<10;i++) {
    RosExecSpinSome(e);
    TurtleStep(ts,0.016);
  }
  RosTestTrue("rotating",t->rot_goal!=NULL);
  MemSet(&tw,0,sizeof(tw));
  TurtleOnCmd(&tw,t);
  for (i=0;i<10 && !r.done;i++)
    RosExecSpinSome(e);
  RosTestI64("aborted by cmd_vel",r.status,ROS_GOAL_ABORTED);
  RosActionClientDel(ac);
  TurtleSimDel(ts);
  RosNodeDel(sn);
  RosNodeDel(cn);
  RosExecDel(e);
}

U0 TestAction()
{
  TestActionBasics();
  TestActionCancelPolicy();
  TestActionCli();
  TestActionTurtle();
}
