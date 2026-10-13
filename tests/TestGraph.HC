//Unit tests for src/Graph and Core/Yaml (single task: everything is
//driven by spinning executors by hand).

class CTestRx
{
  I64 count;
  I64 last;
  U8 *last_ptr;
  U8 *text;
};

U0 TestOnInt(CMsg_std_msgs_Int64 *m,CTestRx *rx)
{
  rx->count++;
  rx->last=m->data;
  rx->last_ptr=m;
}

U0 TestOnStr(CMsg_std_msgs_String *m,CTestRx *rx)
{
  rx->count++;
  Free(rx->text);
  rx->text=StrNew(m->data);
}

U0 TestOnLog(CMsg_rcl_interfaces_Log *m,CTestRx *rx)
{
  rx->count++;
  rx->last=m->level;
  Free(rx->text);
  rx->text=StrNew(m->msg);
}

U0 TestNames()
{
  U8 *s;
  CRosNode *n;
  RosTestGroup("Names");
  s=RosNameExpand("chatter","talker","/");
  RosTestStr("relative in /",s,"/chatter");
  Free(s);
  s=RosNameExpand("chatter","talker","/robot1");
  RosTestStr("relative in ns",s,"/robot1/chatter");
  Free(s);
  s=RosNameExpand("/abs/x","talker","/robot1");
  RosTestStr("absolute",s,"/abs/x");
  Free(s);
  s=RosNameExpand("~/state","talker","/robot1");
  RosTestStr("private",s,"/robot1/talker/state");
  Free(s);
  s=RosNameExpand("~","talker","/");
  RosTestStr("private bare",s,"/talker");
  Free(s);
  RosTestTrue("bad: digit start",!RosNameExpand("1abc","n","/"));
  RosTestTrue("bad: double slash",!RosNameExpand("/a//b","n","/"));
  RosTestTrue("bad: char",!RosNameExpand("a-b","n","/"));
  RosTestTrue("bad: trailing /",!RosNameExpand("a/","n","/"));
  RosTestTrue("bad: ~x",!RosNameExpand("~x","n","/"));
  RosTestTrue("node name ok",RosNodeNameOk("my_node2"));
  RosTestTrue("node name bad",!RosNodeNameOk("my/node") && !RosNodeNameOk("2node"));
  ros_node_builtins=FALSE;
  n=RosNodeNew("n1","ns");
  RosTestStr("ns gets slash",n->fqn,"/ns/n1");
  RosNodeRemap(n,"chatter","/news");
  s=RosNodeResolve(n,"chatter");
  RosTestStr("node remap",s,"/news");
  Free(s);
  s=RosNodeResolve(n,"other");
  RosTestStr("not remapped",s,"/ns/other");
  Free(s);
  RosRemapGlobal("other","/global_other");
  s=RosNodeResolve(n,"other");
  RosTestStr("global remap",s,"/global_other");
  Free(s);
  RosStrVecDel(ros_global_remaps);
  ros_global_remaps=NULL;
  RosNodeDel(n);
  RosTestTrue("bad node rejected",!RosNodeNew("bad-name",NULL));
  ros_node_builtins=TRUE;
}

U0 TestYaml()
{
  CRosVec *v;
  CRosYamlItem *it;
  RosTestGroup("Yaml");
  v=RosYamlFlatten("# params\n/robot/talker:\n  ros__parameters:\n    rate: 2.0 # hz\n    name: 'a # b'\n    nested:\n      deep: [1, 2]\n    list:\n      - x\n      - 'y'\nother:\n  k: v\n");
  RosTestTrue("parsed",v!=NULL);
  if (!v)
    return;
  RosTestI64("items",v->len,5);
  it=RosVecGetI64(v,0);
  RosTestStr("path 0",it->path,"/robot/talker.ros__parameters.rate");
  RosTestStr("val 0",it->val,"2.0");
  it=RosVecGetI64(v,1);
  RosTestStr("quoted # kept",it->val,"'a # b'");
  it=RosVecGetI64(v,2);
  RosTestStr("nested path",it->path,"/robot/talker.ros__parameters.nested.deep");
  it=RosVecGetI64(v,3);
  RosTestStr("block list path",it->path,"/robot/talker.ros__parameters.list");
  RosTestStr("block list",it->val,"[x, 'y']");
  it=RosVecGetI64(v,4);
  RosTestStr("back out",it->path,"other.k");
  RosYamlFree(v);
  RosTestTrue("tabs refused",!RosYamlFlatten("a:\n\tb: 1\n"));
  RosTestTrue("junk refused",!RosYamlFlatten("just text\n"));
}

U0 TestPubSub()
{
  CRosNode *a,*b;
  CRosExec *e=RosExecNew();
  CRosPub *p,*pbe;
  CRosSub *s1,*s2,*s3,*sp;
  CRosQoS q;
  CTestRx rx1,rx2,rx3;
  CMsg_std_msgs_Int64 *m,msg;
  I64 i;
  U8 *taken;
  RosTestGroup("PubSub");
  MemSet(&rx1,0,sizeof(CTestRx));
  MemSet(&rx2,0,sizeof(CTestRx));
  MemSet(&rx3,0,sizeof(CTestRx));
  a=RosNodeNew("pub_node",NULL);
  b=RosNodeNew("sub_node",NULL);
  RosExecAdd(e,a);
  RosExecAdd(e,b);
  p=RosPubNew(a,"numbers",ros_t_std_msgs_Int64,NULL);
  s1=RosSubNew(b,"/numbers",ros_t_std_msgs_Int64,NULL,&TestOnInt,&rx1);
  s2=RosSubNew(b,"numbers",ros_t_std_msgs_Int64,NULL,&TestOnInt,&rx2);
  RosTestI64("matched subs",RosPubSubCount(p),2);
  RosTestI64("matched pubs",RosSubPubCount(s1),1);
  RosTestTrue("wrong type refused",!RosSubNew(b,"numbers",ros_t_std_msgs_String,NULL,&TestOnStr,&rx3));
  m=RosPubLoan(p);
  m->data=41;
  RosPublishLoaned(p,m);
  RosTestI64("queued, not run yet",rx1.count,0);
  RosTestI64("spin ran 2 callbacks",RosExecSpinSome(e),2);
  RosTestI64("sub1 got it",rx1.last,41);
  RosTestI64("sub2 got it",rx2.last,41);
  taken=m;
  RosTestTrue("zero copy: same message",rx1.last_ptr==taken && rx2.last_ptr==taken);
  msg.data=7;
  RosPublish(p,&msg);
  msg.data=8;		//publish copied: this must not leak through
  RosExecSpinSome(e);
  RosTestI64("publish copies",rx1.last,7);
  //keep last 10: 15 published, 10 delivered, newest last
  for (i=0;i<15;i++) {
    msg.data=100+i;
    RosPublish(p,&msg);
  }
  rx1.count=0;
  RosExecSpinSome(e);
  RosTestI64("keep last 10",rx1.count,10);
  RosTestI64("newest kept",rx1.last,114);
  RosTestI64("dropped counted",s1->dropped,5);
  //polling without a callback
  sp=RosSubNew(b,"numbers",ros_t_std_msgs_Int64,NULL,NULL,NULL);
  msg.data=5;
  RosPublish(p,&msg);
  taken=RosSubTake(sp);
  RosTestTrue("take",taken!=NULL);
  if (taken) {
    m=taken;
    RosTestI64("take value",m->data,5);
    RosMsgRelease(taken);
  }
  RosTestTrue("take empty",RosSubTake(sp)==NULL);
  RosSubDel(sp);
  RosExecSpinSome(e);

  //QoS: best effort publisher can't feed a reliable subscription
  RosQoSSensorData(&q);
  pbe=RosPubNew(a,"numbers",ros_t_std_msgs_Int64,&q);
  RosTestI64("best effort matches nobody reliable",RosPubSubCount(pbe),0);
  RosTestTrue("incompatible counted",s1->incompatible==1 && pbe->incompatible==2);
  rx1.count=0;
  RosPublish(pbe,&msg);
  RosExecSpinSome(e);
  RosTestI64("nothing delivered",rx1.count,0);
  s3=RosSubNew(b,"numbers",ros_t_std_msgs_Int64,&q,&TestOnInt,&rx3);
  RosPublish(pbe,&msg);
  RosExecSpinSome(e);
  RosTestI64("best effort sub gets it",rx3.count,1);
  RosPubDel(pbe);
  RosSubDel(s3);

  //transient local: late joiner gets the last depth messages
  RosQoSTransientLocal(&q,3);
  RosPubDel(p);
  p=RosPubNew(a,"latched",ros_t_std_msgs_Int64,&q);
  for (i=1;i<=5;i++) {
    msg.data=i;
    RosPublish(p,&msg);
  }
  MemSet(&rx3,0,sizeof(CTestRx));
  s3=RosSubNew(b,"latched",ros_t_std_msgs_Int64,&q,&TestOnInt,&rx3);
  RosExecSpinSome(e);
  RosTestI64("late joiner got history",rx3.count,3);
  RosTestI64("history newest",rx3.last,5);
  RosSubDel(s3);
  MemSet(&rx3,0,sizeof(CTestRx));
  s3=RosSubNew(b,"latched",ros_t_std_msgs_Int64,NULL,&TestOnInt,&rx3);
  RosExecSpinSome(e);
  RosTestI64("volatile late joiner gets nothing",rx3.count,0);
  RosSubDel(s3);

  //lifespan: old messages are dropped at take time
  RosQoSDefault(&q);
  q.lifespan_ns=ROS_NS_PER_MS;
  s3=RosSubNew(b,"latched",ros_t_std_msgs_Int64,&q,&TestOnInt,&rx3);
  RosPublish(p,&msg);
  RosSleepNs(3*ROS_NS_PER_MS);
  RosExecSpinSome(e);
  RosTestI64("expired",s3->expired,1);
  RosTestI64("not delivered",rx3.count,0);

  RosNodeDel(a);
  RosNodeDel(b);
  RosTestTrue("topics cleaned up",!RosMapHas(ros_graph->topics,"/numbers") &&
	!RosMapHas(ros_graph->topics,"/latched"));
  RosExecDel(e);
}

U0 TestTick(CTestRx *rx)
{
  rx->count++;
}

U0 TestRepublish(CMsg_std_msgs_Int64 *m,CRosPub *p)
{
  CMsg_std_msgs_Int64 out;
  out.data=m->data+1;
  if (out.data<5)
    RosPublish(p,&out);
}

U0 TestTimersExec()
{
  CRosNode *n=RosNodeNew("timer_node",NULL);
  CRosExec *e=RosNodeExec(n);
  CRosTimer *t;
  CRosPub *p;
  CTestRx rx;
  CMsg_std_msgs_Int64 msg;
  I64 rounds=0;
  RosTestGroup("Timers/Exec");
  MemSet(&rx,0,sizeof(CTestRx));
  t=RosTimerNew(n,5*ROS_NS_PER_MS,&TestTick,&rx);
  RosExecSpinFor(e,32*ROS_NS_PER_MS);
  RosTestTrue("~6 ticks in 32ms",rx.count>=4 && rx.count<=7);
  RosTimerCancel(t);
  rx.count=0;
  RosExecSpinFor(e,12*ROS_NS_PER_MS);
  RosTestI64("canceled timer quiet",rx.count,0);
  //a callback that publishes: each message handled on a later spin
  p=RosPubNew(n,"loop",ros_t_std_msgs_Int64,NULL);
  RosSubNew(n,"loop",ros_t_std_msgs_Int64,NULL,&TestRepublish,p);
  msg.data=0;
  RosPublish(p,&msg);
  while (RosExecSpinSome(e))
    rounds++;
  RosTestI64("chain of 5 took 5 spins",rounds,5);
  RosTestTrue("spin once times out",!RosExecSpinOnce(e,2*ROS_NS_PER_MS));
  RosNodeDel(n);
  RosExecDel(e);
}

U0 TestAdd(CSrv_example_interfaces_AddTwoInts_Request *req,
	CSrv_example_interfaces_AddTwoInts_Response *resp,U8 *user)
{
  resp->sum=req->a+req->b;
}

U0 TestOnSum(CSrv_example_interfaces_AddTwoInts_Response *resp,I64 seq,CTestRx *rx)
{
  rx->count++;
  rx->last=resp->sum;
}

U0 TestServices()
{
  CRosNode *srv_node,*cli_node;
  CRosExec *e=RosExecNew();
  CRosClient *c,*c2;
  CSrv_example_interfaces_AddTwoInts_Request req;
  CSrv_example_interfaces_AddTwoInts_Response *resp;
  CTestRx rx;
  I64 seq;
  RosTestGroup("Services");
  MemSet(&rx,0,sizeof(CTestRx));
  ros_node_builtins=FALSE;
  srv_node=RosNodeNew("adder",NULL);
  cli_node=RosNodeNew("asker",NULL);
  RosExecAdd(e,srv_node);
  RosExecAdd(e,cli_node);
  c=RosClientNew(cli_node,"add_two_ints",ros_srv_example_interfaces_AddTwoInts);
  RosTestTrue("not ready yet",!RosClientReady(c));
  req.a=2;
  req.b=3;
  RosTestI64("call without service",RosClientCall(c,&req,&TestOnSum,&rx),-1);
  RosTestTrue("service",RosServiceNew(srv_node,"add_two_ints",
	ros_srv_example_interfaces_AddTwoInts,&TestAdd,NULL)!=NULL);
  RosTestTrue("duplicate service refused",!RosServiceNew(srv_node,"/add_two_ints",
	ros_srv_example_interfaces_AddTwoInts,&TestAdd,NULL));
  RosTestTrue("ready",RosClientReady(c));
  seq=RosClientCall(c,&req,&TestOnSum,&rx);
  RosTestI64("first seq",seq,1);
  RosExecSpinSome(e);
  RosExecSpinSome(e);
  RosTestI64("async answer",rx.last,5);
  req.a=-10;
  req.b=4;
  resp=RosClientCallWait(c,e,&req,ROS_NS_PER_SEC);
  RosTestTrue("sync answer",resp && resp->sum==-6);
  RosMsgDel(ros_t_example_interfaces_AddTwoInts_Response,resp);
  //a client that goes away while its request is queued
  c2=RosClientNew(cli_node,"add_two_ints",ros_srv_example_interfaces_AddTwoInts);
  RosClientCall(c2,&req,&TestOnSum,&rx);
  RosClientDel(c2);
  RosExecSpinSome(e);
  RosTestTrue("survived orphan request",TRUE);
  RosTestTrue("wrong type not ready",
	!RosClientReady(RosClientNew(cli_node,"add_two_ints",ros_srv_std_srvs_Trigger)));
  RosNodeDel(srv_node);
  RosNodeDel(cli_node);
  RosExecDel(e);
  ros_node_builtins=TRUE;
}

Bool TestParamGuard(CRosNode *n,U8 *name,CMsg_rcl_interfaces_ParameterValue *v,U8 *user)
{
  if (!StrCmp(name,"rate") && v->double_value>100.0)
    return FALSE;
  return TRUE;
}

U0 TestOnEvent(CMsg_rcl_interfaces_ParameterEvent *ev,CTestRx *rx)
{
  CMsg_rcl_interfaces_Parameter *ps;
  rx->count++;
  if (ev->changed_parameters.len) {
    ps=ev->changed_parameters.data;
    Free(rx->text);
    rx->text=StrNew(ps[0].name);
  }
}

U0 TestParams()
{
  CRosNode *n,*tool;
  CRosExec *e=RosExecNew();
  CRosClient *c;
  CTestRx rx;
  U8 *s,**names;
  CSrv_rcl_interfaces_GetParameters_Request *greq;
  CSrv_rcl_interfaces_GetParameters_Response *gresp;
  CSrv_rcl_interfaces_SetParameters_Request *sreq;
  CSrv_rcl_interfaces_SetParameters_Response *sresp;
  CSrv_rcl_interfaces_ListParameters_Request *lreq;
  CSrv_rcl_interfaces_ListParameters_Response *lresp;
  CMsg_rcl_interfaces_ParameterValue *vals;
  CMsg_rcl_interfaces_Parameter *p;
  CMsg_rcl_interfaces_SetParametersResult *res;
  CRosType *t;
  RosTestGroup("Params");
  MemSet(&rx,0,sizeof(CTestRx));
  RosTestTrue("yaml",RosParamLoadYaml(
	"/robot/driver:\n  ros__parameters:\n    rate: 50\n    wheels: [0.1, 0.2]\n/**:\n  ros__parameters:\n    use_sim_time: false\nother:\n  ros__parameters:\n    rate: 1.0\n"));
  n=RosNodeNew("driver","robot");
  tool=RosNodeNew("param_tool",NULL);
  RosExecAdd(e,n);
  RosExecAdd(e,tool);
  RosSubNew(tool,"/parameter_events",ros_t_rcl_interfaces_ParameterEvent,NULL,&TestOnEvent,&rx);
  RosTestTrue("declare double",RosParamDeclare(n,"rate","10.0")!=NULL);
  RosTestF64("override (int) wins, as double",RosParamF64(n,"rate"),50.0,0.0);
  RosTestI64("type double",RosParamType(n,"rate"),ROS_PT_DOUBLE);
  RosTestTrue("declare wildcard",RosParamDeclare(n,"use_sim_time","true")!=NULL);
  RosTestTrue("wildcard override",!RosParamBool(n,"use_sim_time"));
  RosParamDeclare(n,"wheels","[0.0, 0.0]");
  s=RosParamText(n,"wheels");
  RosTestStr("array override",s,"[0.1, 0.2]");
  Free(s);
  RosParamDeclare(n,"name","'bot'");
  RosTestStr("string",RosParamStr(n,"name"),"bot");
  RosParamDeclare(n,"count","3");
  RosTestI64("int",RosParamI64(n,"count"),3);
  RosParamDeclare(n,"flags","[true, false]");
  RosTestI64("bool array type",RosParamType(n,"flags"),ROS_PT_BOOL_ARRAY);
  RosParamDeclare(n,"words","[a, 'b c']");
  s=RosParamText(n,"words");
  RosTestStr("string array",s,"['a', 'b c']");
  Free(s);
  RosParamDeclareEx(n,"serial","'X1'","board serial number",TRUE);
  RosTestTrue("redeclare refused",!RosParamDeclare(n,"rate","1.0"));
  RosTestTrue("set ok",RosParamSet(n,"rate","20.5"));
  RosTestF64("set value",RosParamF64(n,"rate"),20.5,0.0);
  RosTestTrue("int into double ok",RosParamSet(n,"rate","30"));
  RosTestTrue("wrong type refused",!RosParamSet(n,"rate","'fast'"));
  RosTestTrue("read only refused",!RosParamSet(n,"serial","'X2'"));
  RosTestTrue("undeclared refused",!RosParamSet(n,"nope","1"));
  RosParamOnSet(n,&TestParamGuard,NULL);
  RosTestTrue("callback refuses",!RosParamSet(n,"rate","500.0"));
  RosTestTrue("why",RosStrFind(ros_param_error,"rejected")>=0);
  RosTestF64("unchanged",RosParamF64(n,"rate"),30.0,0.0);
  RosParamDeclare(n,"anything",NULL);
  RosTestTrue("dynamic typing",RosParamSet(n,"anything","1") && RosParamSet(n,"anything","'x'"));
  RosTestTrue("undeclare",RosParamUndeclare(n,"anything") && !RosParamHas(n,"anything"));
  RosExecSpinSome(e);
  RosTestTrue("events seen",rx.count>=10);
  RosTestStr("last change event",rx.text,"anything");

  //the same things through the parameter services, like ros2 param does
  t=ros_t_rcl_interfaces_GetParameters_Request;
  c=RosClientNew(tool,"/robot/driver/get_parameters",ros_srv_rcl_interfaces_GetParameters);
  RosTestTrue("get_parameters service exists",RosClientReady(c));
  greq=RosMsgNew(t);
  RosMsgSet(t,greq,"names","[rate, name, missing]");
  gresp=RosClientCallWait(c,e,greq,ROS_NS_PER_SEC);
  RosTestTrue("get answered",gresp && gresp->values.len==3);
  if (gresp) {
    vals=gresp->values.data;
    RosTestF64("get rate",vals[0].double_value,30.0,0.0);
    RosTestStr("get name",vals[1].string_value,"bot");
    RosTestI64("missing is not set",vals[2].type,ROS_PT_NOT_SET);
    RosMsgDel(ros_t_rcl_interfaces_GetParameters_Response,gresp);
  }
  RosMsgDel(t,greq);

  t=ros_t_rcl_interfaces_SetParameters_Request;
  c=RosClientNew(tool,"/robot/driver/set_parameters",ros_srv_rcl_interfaces_SetParameters);
  sreq=RosMsgNew(t);
  RosMsgSet(t,sreq,"parameters[0].name","count");
  RosMsgSet(t,sreq,"parameters[0].value.type","2");
  RosMsgSet(t,sreq,"parameters[0].value.integer_value","9");
  RosMsgSet(t,sreq,"parameters[1].name","serial");
  RosMsgSet(t,sreq,"parameters[1].value.type","4");
  RosMsgSet(t,sreq,"parameters[1].value.string_value","hack");
  sresp=RosClientCallWait(c,e,sreq,ROS_NS_PER_SEC);
  RosTestTrue("set answered",sresp && sresp->results.len==2);
  if (sresp) {
    res=sresp->results.data;
    RosTestTrue("set count ok",res[0].successful);
    RosTestTrue("set serial refused",!res[1].successful && RosStrFind(res[1].reason,"read only")>=0);
    RosMsgDel(ros_t_rcl_interfaces_SetParameters_Response,sresp);
  }
  RosMsgDel(t,sreq);
  RosTestI64("count changed",RosParamI64(n,"count"),9);

  t=ros_t_rcl_interfaces_ListParameters_Request;
  c=RosClientNew(tool,"/robot/driver/list_parameters",ros_srv_rcl_interfaces_ListParameters);
  lreq=RosMsgNew(t);
  lresp=RosClientCallWait(c,e,lreq,ROS_NS_PER_SEC);
  RosTestTrue("list answered",lresp!=NULL);
  if (lresp) {
    RosTestI64("list count",lresp->result.names.len,8);
    names=lresp->result.names.data;
    RosTestStr("list order",names[0],"rate");
    RosMsgDel(ros_t_rcl_interfaces_ListParameters_Response,lresp);
  }
  RosMsgDel(t,lreq);

  //rosout
  Free(rx.text);
  MemSet(&rx,0,sizeof(CTestRx));
  ros_log_quiet=TRUE;
  RosSubNew(tool,"/rosout",ros_t_rcl_interfaces_Log,NULL,&TestOnLog,&rx);
  RosNodeLog(n,ROS_LOG_WARN,"hello rosout");
  RosNodeLog(n,ROS_LOG_DEBUG,"filtered out");
  ros_log_quiet=FALSE;
  RosExecSpinSome(e);
  RosTestI64("rosout got one",rx.count,1);
  RosTestI64("rosout level WARN=30",rx.last,30);
  RosTestStr("rosout text",rx.text,"hello rosout");

  RosNodeDel(n);
  RosNodeDel(tool);
  RosExecDel(e);
  Free(rx.text);
  RosTestI64("graph empty again",ros_graph->nodes->len,0);
  RosTestI64("no topics left",ros_graph->topics->count,0);
  RosTestI64("no services left",ros_graph->services->count,0);
}

U0 TestGraph()
{
  TestNames();
  TestYaml();
  TestPubSub();
  TestTimersExec();
  TestServices();
  TestParams();
}
