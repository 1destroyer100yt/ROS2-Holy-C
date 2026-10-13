//Tests for src/Tools: the CLI functions, launch, turtlesim.
//Target nodes run in ros_cli_assist, which the CLI spins while it
//waits (everything is one task here).

U0 TestYamlFlow()
{
  CRosVec *v;
  CRosYamlItem *it;
  RosTestGroup("Yaml flow");
  v=RosYamlFlowFlatten("{linear: {x: 0.5, y: -1}, name: 'a, b', v: [1, 2], s: {}}");
  RosTestTrue("parsed",v!=NULL);
  if (v) {
    RosTestI64("items",v->len,4);
    it=RosVecGetI64(v,0);
    RosTestTrue("nested",!StrCmp(it->path,"linear.x") && !StrCmp(it->val,"0.5"));
    it=RosVecGetI64(v,2);
    RosTestTrue("quoted comma",!StrCmp(it->path,"name") && !StrCmp(it->val,"'a, b'"));
    it=RosVecGetI64(v,3);
    RosTestTrue("list",!StrCmp(it->path,"v") && !StrCmp(it->val,"[1, 2]"));
    RosYamlFree(v);
  }
  v=RosYamlFlowFlatten("linear.x=0.5 name='a b' v=[1, 2]");
  RosTestTrue("key=value form",v && v->len==3);
  if (v) {
    it=RosVecGetI64(v,1);
    RosTestStr("quoted value",it->val,"'a b'");
    it=RosVecGetI64(v,2);
    RosTestStr("list value",it->val,"[1, 2]");
    RosYamlFree(v);
  }
  RosTestTrue("bad",!RosYamlFlowFlatten("{a: 1"));
}

U0 TestCliCount(CMsg_std_msgs_String *m,CTestRx *rx)
{
  rx->count++;
  Free(rx->text);
  rx->text=StrNew(m->data);
}

U0 TestAddSrv(CSrv_example_interfaces_AddTwoInts_Request *req,
	CSrv_example_interfaces_AddTwoInts_Response *resp,U8 *user)
{
  resp->sum=req->a+req->b;
}

U0 TestCli()
{
  CRosNode *a,*b;
  CRosPub *p;
  CTestRx rx;
  U8 *s;
  CMsg_std_msgs_String msg;
  I64 n;
  RosTestGroup("CLI");
  MemSet(&rx,0,sizeof(CTestRx));
  ros_cli_assist=RosExecNew();
  a=RosNodeNew("talker","/demo");
  b=RosNodeNew("listener",NULL);
  RosExecAdd(ros_cli_assist,a);
  RosExecAdd(ros_cli_assist,b);
  p=RosPubNew(a,"chatter",ros_t_std_msgs_String,NULL);
  RosSubNew(b,"/demo/chatter",ros_t_std_msgs_String,NULL,&TestCliCount,&rx);
  RosServiceNew(b,"add_two_ints",ros_srv_example_interfaces_AddTwoInts,&TestAddSrv,NULL);
  RosParamDeclare(a,"rate","2.0");
  RosParamDeclare(a,"name","'chatty'");
  RosParamDeclare(a,"ids","[1, 2]");

  s=RosNodeListStr();
  RosTestStr("node list",s,"/demo/talker\n/listener\n");
  Free(s);
  s=RosNodeInfoStr("/demo/talker");
  RosTestTrue("node info pubs",RosStrFind(s,"  Publishers:\n    /demo/chatter: std_msgs/msg/String\n")>=0);
  RosTestTrue("node info services",
	RosStrFind(s,"/demo/talker/get_parameters: rcl_interfaces/srv/GetParameters")>=0);
  Free(s);
  s=RosNodeInfoStr("/nope");
  RosTestTrue("node info unknown",RosStrStartsWith(s,"Unable to find node"));
  Free(s);
  s=RosTopicListStr(TRUE);
  RosTestTrue("topic list -t",RosStrFind(s,"/demo/chatter [std_msgs/msg/String]\n")>=0 &&
	RosStrFind(s,"/rosout [rcl_interfaces/msg/Log]\n")>=0);
  Free(s);
  s=RosTopicInfoStr("/demo/chatter",FALSE);
  RosTestStr("topic info",s,"Type: std_msgs/msg/String\nPublisher count: 1\nSubscription count: 1\n");
  Free(s);
  s=RosTopicInfoStr("/demo/chatter",TRUE);
  RosTestTrue("topic info -v",RosStrFind(s,"Node name: /listener\nEndpoint type: SUBSCRIPTION")>=0);
  Free(s);

  //ros2 topic pub (3 times at 100 Hz) reaches the listener
  n=RosTopicPubN("/demo/chatter","std_msgs/msg/String","{data: 'hello cli'}",100.0,3);
  RosExecSpinSome(ros_cli_assist);
  RosTestI64("pub sent",n,3);
  RosTestI64("listener got them",rx.count,3);
  RosTestStr("listener text",rx.text,"hello cli");
  RosTestI64("pub bad type",RosTopicPubN("/demo/chatter","nope/msg/X","{}",1.0,1),0);

  //ros2 topic echo: gets what the talker publishes
  msg.data="echo me";
  RosPublish(p,&msg);	//before echo subscribes: not seen (volatile)
  RosTestI64("echo times out with nothing",RosTopicEchoN("/demo/chatter",NULL,1,20*ROS_NS_PER_MS),0);

  s=RosServiceListStr(TRUE);
  RosTestTrue("service list -t",RosStrFind(s,"/add_two_ints [example_interfaces/srv/AddTwoInts]\n")>=0);
  Free(s);
  s=RosServiceCallStr("/add_two_ints","example_interfaces/srv/AddTwoInts","{a: 40, b: 2}");
  RosTestTrue("service call",RosStrFind(s,"response:\nsum: 42\n")>=0);
  Free(s);
  s=RosServiceCallStr("/nobody","example_interfaces/srv/AddTwoInts","{a: 1}");
  RosTestTrue("service missing",RosStrFind(s,"service not available")>=0);
  Free(s);

  s=RosParamListStr("/demo/talker");
  RosTestStr("param list",s,"  ids\n  name\n  rate\n");
  Free(s);
  s=RosParamGetStr("/demo/talker","rate");
  RosTestStr("param get",s,"Double value is: 2.0\n");
  Free(s);
  s=RosParamGetStr("/demo/talker","name");
  RosTestStr("param get string",s,"String value is: chatty\n");
  Free(s);
  s=RosParamSetStr("/demo/talker","rate","4.5");
  RosTestStr("param set",s,"Set parameter successful\n");
  Free(s);
  RosTestF64("really set",RosParamF64(a,"rate"),4.5,0.0);
  s=RosParamSetStr("/demo/talker","rate","'x'");
  RosTestTrue("param set wrong type",RosStrStartsWith(s,"Setting parameter failed"));
  Free(s);
  s=RosParamDumpStr("/demo/talker");
  RosTestStr("param dump",s,"/demo/talker:\n  ros__parameters:\n    ids: [1, 2]\n    name: 'chatty'\n    rate: 4.5\n");
  Free(s);
  s=RosParamGetStr("/nobody","rate");
  RosTestTrue("param get unknown node",RosStrStartsWith(s,"Node not found"));
  Free(s);

  s=RosInterfaceShowStr("example_interfaces/srv/AddTwoInts");
  RosTestStr("interface show srv",s,"int64 a\nint64 b\n---\nint64 sum\n");
  Free(s);
  s=RosInterfaceProtoStr("geometry_msgs/Vector3");
  RosTestStr("interface proto",s,"x: 0.0\ny: 0.0\nz: 0.0\n");
  Free(s);
  s=RosInterfaceListStr();
  RosTestTrue("interface list",RosStrFind(s,"    sensor_msgs/msg/LaserScan\n")>=0 &&
	RosStrFind(s,"Services:\n")>=0 && RosStrFind(s,"    std_srvs/srv/Trigger\n")>=0);
  Free(s);

  RosNodeDel(a);
  RosNodeDel(b);
  RosExecDel(ros_cli_assist);
  ros_cli_assist=NULL;
  Free(rx.text);
  s=RosNodeListStr();
  RosTestStr("cli nodes cleaned up",s,"");
  Free(s);
}

U0 TestHzPub(CRosPub *p)
{
  CMsg_std_msgs_String m;
  m.data="tick";
  RosPublish(p,&m);
}

U0 TestHz()
{
  CRosNode *a;
  U8 *s,*num;
  F64 rate;
  I64 i;
  RosTestGroup("CLI hz");
  ros_cli_assist=RosExecNew();
  a=RosNodeNew("ticker",NULL);
  RosExecAdd(ros_cli_assist,a);
  RosTimerNew(a,10*ROS_NS_PER_MS,&TestHzPub,RosPubNew(a,"tick",ros_t_std_msgs_String,NULL));
  s=RosTopicHzStr("/tick",300*ROS_NS_PER_MS);
  RosTestTrue("hz report",RosStrStartsWith(s,"average rate: "));
  i=14;
  num=RosStrSub(s,i,RosStrFindC(s+i,'\n'));
  RosStrToF64(num,&rate);
  Free(num);
  RosTestTrue("about 100 Hz",rate>60.0 && rate<140.0);
  Free(s);
  RosNodeDel(a);
  RosExecDel(ros_cli_assist);
  ros_cli_assist=NULL;
}

U0 TestLaunchTalker(CRosNode *n,U8 *arg)
{
  RosParamDeclare(n,"rate","1.0");
  RosPubNew(n,"chatter",ros_t_std_msgs_String,NULL);
}

U0 TestLaunch()
{
  CRosLaunch *l=RosLaunchNew();
  CRosLaunchNode *ln;
  CRosNode *n;
  U8 *s;
  RosTestGroup("Launch");
  ln=RosLaunchAdd(l,"talker","/robot",&TestLaunchTalker,NULL);
  RosLaunchRemap(ln,"chatter","news");
  RosLaunchParam(ln,"rate","7.5");
  RosLaunchAdd(l,"turtlesim",NULL,&TurtleSimSetup,NULL);
  RosLaunchStartHere(l);
  n=RosLaunchNode(l,"talker");
  RosTestTrue("node made",n!=NULL);
  RosTestF64("launch param",RosParamF64(n,"rate"),7.5,0.0);
  s=RosTopicListStr(FALSE);
  RosTestTrue("launch remap",RosStrFind(s,"/robot/news\n")>=0 && RosStrFind(s,"/robot/chatter")<0);
  RosTestTrue("turtlesim up",RosStrFind(s,"/turtle1/pose\n")>=0);
  Free(s);
  RosLaunchSpinFor(l,20*ROS_NS_PER_MS);
  RosLaunchStop(l);
  RosTestTrue("sim freed with its node",turtle_sim==NULL);
  s=RosNodeListStr();
  RosTestStr("all stopped",s,"");
  Free(s);
}

U0 TestOnPose(CMsg_turtlesim_Pose *p,CMsg_turtlesim_Pose *last)
{
  MemCpy(last,p,sizeof(CMsg_turtlesim_Pose));
}

U0 TestTurtlesim()
{
  CRosNode *n,*me;
  CRosExec *e=RosExecNew();
  CTurtleSim *ts;
  CTurtle *t;
  CRosPub *cmd;
  CMsg_geometry_msgs_Twist tw;
  CMsg_turtlesim_Pose pose;
  U8 *s;
  I64 i;
  RosTestGroup("Turtlesim");
  MemSet(&pose,0,sizeof(CMsg_turtlesim_Pose));
  n=RosNodeNew("turtlesim",NULL);
  me=RosNodeNew("driver",NULL);
  RosExecAdd(e,n);
  RosExecAdd(e,me);
  ts=TurtleSimNew(n);
  RosSubNew(me,"/turtle1/pose",ros_t_turtlesim_Pose,NULL,&TestOnPose,&pose);
  cmd=RosPubNew(me,"/turtle1/cmd_vel",ros_t_geometry_msgs_Twist,NULL);
  MemSet(&tw,0,sizeof(CMsg_geometry_msgs_Twist));
  tw.linear.x=1.0;
  for (i=0;i<10;i++) {	//drive forward ~0.5 s at 1 m/s
    RosPublish(cmd,&tw);
    RosExecSpinFor(e,50*ROS_NS_PER_MS);
  }
  RosTestTrue("moved forward ~0.5 m",pose.x>5.8 && pose.x<6.3);
  RosTestF64("y unchanged",pose.y,RosF32Round(TURTLE_START),0.0001);
  RosTestTrue("left a trail",ts->trail->len>5);
  //no commands for > 1 s: it stops
  RosExecSpinFor(e,1100*ROS_NS_PER_MS);
  RosTestF64("stopped after timeout",pose.linear_velocity,0.0,0.0);
  ros_cli_assist=e;
  s=RosServiceCallStr("/spawn","turtlesim/srv/Spawn","{x: 2.0, y: 3.0, theta: 1.57}");
  RosTestTrue("spawn",RosStrFind(s,"name: 'turtle2'")>=0);
  Free(s);
  s=RosServiceCallStr("/spawn","turtlesim/srv/Spawn","{name: turtle2}");
  RosTestTrue("spawn duplicate",RosStrFind(s,"name: ''")>=0);
  Free(s);
  s=RosServiceCallStr("/turtle2/teleport_absolute","turtlesim/srv/TeleportAbsolute",
	"{x: 9.0, y: 9.0, theta: 0.0}");
  Free(s);
  t=TurtleFind(ts,"turtle2");
  RosTestTrue("teleported",t && t->x==9.0 && t->y==9.0);
  s=RosServiceCallStr("/turtle2/set_pen","turtlesim/srv/SetPen","{r: 255, g: 0, b: 0, width: 2, off: 1}");
  Free(s);
  RosTestTrue("pen off",t && t->pen_off && t->r==255);
  s=RosServiceCallStr("/clear","std_srvs/srv/Empty","");
  Free(s);
  RosTestI64("cleared",ts->trail->len,0);
  s=RosServiceCallStr("/kill","turtlesim/srv/Kill","{name: turtle2}");
  Free(s);
  RosTestTrue("killed",!TurtleFind(ts,"turtle2"));
  s=RosTopicListStr(FALSE);
  RosTestTrue("killed turtle's topics gone",RosStrFind(s,"/turtle2/")<0);
  Free(s);
  ros_cli_assist=NULL;
  TurtleSimDel(ts);
  RosNodeDel(n);
  RosNodeDel(me);
  RosExecDel(e);
}

U0 TestTools()
{
  TestYamlFlow();
  TestCli();
  TestHz();
  TestLaunch();
  TestTurtlesim();
}
