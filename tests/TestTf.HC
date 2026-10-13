//Tests for src/Tf.

U0 TestTfMath()
{
  CRosTf a,b,c,inv;
  F64 x,y,z,r,p,yw;
  RosTestGroup("Tf math");
  RosTfIdentity(&a);
  RosTfSetRpy(&a,0.0,0.0,pi/2.0);	//yaw 90 degrees
  x=1.0;
  y=0.0;
  z=0.0;
  RosTfRotate(&a,&x,&y,&z);
  RosTestF64("yaw 90: x->y (x)",x,0.0,1e-12);
  RosTestF64("yaw 90: x->y (y)",y,1.0,1e-12);
  RosTfSetRpy(&a,0.3,-0.2,1.1);
  a.x=1.0;
  a.y=-2.0;
  a.z=0.5;
  RosTfGetRpy(&a,&r,&p,&yw);
  RosTestTrue("rpy round trip",Abs(r-0.3)<1e-12 && Abs(p+0.2)<1e-12 && Abs(yw-1.1)<1e-12);
  RosTfInverse(&inv,&a);
  RosTfCompose(&c,&a,&inv);
  RosTestTrue("a * inv(a) = identity",Abs(c.x)<1e-12 && Abs(c.y)<1e-12 && Abs(c.z)<1e-12 &&
	Abs(Abs(c.qw)-1.0)<1e-12);
  RosTfCompose(&c,&inv,&a);	//aliasing: out is an input
  RosTestTrue("inv(a) * a = identity",Abs(c.x)<1e-12 && Abs(Abs(c.qw)-1.0)<1e-12);
  x=0.0;
  y=0.0;
  z=0.0;
  RosTfApply(&a,&x,&y,&z);
  RosTestTrue("apply moves origin to translation",x==1.0 && y==-2.0 && z==0.5);
  RosTfIdentity(&a);
  RosTfIdentity(&b);
  RosTfSetRpy(&b,0.0,0.0,pi/2.0);
  b.x=2.0;
  RosTfLerp(&c,&a,&b,0.5);
  RosTfGetRpy(&c,&r,&p,&yw);
  RosTestF64("slerp halfway: yaw 45",yw,pi/4.0,1e-12);
  RosTestF64("lerp halfway: x",c.x,1.0,1e-12);
  RosTfSetRpy(&b,0.0,0.0,pi-0.01);	//shortest way, across +-pi
  RosTfSetRpy(&a,0.0,0.0,-pi+0.01);
  RosTfLerp(&c,&a,&b,0.5);
  RosTfGetRpy(&c,&r,&p,&yw);
  RosTestF64("slerp takes the short way",Abs(yw),pi,1e-9);
}

U0 TestTfBuffer()
{
  CRosTfBuffer *tb=RosTfBufferNew();
  CRosTf t,out,back;
  I64 s=ROS_NS_PER_SEC,i;
  U8 *str;
  CRosTfFrame *f;
  RosTestGroup("Tf buffer");
  RosTfIdentity(&t);
  t.x=1.0;
  RosTfSet(tb,"map","odom",0,&t,TRUE);	//static
  RosTfIdentity(&t);
  RosTfSet(tb,"odom","/base_link",1*s,&t,FALSE);	//leading / is ignored
  t.x=2.0;
  RosTfSetRpy(&t,0.0,0.0,pi/2.0);
  RosTfSet(tb,"odom","base_link",3*s,&t,FALSE);
  RosTfIdentity(&t);
  t.x=0.5;
  RosTfSet(tb,"base_link","laser",0,&t,TRUE);
  RosTfIdentity(&t);
  t.y=0.2;
  RosTfSet(tb,"base_link","wheel_left",0,&t,TRUE);
  t.y=-0.2;
  RosTfSet(tb,"base_link","wheel_right",0,&t,TRUE);
  //at t=2s the base is halfway: x=1 (in odom), yaw 45; the laser is
  //0.5 ahead of it
  RosTestTrue("lookup at 2s",RosTfLookup(tb,"map","laser",2*s,&out));
  RosTestF64("map<-laser x",out.x,2.0+0.5*Cos(pi/4.0),1e-9);
  RosTestF64("map<-laser y",out.y,0.5*Sin(pi/4.0),1e-9);
  RosTestTrue("inverse lookup",RosTfLookup(tb,"laser","map",2*s,&back));
  RosTfCompose(&t,&out,&back);
  RosTestTrue("forward * inverse = identity",Abs(t.x)<1e-9 && Abs(t.y)<1e-9 &&
	Abs(Abs(t.qw)-1.0)<1e-9);
  RosTestTrue("latest (0)",RosTfLookup(tb,"map","base_link",0,&out));
  RosTestF64("latest uses 3s",out.x,3.0,1e-9);
  RosTestI64("latest time",RosTfLatestTime(tb,"map","laser"),3*s);
  RosTestTrue("across branches",RosTfLookup(tb,"wheel_left","wheel_right",0,&out));
  RosTestF64("wheel to wheel",out.y,-0.4,1e-9);
  RosTestTrue("future refused",!RosTfLookup(tb,"map","laser",5*s,&out));
  RosTestTrue("says future",RosStrFind(ros_tf_error,"future")>=0);
  RosTestTrue("past refused",!RosTfLookup(tb,"map","laser",s/2,&out));
  RosTestTrue("unknown frame",!RosTfLookup(tb,"map","nope",0,&out) &&
	RosStrFind(ros_tf_error,"does not exist")>=0);
  RosTfIdentity(&t);
  RosTfSet(tb,"world","island",0,&t,TRUE);
  RosTestTrue("not connected",!RosTfLookup(tb,"map","island",0,&out) &&
	RosStrFind(ros_tf_error,"not connected")>=0);
  RosTestTrue("same frame",RosTfLookup(tb,"laser","laser",0,&out) && out.qw==1.0);
  RosTestTrue("root to child",RosTfLookup(tb,"odom","map",0,&out) && out.x==-1.0);
  //history is trimmed to 10 s
  for (i=4;i<=30;i++) {
    t.x=i;
    RosTfSet(tb,"odom","base_link",i*s,&t,FALSE);
  }
  f=RosMapGet(tb->frames,"base_link");
  RosTestTrue("history trimmed",f->hist->len<=12);
  RosTestTrue("old data gone",!RosTfLookup(tb,"odom","base_link",5*s,&out));
  RosTestTrue("recent data",RosTfLookup(tb,"odom","base_link",25*s+s/2,&out) &&
	Abs(out.x-25.5)<1e-9);
  str=RosTfFramesStr(tb);
  RosTestTrue("frames list",RosStrFind(str,"laser <- base_link (static)\n")>=0 &&
	RosStrFind(str,"base_link <- odom\n")>=0);
  Free(str);
  RosTfBufferDel(tb);
}

class CTfPub
{
  CRosTfBroadcaster *br;
  CRosTf t;
};

U0 TestTfTick(CTfPub *tp)	//robots publish /tf continuously
{
  RosTfSend(tp->br,"odom","base_link",RosTimeMake(1700000000,500000000),&tp->t);
}

U0 TestTfRos()
{
  CRosNode *robot,*viewer;
  CRosExec *e=RosExecNew();
  CRosTfBroadcaster *br,*sb;
  CRosTfBuffer *tb=RosTfBufferNew();
  CRosTfListener *l;
  CRosTf t,out;
  CTfPub tp;
  U8 *s;
  RosTestGroup("Tf over ROS");
  robot=RosNodeNew("robot_tf",NULL);
  viewer=RosNodeNew("viewer",NULL);
  RosExecAdd(e,robot);
  RosExecAdd(e,viewer);
  br=RosTfBroadcasterNew(robot);
  sb=RosTfStaticBroadcasterNew(robot);
  RosTfIdentity(&t);
  t.z=0.3;
  RosTfSend(sb,"base_link","lidar",0,&t);	//before anyone listens
  t.z=0.1;
  RosTfSend(sb,"base_link","imu",0,&t);
  l=RosTfListenerNew(viewer,tb);		//late joiner
  RosTfIdentity(&t);
  t.x=1.0;
  t.y=2.0;
  RosTfSend(br,"odom","base_link",RosTimeMake(1700000000,500000000),&t);
  RosExecSpinSome(e);
  RosTestTrue("static reached a late joiner",RosTfLookup(tb,"base_link","lidar",0,&out) &&
	out.z==0.3);
  RosTestTrue("both statics",RosTfLookup(tb,"base_link","imu",0,&out) && out.z==0.1);
  RosTestTrue("moving transform",RosTfLookup(tb,"odom","lidar",0,&out) && out.x==1.0 &&
	out.y==2.0 && out.z==0.3);
  tp.br=br;
  MemCpy(&tp.t,&t,sizeof(CRosTf));
  RosTimerNew(robot,10*ROS_NS_PER_MS,&TestTfTick,&tp);
  ros_cli_assist=e;
  s=RosTfEchoStr("odom","base_link",ROS_NS_PER_SEC);
  RosTestStr("tf2_echo",s,"At time 1700000000.500\n- Translation: [1.000, 2.000, 0.000]\n- Rotation: in Quaternion [0.000, 0.000, 0.000, 1.000]\n- Rotation: in RPY (radian) [0.000, 0.000, 0.000]\n");
  Free(s);
  s=RosTfEchoStr("odom","nowhere",50*ROS_NS_PER_MS);
  RosTestTrue("tf2_echo error",RosStrStartsWith(s,"Could not transform nowhere to odom"));
  Free(s);
  ros_cli_assist=NULL;
  RosTfListenerDel(l);
  RosTfBroadcasterDel(br);
  RosTfBroadcasterDel(sb);
  RosTfBufferDel(tb);
  RosNodeDel(robot);
  RosNodeDel(viewer);
  RosExecDel(e);
}

U0 TestTf()
{
  TestTfMath();
  TestTfBuffer();
  TestTfRos();
}
