//Tests for src/Tools/Bag.HC (rosbag2 MCAP).

U8 *TestBagDir(U8 *name)	//where test bags go
{
  U8 *s=NULL;
//@host-skip-begin
  s=MStrPrint("~/Tmp/%s",name);
//@host-skip-end
//@host: s=MStrPrint("%s/%s",getenv("TMPDIR") ? getenv("TMPDIR") : "/tmp",name);
  return s;
}

class CTestBagGot
{
  I64 twists,strings;
  F64 last_x;
  U8 last_str[32];
};

U0 TestBagOnTwist(CMsg_geometry_msgs_Twist *m,CTestBagGot *g)
{
  g->twists++;
  g->last_x=m->linear.x;
}

U0 TestBagOnString(CMsg_std_msgs_String *m,CTestBagGot *g)
{
  g->strings++;
  if (StrLen(m->data)<32)
    StrCpy(g->last_str,m->data);
}

U0 TestBagWriteRead()
{
  U8 *dir=TestBagDir("holyc_bag_rw"),*s,*meta,*path;
  CRosBag *bag;
  CMsg_geometry_msgs_Twist *tw=RosMsgNew(ros_t_geometry_msgs_Twist);
  CMsg_std_msgs_String *str=RosMsgNew(ros_t_std_msgs_String);
  CMsg_geometry_msgs_Twist *back=RosMsgNew(ros_t_geometry_msgs_Twist);
  CRosBagChan *c;
  CRosBagMsg *m;
  I64 i,t0=1700000000*ROS_NS_PER_SEC;
  RosTestGroup("Bag write/read");
  bag=RosBagCreate(dir);
  for (i=0;i<5;i++) {
    tw->linear.x=i*0.5;
    tw->angular.z=-1.25;
    RosBagWriteMsg(bag,"/cmd_vel",ros_t_geometry_msgs_Twist,t0+i*100*ROS_NS_PER_MS,tw);
  }
  Free(str->data);
  str->data=StrNew("hello bag");
  RosBagWriteMsg(bag,"/chatter",ros_t_std_msgs_String,t0+250*ROS_NS_PER_MS,str);
  RosTestTrue("close writes the bag",RosBagClose(bag));
  RosBagFree(bag);
  path=MStrPrint("%s/metadata.yaml",dir);
  meta=RosFileRead(path,NULL);
  Free(path);
  RosTestTrue("metadata.yaml written",meta!=NULL);
  if (meta) {
    RosTestTrue("metadata: count",RosStrFind(meta,"  message_count: 6\n")>=0);
    RosTestTrue("metadata: topic",RosStrFind(meta,"name: /cmd_vel\n        type: geometry_msgs/msg/Twist\n")>=0);
    RosTestTrue("metadata: file",RosStrFind(meta,"    - holyc_bag_rw_0.mcap\n")>=0);
    RosTestTrue("metadata: duration",RosStrFind(meta,"nanoseconds: 400000000\n")>=0);
    Free(meta);
  }
  bag=RosBagOpen(dir);
  RosTestTrue("open",bag!=NULL);
  if (bag) {
    RosTestI64("messages",bag->msgs->len,6);
    RosTestI64("channels",bag->chans->len,2);
    RosTestI64("start",bag->start,t0);
    RosTestI64("end",bag->end,t0+400*ROS_NS_PER_MS);
    c=RosBagChanFind(bag,"/cmd_vel");
    RosTestTrue("channel type",c && c->t==ros_t_geometry_msgs_Twist && c->count==5);
    m=RosVecGetI64(bag->msgs,4);	//the string at 250 ms sorts before it
    RosTestTrue("deserializes",RosCdrDeser(ros_t_geometry_msgs_Twist,m->data,m->len,back));
    RosTestTrue("message 4 data",back->linear.x==1.5 && back->angular.z==-1.25);
    RosBagFree(bag);
  }
  s=RosBagInfoStr(dir);
  RosTestStr("info",s,"Storage id:        mcap\nDuration:          0.400000000s\nMessages:          6\nTopic information: Topic: /cmd_vel | Type: geometry_msgs/msg/Twist | Count: 5 | Serialization Format: cdr\n                   Topic: /chatter | Type: std_msgs/msg/String | Count: 1 | Serialization Format: cdr\n");
  Free(s);
  s=RosBagInfoStr("/nonexistent/bag");
  RosTestTrue("info: missing bag",RosStrStartsWith(s,"Error: can't read"));
  Free(s);
  RosMsgDel(ros_t_geometry_msgs_Twist,tw);
  RosMsgDel(ros_t_geometry_msgs_Twist,back);
  RosMsgDel(ros_t_std_msgs_String,str);
  Free(dir);
}

U0 TestBagLearn()
{
  U8 *text="Inner in\nfloat64 w\n================================================================================\nMSG: holyc_bag_test/Inner\nint32 a\nstring b\n";
  CRosType *t;
  RosTestGroup("Bag schema learning");
  RosTestTrue("learn custom type",RosBagLearnSchema("holyc_bag_test/msg/Outer",text));
  t=RosTypeFind("holyc_bag_test/msg/Outer");
  RosTestTrue("outer defined",t!=NULL && t->fields->len==2);
  RosTestTrue("inner defined",RosTypeFind("holyc_bag_test/msg/Inner")!=NULL);
  RosTestTrue("known type is fine",RosBagLearnSchema("std_msgs/msg/String",""));
  RosTestTrue("bad text refused",!RosBagLearnSchema("holyc_bag_test/msg/Bad","Nope x\n"));
}

U0 TestBagRecordPlay()
{
  U8 *dir=TestBagDir("holyc_bag_rec");
  CRosNode *talker,*listener;
  CRosExec *e=RosExecNew();
  CRosPub *p1,*p2;
  CRosBagRec *r;
  CMsg_geometry_msgs_Twist *tw;
  CMsg_std_msgs_String *str;
  CTestBagGot got;
  CRosQoS q;
  I64 i,n;
  RosTestGroup("Bag record/play");
  talker=RosNodeNew("bag_talker",NULL);
  listener=RosNodeNew("bag_listener",NULL);
  RosExecAdd(e,talker);
  RosExecAdd(e,listener);
  p1=RosPubNew(talker,"/cmd_vel",ros_t_geometry_msgs_Twist,NULL);
  p2=RosPubNew(talker,"/chatter",ros_t_std_msgs_String,NULL);
  r=RosBagRecordStart(dir,"/cmd_vel /chatter");
  RosTestI64("recording 2 topics",r->subs->len,2);
  for (i=0;i<10;i++) {
    tw=RosPubLoan(p1);
    tw->linear.x=i;
    RosPublishLoaned(p1,tw);
  }
  str=RosPubLoan(p2);
  Free(str->data);
  str->data=StrNew("recorded");
  RosPublishLoaned(p2,str);
  RosTestTrue("record stop",RosBagRecordStop(r));
  MemSet(&got,0,sizeof(CTestBagGot));
  RosQoSDefault(&q);
  q.depth=100;
  RosSubNew(listener,"/cmd_vel",ros_t_geometry_msgs_Twist,&q,&TestBagOnTwist,&got);
  RosSubNew(listener,"/chatter",ros_t_std_msgs_String,&q,&TestBagOnString,&got);
  ros_cli_assist=e;
  n=RosBagPlay(dir,0.0);
  RosExecSpinSome(e);
  ros_cli_assist=NULL;
  RosTestI64("played",n,11);
  RosTestI64("twists heard",got.twists,10);
  RosTestI64("strings heard",got.strings,1);
  RosTestTrue("last twist",got.last_x==9.0);
  RosTestStr("string",got.last_str,"recorded");
  RosNodeDel(talker);
  RosNodeDel(listener);
  RosExecDel(e);
  Free(dir);
}

U0 TestBag()
{
  TestBagWriteRead();
  TestBagLearn();
  TestBagRecordPlay();
}
