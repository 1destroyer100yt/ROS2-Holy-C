//HolyC side of the rosbag2 interop check (tools/bagtest/run.sh).
//  BagInteropWrite: write a bag for rosbags/mcap to read
//  BagInteropRead:  read the bag rosbags wrote (chunks, custom type)
//The bag directory comes from the BAG_DIR environment variable.

Cd(__DIR__);;
#include "../../Load.HC"

U8 *BagInteropDir()
{
  U8 *s=NULL;
//@host: s=(U8 *)getenv("BAG_DIR");
  return s;
}

I64 BagInteropWrite()
{
  CRosBag *bag=RosBagCreate(BagInteropDir());
  CMsg_geometry_msgs_Twist *tw;
  CMsg_std_msgs_String *str;
  CRosTfBroadcaster *br;
  CRosTf t;
  CRosNode *n;
  CRosExec *e;
  CRosBagRec *r;
  I64 i,t0=1700000000*ROS_NS_PER_SEC;
  Bool ok;
  RosIfaceLoadAll();
  tw=RosMsgNew(ros_t_geometry_msgs_Twist);
  str=RosMsgNew(ros_t_std_msgs_String);
  for (i=0;i<20;i++) {
    tw->linear.x=i*0.25;
    tw->angular.z=-i;
    RosBagWriteMsg(bag,"/cmd_vel",ros_t_geometry_msgs_Twist,t0+i*50*ROS_NS_PER_MS,tw);
  }
  Free(str->data);
  str->data=StrNew("from holyc");
  RosBagWriteMsg(bag,"/chatter",ros_t_std_msgs_String,t0+1,str);
  for (i=0;i<3;i++) {	//big ones: the file gets several chunks
    Free(str->data);
    str->data=MAlloc(400001);
    MemSet(str->data,'a'+i,400000);
    str->data[400000]=0;
    RosBagWriteMsg(bag,"/big",ros_t_std_msgs_String,t0+2+i,str);
  }
  //and two /tf messages recorded off the graph
  n=RosNodeNew("bag_tf",NULL);
  e=RosExecNew();
  RosExecAdd(e,n);
  br=RosTfBroadcasterNew(n);
  r=RosBagRecordStart("/unused","/tf");
  RosBagFree(r->bag);	//record into our bag instead
  r->bag=bag;
  RosTfIdentity(&t);
  t.x=1.0;
  RosTfSend(br,"odom","base_link",t0,&t);
  t.x=2.0;
  RosTfSend(br,"odom","base_link",t0+ROS_NS_PER_SEC,&t);
  ok=RosBagRecordStop(r);
  RosTfBroadcasterDel(br);
  RosNodeDel(n);
  RosExecDel(e);
  RosMsgDel(ros_t_geometry_msgs_Twist,tw);
  RosMsgDel(ros_t_std_msgs_String,str);
  if (!ok) {
    Print("RESULT write FAIL %s\n",ros_bag_error);
    return 1;
  }
  Print("RESULT write OK\n");
  return 0;
}

I64 BagInteropRead()
{
  CRosBag *bag;
  CRosBagChan *c;
  CRosBagMsg *m;
  CRosType *pt;
  U8 *msg,*y,*info;
  I64 i,bad=0,tw=0;
  CMsg_geometry_msgs_Twist *t;
  RosIfaceLoadAll();
  info=RosBagInfoStr(BagInteropDir());
  Print("%s",info);
  Free(info);
  bag=RosBagOpen(BagInteropDir());
  if (!bag) {
    Print("RESULT read FAIL %s\n",ros_bag_error);
    return 1;
  }
  if (bag->msgs->len!=22) {
    Print("messages: %d\n",bag->msgs->len);
    bad++;
  }
  pt=RosTypeFind("bagtest_msgs/msg/Pair");
  if (!pt) {
    Print("custom type not learned\n");
    bad++;
  }
  for (i=0;i<bag->msgs->len;i++) {
    m=RosVecGetI64(bag->msgs,i);
    c=RosBagChanById(bag,m->chan);
    if (!c || !c->t) {
      bad++;
      continue;
    }
    msg=RosMsgNew(c->t);
    if (!RosCdrDeser(c->t,m->data,m->len,msg)) {
      Print("can't deserialize on %s\n",c->topic);
      bad++;
    } else if (!StrCmp(c->topic,"/cmd_vel")) {
      t=msg;
      if (t->linear.x!=tw*0.25 || t->angular.z!=-tw ||
          m->stamp!=1700000000*ROS_NS_PER_SEC+tw*50*ROS_NS_PER_MS)
        bad++;
      tw++;
    } else {
      y=RosMsgToYaml(c->t,msg);
      Print("%s:\n%s",c->topic,y);
      if (!StrCmp(c->topic,"/chatter") && StrCmp(y,"data: 'from rosbags'\n"))
        bad++;
      if (!StrCmp(c->topic,"/pair") &&
          StrCmp(y,"name: 'pair one'\nwhere:\n  x: 1.5\n  y: -2.0\n  z: 3.0\nids:\n- 4\n- 5\n- 6\n"))
        bad++;
      Free(y);
    }
    RosMsgDel(c->t,msg);
  }
  if (tw!=20)
    bad++;
  RosBagFree(bag);
  if (bad) {
    Print("RESULT read FAIL (%d problems)\n",bad);
    return 1;
  }
  Print("RESULT read OK\n");
  return 0;
}
