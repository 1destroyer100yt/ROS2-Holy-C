//TfRos: tf over ROS topics, compatible with tf2_ros.
//  /tf         tf2_msgs/TFMessage, moving transforms
//  /tf_static  tf2_msgs/TFMessage, transient local: late joiners get
//              every static transform (one message holds them all)
//
//  CRosTfBroadcaster *br=RosTfBroadcasterNew(node);
//  RosTfSend(br,"odom","base_link",stamp,&t);
//  CRosTfBroadcaster *sb=RosTfStaticBroadcasterNew(node);
//  RosTfSend(sb,"base_link","laser",0,&t);

class CRosTfListener
{
  CRosTfBuffer *buf;
  CRosSub *tf,*tf_static;
};

class CRosTfBroadcaster
{
  CRosPub *pub;
  Bool is_static;
  CMsg_tf2_msgs_TFMessage *statics;	//all static transforms sent so far
};

I64 RosStampNs(CMsg_builtin_interfaces_Time *t)
{
  return RosTimeMake(t->sec,t->nanosec);
}

U0 RosTfFromMsg(CRosTf *t,CMsg_geometry_msgs_Transform *m)
{
  RosTfSetXyzQ(t,m->translation.x,m->translation.y,m->translation.z,
	m->rotation.x,m->rotation.y,m->rotation.z,m->rotation.w);
}

U0 RosTfToMsg(CMsg_geometry_msgs_Transform *m,CRosTf *t)
{
  m->translation.x=t->x;
  m->translation.y=t->y;
  m->translation.z=t->z;
  m->rotation.x=t->qx;
  m->rotation.y=t->qy;
  m->rotation.z=t->qz;
  m->rotation.w=t->qw;
}

U0 RosTfTake(CMsg_tf2_msgs_TFMessage *m,CRosTfBuffer *b,Bool is_static)
{
  CMsg_geometry_msgs_TransformStamped *ts=m->transforms.data;
  CRosTf t;
  I64 i;
  for (i=0;i<m->transforms.len;i++) {
    RosTfFromMsg(&t,&ts[i].transform);
    RosTfSet(b,ts[i].header.frame_id,ts[i].child_frame_id,RosStampNs(&ts[i].header.stamp),
	&t,is_static);
  }
}

U0 RosTfOnTf(CMsg_tf2_msgs_TFMessage *m,CRosTfBuffer *b)
{
  RosTfTake(m,b,FALSE);
}

U0 RosTfOnStatic(CMsg_tf2_msgs_TFMessage *m,CRosTfBuffer *b)
{
  RosTfTake(m,b,TRUE);
}

U0 RosTfQoSStatic(CRosQoS *q)
{
  RosQoSTransientLocal(q,1);
}

//Keep b filled from /tf and /tf_static (callbacks run in n's executor).
CRosTfListener *RosTfListenerNew(CRosNode *n,CRosTfBuffer *b)
{
  CRosTfListener *l=CAlloc(sizeof(CRosTfListener));
  CRosQoS q;
  l->buf=b;
  RosQoSDefault(&q);
  q.depth=100;
  l->tf=RosSubNew(n,"/tf",ros_t_tf2_msgs_TFMessage,&q,&RosTfOnTf,b);
  RosTfQoSStatic(&q);
  l->tf_static=RosSubNew(n,"/tf_static",ros_t_tf2_msgs_TFMessage,&q,&RosTfOnStatic,b);
  return l;
}

U0 RosTfListenerDel(CRosTfListener *l)
{
  if (!l)
    return;
  RosSubDel(l->tf);
  RosSubDel(l->tf_static);
  Free(l);
}

CRosTfBroadcaster *RosTfBroadcasterNew(CRosNode *n)
{
  CRosTfBroadcaster *br=CAlloc(sizeof(CRosTfBroadcaster));
  CRosQoS q;
  RosQoSDefault(&q);
  q.depth=100;
  br->pub=RosPubNew(n,"/tf",ros_t_tf2_msgs_TFMessage,&q);
  return br;
}

CRosTfBroadcaster *RosTfStaticBroadcasterNew(CRosNode *n)
{
  CRosTfBroadcaster *br=CAlloc(sizeof(CRosTfBroadcaster));
  CRosQoS q;
  RosTfQoSStatic(&q);
  br->pub=RosPubNew(n,"/tf_static",ros_t_tf2_msgs_TFMessage,&q);
  br->is_static=TRUE;
  br->statics=RosMsgNew(ros_t_tf2_msgs_TFMessage);
  return br;
}

U0 RosTfBroadcasterDel(CRosTfBroadcaster *br)
{
  if (!br)
    return;
  RosPubDel(br->pub);
  RosMsgDel(ros_t_tf2_msgs_TFMessage,br->statics);
  Free(br);
}

U0 RosTfFillStamped(CMsg_geometry_msgs_TransformStamped *ts,U8 *parent,U8 *child,I64 stamp,
	CRosTf *t)
{
  ts->header.stamp.sec=RosTimeSec(stamp);
  ts->header.stamp.nanosec=RosTimeNsec(stamp);
  Free(ts->header.frame_id);
  ts->header.frame_id=StrNew(RosTfName(parent));
  Free(ts->child_frame_id);
  ts->child_frame_id=StrNew(RosTfName(child));
  RosTfToMsg(&ts->transform,t);
}

//Send where child is relative to parent at stamp (ns).
U0 RosTfSend(CRosTfBroadcaster *br,U8 *parent,U8 *child,I64 stamp,CRosTf *t)
{
  CMsg_tf2_msgs_TFMessage *m;
  CMsg_geometry_msgs_TransformStamped *ts;
  CRosField *f=RosTypeField(ros_t_tf2_msgs_TFMessage,"transforms");
  I64 i,n;
  if (br->is_static) {	//replace or add, then send them all
    ts=br->statics->transforms.data;
    n=br->statics->transforms.len;
    for (i=0;i<n;i++)
      if (!StrCmp(ts[i].child_frame_id,RosTfName(child)))
        break;
    if (i==n) {
      RosSeqResize(f,br->statics,n+1);
      ts=br->statics->transforms.data;
    }
    RosTfFillStamped(&ts[i],parent,child,stamp,t);
    RosPublish(br->pub,br->statics);
    return;
  }
  m=RosPubLoan(br->pub);
  RosSeqResize(f,m,1);
  ts=m->transforms.data;
  RosTfFillStamped(ts,parent,child,stamp,t);
  RosPublishLoaned(br->pub,m);
}

//Add a fixed-point number with d decimals.
U0 RosStrBufAddFixed(CRosStrBuf *sb,F64 x,I64 d)
{
  I64 scale=1,i,v,ip,fp;
  for (i=0;i<d;i++)
    scale*=10;
  v=ToI64(Abs(x)*scale+0.5);
  if (x<0.0 && v)
    RosStrBufAddC(sb,'-');
  ip=v/scale;
  fp=v%scale;
  RosStrBufAddI64(sb,ip);
  if (d) {
    RosStrBufAddC(sb,'.');
    i=scale/10;
    while (i>fp && i>1) {
      RosStrBufAddC(sb,'0');
      i/=10;
    }
    if (fp)
      RosStrBufAddI64(sb,fp);
    else if (scale>1)
      RosStrBufAddC(sb,'0');
  }
}

//What tf2_echo prints for T(target <- source).
U8 *RosTfEchoFmt(CRosTf *t,I64 stamp)
{
  CRosStrBuf *sb=RosStrBufNew();
  F64 r,p,y;
  I64 ms=RosTimeNsec(stamp)/1000000;
  RosStrBufAdd(sb,"At time ");
  RosStrBufAddI64(sb,RosTimeSec(stamp));
  RosStrBufAddC(sb,'.');
  if (ms<100)
    RosStrBufAddC(sb,'0');
  if (ms<10)
    RosStrBufAddC(sb,'0');
  RosStrBufAddI64(sb,ms);
  RosStrBufAdd(sb,"\n- Translation: [");
  RosStrBufAddFixed(sb,t->x,3);
  RosStrBufAdd(sb,", ");
  RosStrBufAddFixed(sb,t->y,3);
  RosStrBufAdd(sb,", ");
  RosStrBufAddFixed(sb,t->z,3);
  RosStrBufAdd(sb,"]\n- Rotation: in Quaternion [");
  RosStrBufAddFixed(sb,t->qx,3);
  RosStrBufAdd(sb,", ");
  RosStrBufAddFixed(sb,t->qy,3);
  RosStrBufAdd(sb,", ");
  RosStrBufAddFixed(sb,t->qz,3);
  RosStrBufAdd(sb,", ");
  RosStrBufAddFixed(sb,t->qw,3);
  RosTfGetRpy(t,&r,&p,&y);
  RosStrBufAdd(sb,"]\n- Rotation: in RPY (radian) [");
  RosStrBufAddFixed(sb,r,3);
  RosStrBufAdd(sb,", ");
  RosStrBufAddFixed(sb,p,3);
  RosStrBufAdd(sb,", ");
  RosStrBufAddFixed(sb,y,3);
  RosStrBufAdd(sb,"]\n");
  return RosStrBufTake(sb);
}

//Like `ros2 run tf2_ros tf2_echo target source`: listen for up to
//wait_ns, then print the latest transform (or why there isn't one).
U8 *RosTfEchoStr(U8 *target,U8 *source,I64 wait_ns)
{
  CRosNode *n=RosCliNodeNew();
  CRosTfBuffer *b=RosTfBufferNew();
  CRosTfListener *l=RosTfListenerNew(n,b);
  CRosTf t;
  U8 *s;
  I64 end=RosTimeSteadyNs()+wait_ns;
  Bool ok=FALSE;
  while (!ok && RosTimeSteadyNs()<end && ros_ok) {
    RosCliSpinOnce(n,10*ROS_NS_PER_MS);
    ok=RosTfLookup(b,target,source,0,&t);
  }
  if (ok)
    s=RosTfEchoFmt(&t,RosTfLatestTime(b,target,source));
  else
    s=MStrPrint("Could not transform %s to %s: %s\n",source,target,ros_tf_error);
  RosTfListenerDel(l);
  RosTfBufferDel(b);
  RosCliNodeDel(n);
  return s;
}

U0 RosTfEcho(U8 *target,U8 *source)
{
  RosCliPrint(RosTfEchoStr(target,source,3*ROS_NS_PER_SEC));
}
