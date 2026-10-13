//Bridge: puts the local ROS graph on the network. Every publisher and
//subscription gets a DDS writer/reader, named the way ROS 2 does it:
//  topic /chatter            -> DDS topic "rt/chatter"
//  type  std_msgs/msg/String -> "std_msgs::msg::dds_::String_"
//and the nodes are announced on "ros_discovery_info" so `ros2 node
//list` on other machines shows them.
//
//  RosDdsStart(0,TRUE);    //domain 0, multicast discovery
//  RosDdsAddPeer(ros_dds,RosIpParse("10.0.2.2"));   //optional unicast peer
//  ...                     //TempleOS: a task spins it; host: RosDdsSpin()
//  RosDdsStop;

CRtpsWriter *ros_dds_info_w=NULL;
Bool ros_dds_info_dirty=FALSE;
Bool ros_dds_running=FALSE;

U8 *RosDdsTopicName(U8 *ros_name)
{
  return MStrPrint("rt%s",ros_name);
}

U8 *RosDdsTypeName(CRosType *t)
{
  CRosStrBuf *sb=RosStrBufNew();
  U8 *kind="msg";
  if (RosStrFind(t->name,"/srv/")>=0)
    kind="srv";
  else if (RosStrFind(t->name,"/action/")>=0)
    kind="action";
  RosStrBufAdd(sb,t->pkg);
  RosStrBufAdd(sb,"::");
  RosStrBufAdd(sb,kind);
  RosStrBufAdd(sb,"::dds_::");
  RosStrBufAdd(sb,t->base);
  RosStrBufAddC(sb,'_');
  return RosStrBufTake(sb);
}

I64 RosDdsNewEid(CRtpsPart *pt,I64 kind)
{
  I64 key=pt->next_key++;
  return (key<<8)|kind;
}

U0 RosDdsOnData(CRtpsReader *r,CRtpsGuid *from,CRtpsChange *c)
{
  CRosSub *s=r->user;
  CRosType *t=s->topic->type;
  CRosShared *sh;
  if (!c->data)
    return;
  sh=RosSharedNew(t);
  if (RosCdrDeser(t,c->data,c->len,RosSharedMsg(sh))) {
    sh->stamp_ns=RosTimeSteadyNs();
    RosSubPush(s,sh);
  }
  RosSharedRelease(sh);
}

U0 RosDdsPubNew(CRosPub *p)
{
  CRtpsPart *pt=ros_dds;
  CRtpsWriter *w;
  U8 *topic,*type;
  if (!pt)
    return;
  topic=RosDdsTopicName(p->topic->name);
  type=RosDdsTypeName(p->topic->type);
  RosLockGet(&pt->lock);
  w=RtpsWriterNew(pt,RosDdsNewEid(pt,RTPS_KIND_WRITER_NOKEY),topic,type,
	p->qos.reliability==ROS_QOS_RELIABLE,p->qos.durability==ROS_QOS_TRANSIENT_LOCAL,
	p->qos.depth);
  w->user=p;
  p->dds=w;
  RtpsSedpAnnounce(pt,&w->guid,topic,type,w->reliable,w->transient,TRUE,FALSE);
  RtpsMatchLocal(pt,w,NULL);
  ros_dds_info_dirty=TRUE;
  RosLockPut(&pt->lock);
  Free(topic);
  Free(type);
}

U0 RosDdsPubDel(CRosPub *p)
{
  CRtpsPart *pt=ros_dds;
  CRtpsWriter *w=p->dds;
  if (!pt || !w)
    return;
  RosLockGet(&pt->lock);
  RtpsSedpAnnounce(pt,&w->guid,w->topic,w->type,w->reliable,w->transient,TRUE,TRUE);
  RtpsWriterDel(pt,w);
  p->dds=NULL;
  ros_dds_info_dirty=TRUE;
  RosLockPut(&pt->lock);
}

U0 RosDdsSubNew(CRosSub *s)
{
  CRtpsPart *pt=ros_dds;
  CRtpsReader *r;
  U8 *topic,*type;
  if (!pt)
    return;
  topic=RosDdsTopicName(s->topic->name);
  type=RosDdsTypeName(s->topic->type);
  RosLockGet(&pt->lock);
  r=RtpsReaderNew(pt,RosDdsNewEid(pt,RTPS_KIND_READER_NOKEY),topic,type,
	s->qos.reliability==ROS_QOS_RELIABLE,s->qos.durability==ROS_QOS_TRANSIENT_LOCAL,
	&RosDdsOnData,s);
  s->dds=r;
  RtpsSedpAnnounce(pt,&r->guid,topic,type,r->reliable,r->transient,FALSE,FALSE);
  RtpsMatchLocal(pt,NULL,r);
  ros_dds_info_dirty=TRUE;
  RosLockPut(&pt->lock);
  Free(topic);
  Free(type);
}

U0 RosDdsSubDel(CRosSub *s)
{
  CRtpsPart *pt=ros_dds;
  CRtpsReader *r=s->dds;
  if (!pt || !r)
    return;
  RosLockGet(&pt->lock);
  RtpsSedpAnnounce(pt,&r->guid,r->topic,r->type,r->reliable,r->transient,FALSE,TRUE);
  RtpsReaderDel(pt,r);
  s->dds=NULL;
  ros_dds_info_dirty=TRUE;
  RosLockPut(&pt->lock);
}

U0 RosDdsPublish(CRosPub *p,U8 *msg)
{
  CRtpsPart *pt=ros_dds;
  CRtpsWriter *w=p->dds;
  CRosBuf *b;
  U8 *data;
  I64 len;
  if (!pt || !w)
    return;
  RosLockGet(&pt->lock);
  if (w->readers->len || w->transient) {
    b=RosBufNew();
    RosCdrSer(p->topic->type,msg,b);
    len=b->len;
    data=b->data;
    b->owned=FALSE;
    RosBufDel(b);
    RtpsWrite(pt,w,data,len,NULL,FALSE);
  }
  RosLockPut(&pt->lock);
}

//The 16-byte gid of a DDS endpoint, into a Gid message.
U0 RosDdsGid(CMsg_rmw_dds_common_Gid *gid,CRtpsGuid *g)
{
  MemCpy(gid->data,g->prefix,12);
  gid->data[12]=(g->eid>>24)&255;
  gid->data[13]=(g->eid>>16)&255;
  gid->data[14]=(g->eid>>8)&255;
  gid->data[15]=g->eid&255;
}

//Publish who we are on ros_discovery_info (lock held).
U0 RosDdsPublishInfo(CRtpsPart *pt)
{
  CRosType *t=ros_t_rmw_dds_common_ParticipantEntitiesInfo;
  CMsg_rmw_dds_common_ParticipantEntitiesInfo *m=RosMsgNew(t);
  CMsg_rmw_dds_common_NodeEntitiesInfo *ni;
  CRosNode *n;
  CRosPub *p;
  CRosSub *s;
  CRtpsWriter *w;
  CRtpsReader *r;
  CRtpsGuid g;
  CRosBuf *b;
  I64 i,j;
  MemCpy(g.prefix,pt->prefix,12);
  g.eid=RTPS_EID_PARTICIPANT;
  RosDdsGid(&m->gid,&g);
  RosGraphLock();
  for (i=0;i<RosGraph()->nodes->len;i++) {
    n=RosVecGetI64(ros_graph->nodes,i);
    ni=RosSeqAppend(t,m,"node_entities_info_seq");
    ni->node_namespace=StrNew(n->ns);
    ni->node_name=StrNew(n->name);
    for (j=0;j<n->subs->len;j++) {
      s=RosVecGetI64(n->subs,j);
      r=s->dds;
      if (r)
        RosDdsGid(RosSeqAppend(ros_t_rmw_dds_common_NodeEntitiesInfo,ni,"reader_gid_seq"),
	      &r->guid);
    }
    for (j=0;j<n->pubs->len;j++) {
      p=RosVecGetI64(n->pubs,j);
      w=p->dds;
      if (w)
        RosDdsGid(RosSeqAppend(ros_t_rmw_dds_common_NodeEntitiesInfo,ni,"writer_gid_seq"),
	      &w->guid);
    }
  }
  RosGraphUnlock();
  b=RosBufNew();
  RosCdrSer(t,m,b);
  RtpsWrite(pt,ros_dds_info_w,b->data,b->len,NULL,FALSE);
  b->owned=FALSE;
  RosBufDel(b);
  RosMsgDel(t,m);
}

