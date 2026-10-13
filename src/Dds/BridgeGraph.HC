//BridgeGraph: what other machines have, for the CLI: their topics
//and services (from SEDP) and their nodes (from ros_discovery_info).

class CRosDdsNodes	//nodes of one remote participant
{
  U8 prefix[12];
  CRosVec *fqns;	//U8 *
};

CRosVec *ros_dds_nodes=NULL;	//CRosDdsNodes *
CRtpsReader *ros_dds_info_r=NULL;

//"std_msgs::msg::dds_::String_" -> "std_msgs/msg/String" (new), NULL if not ROS-shaped.
U8 *RosDdsTypeDemangle(U8 *dds)
{
  I64 a=RosStrFind(dds,"::"),b=RosStrFind(dds,"::dds_::"),n=StrLen(dds);
  CRosStrBuf *sb;
  if (a<=0 || b<=a || n<b+10 || dds[n-1]!='_')
    return NULL;
  sb=RosStrBufNew();
  RosStrBufAddN(sb,dds,a);
  RosStrBufAddC(sb,'/');
  RosStrBufAddN(sb,dds+a+2,b-a-2);
  RosStrBufAddC(sb,'/');
  RosStrBufAddN(sb,dds+b+8,n-b-9);
  return RosStrBufTake(sb);
}

//Add name/type pairs not already there.
U0 RosDdsAddUnique(CRosVec *names,CRosVec *types,U8 *name,U8 *type)
{
  I64 i;
  for (i=0;i<names->len;i++)
    if (!StrCmp(RosVecGetI64(names,i),name)) {
      Free(name);
      Free(type);
      return;
    }
  RosVecPushI64(names,name);
  RosVecPushI64(types,type);
}

U0 RosDdsRemoteTopics(CRosVec *names,CRosVec *types)
{
  CRtpsPart *pt=ros_dds;
  CRtpsRemoteEp *e;
  U8 *t;
  I64 i;
  if (!pt)
    return;
  RosLockGet(&pt->lock);
  for (i=0;i<pt->remote_eps->len;i++) {
    e=RosVecGetI64(pt->remote_eps,i);
    if (!RosStrStartsWith(e->topic,"rt/"))
      continue;
    t=RosDdsTypeDemangle(e->type);
    if (t)
      RosDdsAddUnique(names,types,StrNew(e->topic+2),t);
  }
  RosLockPut(&pt->lock);
}

U0 RosDdsRemoteServices(CRosVec *names,CRosVec *types)
{
  CRtpsPart *pt=ros_dds;
  CRtpsRemoteEp *e;
  U8 *t;
  I64 i,n;
  if (!pt)
    return;
  RosLockGet(&pt->lock);
  for (i=0;i<pt->remote_eps->len;i++) {
    e=RosVecGetI64(pt->remote_eps,i);
    n=StrLen(e->topic);
    if (!RosStrStartsWith(e->topic,"rq/") || !RosStrEndsWith(e->topic,"Request") ||
	!RosStrEndsWith(e->type,"_Request_"))
      continue;
    t=RosDdsTypeDemangle(e->type);
    if (t) {
      t[StrLen(t)-8]=0;	//drop "_Request"
      RosDdsAddUnique(names,types,RosStrSub(e->topic,2,n-9),t);
    }
  }
  RosLockPut(&pt->lock);
}

U0 RosDdsRemoteNodes(CRosVec *fqns)
{
  CRtpsPart *pt=ros_dds;
  CRosDdsNodes *dn;
  I64 i,j;
  if (!pt || !ros_dds_nodes)
    return;
  RosLockGet(&pt->lock);
  for (i=0;i<ros_dds_nodes->len;i++) {
    dn=RosVecGetI64(ros_dds_nodes,i);
    if (!RtpsRemoteFind(pt,dn->prefix))	//that participant is gone
      continue;
    for (j=0;j<dn->fqns->len;j++)
      RosVecPushI64(fqns,StrNew(RosVecGetI64(dn->fqns,j)));
  }
  RosLockPut(&pt->lock);
}

U0 RosDdsOnInfo(CRtpsReader *r,CRtpsGuid *from,CRtpsChange *c)
{
  CRosType *t=ros_t_rmw_dds_common_ParticipantEntitiesInfo;
  CMsg_rmw_dds_common_ParticipantEntitiesInfo *m;
  CMsg_rmw_dds_common_NodeEntitiesInfo *ni;
  CRosDdsNodes *dn=NULL,*x;
  I64 i;
  if (!c->data)
    return;
  m=RosMsgNew(t);
  if (!RosCdrDeser(t,c->data,c->len,m)) {
    RosMsgDel(t,m);
    return;
  }
  if (!ros_dds_nodes)
    ros_dds_nodes=RosVecNew(8);
  for (i=0;i<ros_dds_nodes->len;i++) {
    x=RosVecGetI64(ros_dds_nodes,i);
    if (RtpsPrefixEq(x->prefix,m->gid.data))
      dn=x;
  }
  if (!dn) {
    dn=CAlloc(sizeof(CRosDdsNodes));
    MemCpy(dn->prefix,m->gid.data,12);
    dn->fqns=RosVecNew(8);
    RosVecPushI64(ros_dds_nodes,dn);
  }
  RosStrVecDel(dn->fqns);
  dn->fqns=RosVecNew(8);
  ni=m->node_entities_info_seq.data;
  for (i=0;i<m->node_entities_info_seq.len;i++)
    RosVecPushI64(dn->fqns,RosNameJoin(ni[i].node_namespace,ni[i].node_name));
  RosMsgDel(t,m);
}

U0 RosDdsGraphStart(CRtpsPart *pt)
{
  ros_dds_info_r=RtpsReaderNew(pt,RosDdsNewEid(pt,RTPS_KIND_READER_NOKEY),"ros_discovery_info",
	"rmw_dds_common::msg::dds_::ParticipantEntitiesInfo_",TRUE,TRUE,&RosDdsOnInfo,NULL);
  RtpsSedpAnnounce(pt,&ros_dds_info_r->guid,ros_dds_info_r->topic,ros_dds_info_r->type,
	TRUE,TRUE,FALSE,FALSE);
  ros_hook_remote_topics=&RosDdsRemoteTopics;
  ros_hook_remote_services=&RosDdsRemoteServices;
  ros_hook_remote_nodes=&RosDdsRemoteNodes;
}

U0 RosDdsGraphStop()
{
  CRosDdsNodes *dn;
  I64 i;
  ros_hook_remote_topics=NULL;
  ros_hook_remote_services=NULL;
  ros_hook_remote_nodes=NULL;
  ros_dds_info_r=NULL;
  if (ros_dds_nodes) {
    for (i=0;i<ros_dds_nodes->len;i++) {
      dn=RosVecGetI64(ros_dds_nodes,i);
      RosStrVecDel(dn->fqns);
      Free(dn);
    }
    RosVecDel(ros_dds_nodes);
    ros_dds_nodes=NULL;
  }
}
