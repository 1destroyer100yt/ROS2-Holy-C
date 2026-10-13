//BridgeStart: turn networking on and off, and the network task.

//Handle network traffic once. Returns datagrams handled.
I64 RosDdsSpin()
{
  I64 n;
  if (!ros_dds)
    return 0;
  n=RosDdsSpinOnce(ros_dds);
  if (ros_dds_info_dirty) {
    RosLockGet(&ros_dds->lock);
    ros_dds_info_dirty=FALSE;
    RosDdsPublishInfo(ros_dds);
    RosLockPut(&ros_dds->lock);
  }
  return n;
}

U0 RosDdsTask(U8 *data)
{
  while (ros_dds_running) {
    if (!RosDdsSpin())
      Sleep(1);
  }
}

//Type hash for SEDP: "pkg::msg::dds_::Name_" -> RosTypeHash; service
//request/reply types get the service's hash, as rmw does in Jazzy.
U8 *RosDdsTypeHash(U8 *dds_type)
{
  U8 *name=RosDdsTypeDemangle(dds_type),*h=NULL,*srv;
  CRosType *t;
  CRosSrvType *st;
  I64 n;
  if (!name)
    return NULL;
  n=StrLen(name);
  srv=NULL;
  if (RosStrFind(name,"/srv/")>=0) {
    if (RosStrEndsWith(name,"_Request"))
      srv=RosStrSub(name,0,n-8);
    else if (RosStrEndsWith(name,"_Response"))
      srv=RosStrSub(name,0,n-9);
  }
  if (srv) {
    st=RosSrvFind(srv);
    if (st)
      h=RosSrvTypeHash(st);
    Free(srv);
  } else {
    t=RosTypeFind(name);
    if (t)
      h=RosTypeHash(t);
  }
  Free(name);
  return h;
}

//Start networking on a domain. Existing publishers/subscriptions are
//bridged too. Returns the participant (also in ros_dds), NULL if the
//ports couldn't be opened.
CRtpsPart *RosDdsStart(I64 domain,Bool multicast)
{
  CRtpsPart *pt;
  I64 i,j;
  CRosNode *n;
  if (ros_dds)
    return ros_dds;
  pt=RosDdsNew(domain,"ros2_holyc",multicast);
  if (!pt)
    return NULL;
  rtps_hook_type_hash=&RosDdsTypeHash;
  RosLockGet(&pt->lock);
  ros_dds_info_w=RtpsWriterNew(pt,RosDdsNewEid(pt,RTPS_KIND_WRITER_NOKEY),
	"ros_discovery_info","rmw_dds_common::msg::dds_::ParticipantEntitiesInfo_",TRUE,TRUE,1);
  RtpsSedpAnnounce(pt,&ros_dds_info_w->guid,ros_dds_info_w->topic,ros_dds_info_w->type,
	TRUE,TRUE,TRUE,FALSE);
  RosLockPut(&pt->lock);
  ros_dds=pt;
  ros_hook_pub_new=&RosDdsPubNew;
  ros_hook_pub_del=&RosDdsPubDel;
  ros_hook_sub_new=&RosDdsSubNew;
  ros_hook_sub_del=&RosDdsSubDel;
  ros_hook_publish=&RosDdsPublish;
  ros_hook_srv_new=&RosDdsSrvNew;
  ros_hook_srv_del=&RosDdsSrvDel;
  ros_hook_srv_reply=&RosDdsSrvReply;
  ros_hook_cli_new=&RosDdsCliNew;
  ros_hook_cli_del=&RosDdsCliDel;
  ros_hook_cli_ready=&RosDdsCliReady;
  ros_hook_cli_call=&RosDdsCliCall;
  RosLockGet(&pt->lock);
  RosDdsGraphStart(pt);
  RosLockPut(&pt->lock);
//@host: ros_hook_spin_extra=&RosDdsSpin;	//one task on the host: executors pump the network
  //bridge what already exists
  RosGraphLock();
  for (i=0;i<RosGraph()->nodes->len;i++) {
    n=RosVecGetI64(ros_graph->nodes,i);
    RosGraphUnlock();
    for (j=0;j<n->pubs->len;j++)
      RosDdsPubNew(RosVecGetI64(n->pubs,j));
    for (j=0;j<n->subs->len;j++)
      RosDdsSubNew(RosVecGetI64(n->subs,j));
    for (j=0;j<n->srvs->len;j++)
      RosDdsSrvNew(RosVecGetI64(n->srvs,j));
    for (j=0;j<n->clis->len;j++)
      RosDdsCliNew(RosVecGetI64(n->clis,j));
    RosGraphLock();
  }
  RosGraphUnlock();
  ros_dds_info_dirty=TRUE;
  ros_dds_running=TRUE;
//@host-skip-begin
  Spawn(&RosDdsTask,NULL,"ros_dds");
//@host-skip-end
  return pt;
}

U0 RosDdsStop()
{
  CRtpsPart *pt=ros_dds;
  I64 i,j;
  CRosNode *n;
  CRosPub *p;
  CRosSub *s;
  CRosService *sv;
  CRosClient *c;
  if (!pt)
    return;
  ros_dds_running=FALSE;
  ros_hook_pub_new=NULL;
  ros_hook_pub_del=NULL;
  ros_hook_sub_new=NULL;
  ros_hook_sub_del=NULL;
  ros_hook_publish=NULL;
  ros_hook_srv_new=NULL;
  ros_hook_srv_del=NULL;
  ros_hook_srv_reply=NULL;
  ros_hook_cli_new=NULL;
  ros_hook_cli_del=NULL;
  ros_hook_cli_ready=NULL;
  ros_hook_cli_call=NULL;
  ros_hook_spin_extra=NULL;
  rtps_hook_type_hash=NULL;
  RosDdsGraphStop();
  RosGraphLock();
  for (i=0;i<RosGraph()->nodes->len;i++) {
    n=RosVecGetI64(ros_graph->nodes,i);
    for (j=0;j<n->pubs->len;j++) {
      p=RosVecGetI64(n->pubs,j);
      p->dds=NULL;
    }
    for (j=0;j<n->subs->len;j++) {
      s=RosVecGetI64(n->subs,j);
      s->dds=NULL;
    }
    for (j=0;j<n->srvs->len;j++) {
      sv=RosVecGetI64(n->srvs,j);
      Free(sv->dds);
      sv->dds=NULL;
    }
    for (j=0;j<n->clis->len;j++) {
      c=RosVecGetI64(n->clis,j);
      Free(c->dds);
      c->dds=NULL;
    }
  }
  RosGraphUnlock();
  Sleep(5);	//let the network task notice
  ros_dds=NULL;
  ros_dds_info_w=NULL;
  RosDdsDel(pt);
}
