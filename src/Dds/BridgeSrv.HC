//BridgeSrv: ROS services over DDS, the rmw_cyclonedds way:
//  request on "rq/<service>Request", reply on "rr/<service>Reply",
//  each payload starts with {U64 client id, I64 sequence number}.
//(rmw_fastrtps correlates differently; services between Fast DDS and
//Cyclone DDS don't work in real ROS 2 either.)

class CRosDdsSrv	//network side of a local service
{
  CRtpsReader *rq;
  CRtpsWriter *rr;
};

class CRosDdsCli	//network side of a local client
{
  CRtpsWriter *rq;
  CRtpsReader *rr;
  I64 id;		//goes in every request header
};

class CRosDdsRemoteReq	//kept on a CRosSrvReq that came from the network
{
  I64 client,seq;
};

U8 *RosDdsSrvTopic(U8 *prefix,U8 *name,U8 *suffix)
{
  return MStrPrint("%s%s%s",prefix,name,suffix);
}

//Serialize {client,seq} + message (type t) into a new buffer.
CRosBuf *RosDdsSrvSer(CRosType *t,U8 *m,I64 client,I64 seq)
{
  CRosBuf *b=RosBufNew();
  RosBufPutN(b,0x0100,2);
  RosBufPutN(b,0,2);
  b->origin=b->len;
  RosBufPutN(b,client,8);
  RosBufPutN(b,seq,8);
  RosCdrPutFields(b,t,m);
  return b;
}

//Parse {client,seq} + message into m (a RosMsgNew of t).
Bool RosDdsSrvDeser(CRosType *t,CRtpsChange *c,U8 *m,I64 *client,I64 *seq)
{
  CRosBuf *b;
  Bool ok;
  if (!c->data || c->len<20 || c->data[0] || c->data[1]>1)
    return FALSE;
  b=RosBufReader(c->data,c->len);
  b->swap=c->data[1]==0;
  b->pos=4;
  b->origin=4;
  *client=RosBufGetN(b,8);
  *seq=RosBufGetN(b,8);
  ok=!b->err && RosCdrGetFields(b,t,m);
  RosBufDel(b);
  return ok;
}

//A request from the network for one of our services.
U0 RosDdsOnRequest(CRtpsReader *r,CRtpsGuid *from,CRtpsChange *c)
{
  CRosService *s=r->user;
  CRosSrvReq *q;
  CRosDdsRemoteReq *rr;
  U8 *m=RosMsgNew(s->st->req);
  I64 client,seq;
  if (!RosDdsSrvDeser(s->st->req,c,m,&client,&seq)) {
    RosMsgDel(s->st->req,m);
    return;
  }
  rr=CAlloc(sizeof(CRosDdsRemoteReq));
  rr->client=client;
  rr->seq=seq;
  q=CAlloc(sizeof(CRosSrvReq));
  q->req=m;
  q->remote=rr;
  RosLockGet(&s->lock);
  RosVecPushI64(s->queue,q);
  RosLockPut(&s->lock);
}

//Our service answered a network request.
U0 RosDdsSrvReply(CRosService *s,CRosSrvReq *q,U8 *resp)
{
  CRtpsPart *pt=ros_dds;
  CRosDdsSrv *ds=s->dds;
  CRosDdsRemoteReq *rr=q->remote;
  CRosBuf *b;
  if (!pt || !ds)
    return;
  b=RosDdsSrvSer(s->st->resp,resp,rr->client,rr->seq);
  RosLockGet(&pt->lock);
  RtpsWrite(pt,ds->rr,b->data,b->len,NULL,FALSE);
  RosLockPut(&pt->lock);
  b->owned=FALSE;
  RosBufDel(b);
}

U0 RosDdsSrvNew(CRosService *s)
{
  CRtpsPart *pt=ros_dds;
  CRosDdsSrv *ds;
  U8 *t1,*t2,*y1,*y2;
  if (!pt)
    return;
  ds=CAlloc(sizeof(CRosDdsSrv));
  t1=RosDdsSrvTopic("rq",s->name,"Request");
  t2=RosDdsSrvTopic("rr",s->name,"Reply");
  y1=RosDdsTypeName(s->st->req);
  y2=RosDdsTypeName(s->st->resp);
  RosLockGet(&pt->lock);
  ds->rq=RtpsReaderNew(pt,RosDdsNewEid(pt,RTPS_KIND_READER_NOKEY),t1,y1,TRUE,FALSE,
	&RosDdsOnRequest,s);
  ds->rr=RtpsWriterNew(pt,RosDdsNewEid(pt,RTPS_KIND_WRITER_NOKEY),t2,y2,TRUE,FALSE,1000);
  RtpsSedpAnnounce(pt,&ds->rq->guid,t1,y1,TRUE,FALSE,FALSE,FALSE);
  RtpsSedpAnnounce(pt,&ds->rr->guid,t2,y2,TRUE,FALSE,TRUE,FALSE);
  RtpsMatchLocal(pt,ds->rr,ds->rq);
  s->dds=ds;
  ros_dds_info_dirty=TRUE;
  RosLockPut(&pt->lock);
  Free(t1);
  Free(t2);
  Free(y1);
  Free(y2);
}

U0 RosDdsSrvDel(CRosService *s)
{
  CRtpsPart *pt=ros_dds;
  CRosDdsSrv *ds=s->dds;
  if (!pt || !ds)
    return;
  RosLockGet(&pt->lock);
  RtpsSedpAnnounce(pt,&ds->rq->guid,ds->rq->topic,ds->rq->type,TRUE,FALSE,FALSE,TRUE);
  RtpsSedpAnnounce(pt,&ds->rr->guid,ds->rr->topic,ds->rr->type,TRUE,FALSE,TRUE,TRUE);
  RtpsReaderDel(pt,ds->rq);
  RtpsWriterDel(pt,ds->rr);
  s->dds=NULL;
  ros_dds_info_dirty=TRUE;
  RosLockPut(&pt->lock);
  Free(ds);
}

//A reply for one of our clients.
U0 RosDdsOnReply(CRtpsReader *r,CRtpsGuid *from,CRtpsChange *ch)
{
  CRosClient *c=r->user;
  CRosDdsCli *dc=c->dds;
  CRosPending *p;
  U8 *m=RosMsgNew(c->st->resp);
  I64 client,seq,i;
  if (!dc || !RosDdsSrvDeser(c->st->resp,ch,m,&client,&seq) || client!=dc->id) {
    RosMsgDel(c->st->resp,m);
    return;
  }
  RosLockGet(&c->lock);
  for (i=0;i<c->pending->len;i++) {
    p=RosVecGetI64(c->pending,i);
    if (p->seq==seq && !p->done) {
      p->resp=m;
      p->done=TRUE;
      m=NULL;
      break;
    }
  }
  RosLockPut(&c->lock);
  if (m)
    RosMsgDel(c->st->resp,m);
}

U0 RosDdsCliNew(CRosClient *c)
{
  CRtpsPart *pt=ros_dds;
  CRosDdsCli *dc;
  U8 *t1,*t2,*y1,*y2;
  I64 i;
  if (!pt)
    return;
  dc=CAlloc(sizeof(CRosDdsCli));
  t1=RosDdsSrvTopic("rq",c->name,"Request");
  t2=RosDdsSrvTopic("rr",c->name,"Reply");
  y1=RosDdsTypeName(c->st->req);
  y2=RosDdsTypeName(c->st->resp);
  RosLockGet(&pt->lock);
  dc->rq=RtpsWriterNew(pt,RosDdsNewEid(pt,RTPS_KIND_WRITER_NOKEY),t1,y1,TRUE,FALSE,1000);
  dc->rr=RtpsReaderNew(pt,RosDdsNewEid(pt,RTPS_KIND_READER_NOKEY),t2,y2,TRUE,FALSE,
	&RosDdsOnReply,c);
  for (i=4;i<12;i++)	//client id: from our writer's guid
    dc->id=(dc->id<<8)|dc->rq->guid.prefix[i];
  dc->id=dc->id^dc->rq->guid.eid;
  RtpsSedpAnnounce(pt,&dc->rq->guid,t1,y1,TRUE,FALSE,TRUE,FALSE);
  RtpsSedpAnnounce(pt,&dc->rr->guid,t2,y2,TRUE,FALSE,FALSE,FALSE);
  RtpsMatchLocal(pt,dc->rq,dc->rr);
  c->dds=dc;
  ros_dds_info_dirty=TRUE;
  RosLockPut(&pt->lock);
  Free(t1);
  Free(t2);
  Free(y1);
  Free(y2);
}

U0 RosDdsCliDel(CRosClient *c)
{
  CRtpsPart *pt=ros_dds;
  CRosDdsCli *dc=c->dds;
  if (!pt || !dc)
    return;
  RosLockGet(&pt->lock);
  RtpsSedpAnnounce(pt,&dc->rq->guid,dc->rq->topic,dc->rq->type,TRUE,FALSE,TRUE,TRUE);
  RtpsSedpAnnounce(pt,&dc->rr->guid,dc->rr->topic,dc->rr->type,TRUE,FALSE,FALSE,TRUE);
  RtpsWriterDel(pt,dc->rq);
  RtpsReaderDel(pt,dc->rr);
  c->dds=NULL;
  ros_dds_info_dirty=TRUE;
  RosLockPut(&pt->lock);
  Free(dc);
}

//A remote server is there when both its topics matched ours AND its
//request reader has acked our writer (so it knows us and won't drop
//the first request: the usual DDS discovery race).
Bool RosDdsCliReady(CRosClient *c)
{
  CRtpsPart *pt=ros_dds;
  CRosDdsCli *dc=c->dds;
  CRtpsReaderProxy *rp;
  Bool ok=FALSE;
  I64 i;
  if (!pt || !dc)
    return FALSE;
  RosLockGet(&pt->lock);
  if (dc->rr->writers->len)
    for (i=0;i<dc->rq->readers->len;i++) {
      rp=RosVecGetI64(dc->rq->readers,i);
      if (rp->last_count)
        ok=TRUE;
    }
  RosLockPut(&pt->lock);
  return ok;
}

Bool RosDdsCliCall(CRosClient *c,I64 seq,U8 *req)
{
  CRtpsPart *pt=ros_dds;
  CRosDdsCli *dc=c->dds;
  CRosBuf *b;
  if (!pt || !dc)
    return FALSE;
  b=RosDdsSrvSer(c->st->req,req,dc->id,seq);
  RosLockGet(&pt->lock);
  RtpsWrite(pt,dc->rq,b->data,b->len,NULL,FALSE);
  RosLockPut(&pt->lock);
  b->owned=FALSE;
  RosBufDel(b);
  return TRUE;
}
