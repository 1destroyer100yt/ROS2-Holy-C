//Dds: a small DDS participant speaking RTPS: discovery (SPDP/SEDP),
//best-effort and reliable writers/readers (HEARTBEAT, ACKNACK, GAP),
//and DATA_FRAG reassembly. Enough to talk to Fast DDS / Cyclone DDS,
//which is what ROS 2 runs on. Bridge.HC connects it to the ROS graph.
//
//Everything is guarded by one lock (part->lock). Call RosDdsSpinOnce
//regularly (RosDdsStart runs a task for that in TempleOS).

#define RTPS_MAX_DGRAM		65000
#define RTPS_FRAG_SIZE		60000	//we send DATA_FRAG above this
#define RTPS_HB_PERIOD_NS	100000000	//heartbeat when data is unacked
#define RTPS_HB_IDLE_NS		1000000000	//...and now and then anyway
#define RTPS_SPDP_PERIOD_NS	1000000000
#define RTPS_LEASE_NS		20000000000
#define RTPS_MAX_PENDING	1024	//out-of-order samples kept per writer

Bool ros_dds_debug=FALSE;	//print protocol details

class CRtpsChange
{
  I64 seq;
  U8 *data;		//serialized payload incl. encapsulation header
  I64 len;
  U8 key[16];		//key hash (builtin topics), if has_key
  Bool has_key;
  Bool dispose;
  U8 rel[16];		//related sample identity (Fast DDS services): guid
  I64 rel_seq;		//  and sequence number (-1 = unknown)
  Bool has_rel;
  U8 src[16];		//received: the writer's guid
};

class CRtpsReaderProxy	//a remote reader one of our writers sends to
{
  CRtpsGuid guid;
  CRtpsLoc loc;
  Bool reliable;
  I64 acked;		//all seq <= acked are acknowledged
  I64 last_count;	//ACKNACK count seen
};

class CRtpsWriterProxy	//a remote writer one of our readers takes from
{
  CRtpsGuid guid;
  CRtpsLoc loc;
  Bool reliable;
  I64 expected;		//next seq to deliver
  CRosVec *pending;	//CRtpsChange * received out of order
  I64 hb_count;		//last HEARTBEAT count seen
  I64 acknack_count;
  I64 frag_seq,frag_total,frag_size,frag_got,frag_n;	//DATA_FRAG reassembly
  U8 *frag_buf,*frag_have;
};

class CRtpsWriter
{
  CRtpsGuid guid;
  U8 *topic,*type;
  Bool reliable,transient,builtin,keyed;
  I64 depth;		//history kept (transient local / resends)
  I64 seq;		//last seq written
  CRosVec *hist;	//CRtpsChange *
  CRosVec *readers;	//CRtpsReaderProxy *
  I64 hb_count,next_hb_ns;
  U8 *user;
};

class CRtpsReader
{
  CRtpsGuid guid;
  U8 *topic,*type;
  Bool reliable,transient,builtin;
  CRosVec *writers;	//CRtpsWriterProxy *
  U0 (*on_data)(U8 *r,CRtpsGuid *from,CRtpsChange *c);	//r is the CRtpsReader *
  U8 *user;
  I64 received;
};

class CRtpsRemotePart
{
  U8 prefix[12];
  CRtpsLoc meta_uc,user_uc;
  I64 lease_ns,last_ns,endpoints;
  I64 vendor;		//RTPS vendor id (RTPS_VENDOR_EPROSIMA = Fast DDS)
  U8 *name;
};

class CRtpsRemoteEp	//a remote writer or reader learned from SEDP
{
  CRtpsGuid guid;
  U8 *topic,*type;
  U8 *type_hash;	//"RIHS01_..." from USER_DATA (Jazzy+), or NULL
  CRtpsLoc loc;
  Bool is_writer,reliable,transient;
};

class CRtpsPart
{
  U8 prefix[12];
  I64 domain,pid;
  CRosUdp *meta_uc,*user_uc,*meta_mc,*user_mc;
  CRtpsLoc meta_loc,user_loc;
  Bool multicast;
  CRosVec *peers;	//I64 ips to send SPDP to (unicast discovery)
  CRosVec *remote;	//CRtpsRemotePart *
  CRosVec *remote_eps;	//CRtpsRemoteEp *
  CRosVec *writers,*readers;
  CRtpsWriter *spdp_w,*pub_w,*sub_w;
  CRtpsReader *spdp_r,*pub_r,*sub_r;
  I64 next_key,spdp_next_ns,lock;
  U8 *name;
  U8 *rx;		//receive buffer
  I64 sent,recvd,dropped_tx;	//datagram counters
  I64 test_drop_every;	//testing: drop every Nth user DATA we send
};

U0 RtpsChangeFree(CRtpsChange *c)
{
  if (!c)
    return;
  Free(c->data);
  Free(c);
}

//---- sending

U0 RtpsSendTo(CRtpsPart *pt,CRtpsLoc *l,CRosBuf *b)
{
  if (!l->port)
    return;
  RosUdpSend(pt->user_uc,l->ip,l->port,b->data,b->len);
  pt->sent++;
}

CRosBuf *RtpsMsgNew(CRtpsPart *pt,U8 *dst_prefix)
{
  CRosBuf *b=RosBufNew();
  RtpsWHeader(b,pt->prefix);
  if (dst_prefix)
    RtpsWInfoDst(b,dst_prefix);
  return b;
}

//DATA (or DATA_FRAGs if big) for change c, from writer w to reader rd
//(RTPS_EID_UNKNOWN = any reader).
U0 RtpsWData(CRosBuf *b,CRtpsWriter *w,I64 rd,CRtpsChange *c,I64 frag,I64 nfrag)
{
  I64 at,flags=0,q,fsz,off;
  if (c->has_key || c->has_rel)
    flags|=2;		//Q: inline QoS follows
  if (nfrag>1) {
    at=RtpsWSub(b,RTPS_DATA_FRAG,flags);
    RtpsW16(b,0);
    RtpsW16(b,28);	//octetsToInlineQos
  } else {
    if (c->data)
      flags|=4;		//D: payload follows
    at=RtpsWSub(b,RTPS_DATA,flags);
    RtpsW16(b,0);
    RtpsW16(b,16);
  }
  RtpsW32BE(b,rd);
  RtpsW32BE(b,w->guid.eid);
  RtpsWSn(b,c->seq);
  if (nfrag>1) {
    RtpsW32(b,frag+1);	//fragmentStartingNum (1-based)
    RtpsW16(b,1);	//fragmentsInSubmessage
    RtpsW16(b,RTPS_FRAG_SIZE);
    RtpsW32(b,c->len);	//sampleSize
  }
  if (c->has_key) {
    q=RtpsPlParam(b,PID_KEY_HASH);
    RtpsWBytes(b,c->key,16);
    RtpsPlEnd(b,q);
    if (c->dispose) {
      q=RtpsPlParam(b,PID_STATUS_INFO);
      RtpsW32BE(b,3);	//disposed | unregistered
      RtpsPlEnd(b,q);
    }
  }
  if (c->has_rel) {
    q=RtpsPlParam(b,PID_RELATED_SAMPLE_IDENTITY);
    RtpsWBytes(b,c->rel,16);
    RtpsW32(b,c->rel_seq>>32);	//high (signed)
    RtpsW32(b,c->rel_seq&0xFFFFFFFF);	//low
    RtpsPlEnd(b,q);
  }
  if (c->has_key || c->has_rel)
    RtpsPlEndList(b);
  if (nfrag>1) {
    off=frag*RTPS_FRAG_SIZE;
    fsz=c->len-off;
    if (fsz>RTPS_FRAG_SIZE)
      fsz=RTPS_FRAG_SIZE;
    RtpsWBytes(b,c->data+off,fsz);
  } else if (c->data)
    RtpsWBytes(b,c->data,c->len);
  RtpsWSubEnd(b,at);
}

I64 RtpsFragCount(CRtpsChange *c)
{
  if (c->len>RTPS_FRAG_SIZE)
    return (c->len+RTPS_FRAG_SIZE-1)/RTPS_FRAG_SIZE;
  return 1;
}

//Send fragment i (0-based) of change c to one reader proxy.
U0 RtpsSendFrag(CRtpsPart *pt,CRtpsWriter *w,CRtpsReaderProxy *rp,CRtpsChange *c,I64 i)
{
  CRosBuf *b;
  if (pt->test_drop_every && !w->builtin) {	//loss simulation
    pt->dropped_tx++;
    if (!(pt->dropped_tx%pt->test_drop_every))
      return;
  }
  b=RtpsMsgNew(pt,rp->guid.prefix);
  RtpsWInfoTs(b,RosTimeSystemNs());
  RtpsWData(b,w,rp->guid.eid,c,i,RtpsFragCount(c));
  RtpsSendTo(pt,&rp->loc,b);
  RosBufDel(b);
}

//Send change c (all its fragments) to one reader proxy.
U0 RtpsSendChange(CRtpsPart *pt,CRtpsWriter *w,CRtpsReaderProxy *rp,CRtpsChange *c)
{
  I64 i,n=RtpsFragCount(c);
  for (i=0;i<n;i++)
    RtpsSendFrag(pt,w,rp,c,i);
}

I64 RtpsFirstSeq(CRtpsWriter *w)
{
  CRtpsChange *c;
  if (!w->hist->len)
    return w->seq+1;
  c=RosVecGetI64(w->hist,0);
  return c->seq;
}

U0 RtpsSendHeartbeat(CRtpsPart *pt,CRtpsWriter *w,CRtpsReaderProxy *rp,Bool final)
{
  CRosBuf *b=RtpsMsgNew(pt,rp->guid.prefix);
  I64 at,first=RtpsFirstSeq(w),last=w->seq;
  if (first>last+1)
    first=last+1;
  w->hb_count++;
  if (final)
    at=RtpsWSub(b,RTPS_HEARTBEAT,2);
  else
    at=RtpsWSub(b,RTPS_HEARTBEAT,0);
  RtpsW32BE(b,rp->guid.eid);
  RtpsW32BE(b,w->guid.eid);
  RtpsWSn(b,first);
  RtpsWSn(b,last);
  RtpsW32(b,w->hb_count);
  RtpsWSubEnd(b,at);
  RtpsSendTo(pt,&rp->loc,b);
  RosBufDel(b);
}

//GAP: tell rp that [start,end) won't come.
U0 RtpsSendGap(CRtpsPart *pt,CRtpsWriter *w,CRtpsReaderProxy *rp,I64 start,I64 end)
{
  CRosBuf *b;
  I64 at;
  if (end<=start)
    return;
  b=RtpsMsgNew(pt,rp->guid.prefix);
  at=RtpsWSub(b,RTPS_GAP,0);
  RtpsW32BE(b,rp->guid.eid);
  RtpsW32BE(b,w->guid.eid);
  RtpsWSn(b,start);
  RtpsWSn(b,end);	//gapList base: first seq after the gap
  RtpsW32(b,0);		//numBits
  RtpsWSubEnd(b,at);
  RtpsSendTo(pt,&rp->loc,b);
  RosBufDel(b);
}

//Our reader asks a remote writer for what's missing (or just acks).
U0 RtpsSendAckNack(CRtpsPart *pt,CRtpsReader *r,CRtpsWriterProxy *wp,I64 last)
{
  CRosBuf *b=RtpsMsgNew(pt,wp->guid.prefix);
  I64 at,base=wp->expected,nbits=0,i,j,s,words[8];
  if (base<1)
    base=1;		//not synced yet: sequence numbers start at 1
  CRtpsChange *c;
  Bool have;
  MemSet(words,0,sizeof(I64)*8);
  if (last>=base) {
    nbits=last-base+1;
    if (nbits>256)
      nbits=256;
    for (i=0;i<nbits;i++) {
      s=base+i;
      have=FALSE;
      for (j=0;j<wp->pending->len;j++) {
        c=RosVecGetI64(wp->pending,j);
        if (c->seq==s)
          have=TRUE;
      }
      if (!have)
        words[i/32]=words[i/32]|(1<<(31-i%32));
    }
  }
  wp->acknack_count++;
  if (ros_dds_debug)
    Print("acknack: base %d nbits %d last %d\n",base,nbits,last);
  at=RtpsWSub(b,RTPS_ACKNACK,2);	//F: final
  RtpsW32BE(b,r->guid.eid);
  RtpsW32BE(b,wp->guid.eid);
  RtpsWSn(b,base);
  RtpsW32(b,nbits);
  for (i=0;i<(nbits+31)/32;i++)
    RtpsW32(b,words[i]);
  RtpsW32(b,wp->acknack_count);
  RtpsWSubEnd(b,at);
  RtpsSendTo(pt,&wp->loc,b);
  RosBufDel(b);
}

//Ask for the fragments of a half-received sample we're missing.
U0 RtpsSendNackFrag(CRtpsPart *pt,CRtpsReader *r,CRtpsWriterProxy *wp)
{
  CRosBuf *b;
  I64 at,i,base=0,nbits=0,words[8];
  if (!wp->frag_seq || !wp->frag_have)
    return;
  for (i=0;i<wp->frag_n;i++)
    if (!wp->frag_have[i]) {
      base=i;
      break;
    }
  if (i==wp->frag_n)
    return;
  MemSet(words,0,sizeof(I64)*8);
  for (i=base;i<wp->frag_n && nbits<256;i++) {
    if (!wp->frag_have[i])
      words[nbits/32]=words[nbits/32]|(1<<(31-nbits%32));
    nbits++;
  }
  b=RtpsMsgNew(pt,wp->guid.prefix);
  wp->acknack_count++;
  at=RtpsWSub(b,RTPS_NACK_FRAG,0);
  RtpsW32BE(b,r->guid.eid);
  RtpsW32BE(b,wp->guid.eid);
  RtpsWSn(b,wp->frag_seq);
  RtpsW32(b,base+1);	//fragment numbers are 1-based
  RtpsW32(b,nbits);
  for (i=0;i<(nbits+31)/32;i++)
    RtpsW32(b,words[i]);
  RtpsW32(b,wp->acknack_count);
  RtpsWSubEnd(b,at);
  RtpsSendTo(pt,&wp->loc,b);
  RosBufDel(b);
}

//---- writers

CRtpsWriter *RtpsWriterNew(CRtpsPart *pt,I64 eid,U8 *topic,U8 *type,Bool reliable,
	Bool transient,I64 depth)
{
  CRtpsWriter *w=CAlloc(sizeof(CRtpsWriter));
  MemCpy(w->guid.prefix,pt->prefix,12);
  w->guid.eid=eid;
  w->topic=StrNew(topic);
  w->type=StrNew(type);
  w->reliable=reliable;
  w->transient=transient;
  w->depth=depth;
  if (w->depth<1)
    w->depth=1;
  w->hist=RosVecNew(8);
  w->readers=RosVecNew(8);
  RosVecPushI64(pt->writers,w);
  return w;
}

//Add a sample. Takes ownership of data. Sends it to matched readers.
//rel (16-byte guid) + rel_seq: a related sample identity to send in
//inline QoS (Fast DDS service requests and replies), or rel=NULL.
I64 RtpsWriteRel(CRtpsPart *pt,CRtpsWriter *w,U8 *data,I64 len,U8 *key,Bool dispose,U8 *rel,
	I64 rel_seq)
{
  CRtpsChange *c=CAlloc(sizeof(CRtpsChange)),*old;
  CRtpsReaderProxy *rp;
  I64 i;
  w->seq++;
  c->seq=w->seq;
  c->data=data;
  c->len=len;
  c->dispose=dispose;
  if (key) {
    MemCpy(c->key,key,16);
    c->has_key=TRUE;
  }
  if (rel) {
    MemCpy(c->rel,rel,16);
    c->rel_seq=rel_seq;
    c->has_rel=TRUE;
  }
  for (i=0;i<w->readers->len;i++) {
    rp=RosVecGetI64(w->readers,i);
    RtpsSendChange(pt,w,rp,c);
  }
  RosVecPushI64(w->hist,c);
  //keep depth samples (builtin keyed writers keep one per key: see SEDP)
  while (!w->keyed && w->hist->len>w->depth) {
    old=RosVecGetI64(w->hist,0);
    RosVecRemove(w->hist,0);
    RtpsChangeFree(old);
  }
  if (w->reliable)
    w->next_hb_ns=RosTimeSteadyNs();	//heartbeat soon
  return c->seq;
}

I64 RtpsWrite(CRtpsPart *pt,CRtpsWriter *w,U8 *data,I64 len,U8 *key,Bool dispose)
{
  return RtpsWriteRel(pt,w,data,len,key,dispose,NULL,0);
}

CRtpsChange *RtpsWriterFind(CRtpsWriter *w,I64 seq)
{
  I64 i;
  CRtpsChange *c;
  for (i=0;i<w->hist->len;i++) {
    c=RosVecGetI64(w->hist,i);
    if (c->seq==seq)
      return c;
  }
  return NULL;
}

CRtpsReaderProxy *RtpsWriterProxyFor(CRtpsWriter *w,CRtpsGuid *g)
{
  I64 i;
  CRtpsReaderProxy *rp;
  for (i=0;i<w->readers->len;i++) {
    rp=RosVecGetI64(w->readers,i);
    if (RtpsGuidEq(&rp->guid,g))
      return rp;
  }
  return NULL;
}

U0 RtpsWriterMatch(CRtpsPart *pt,CRtpsWriter *w,CRtpsGuid *g,CRtpsLoc *loc,Bool reliable)
{
  CRtpsReaderProxy *rp=RtpsWriterProxyFor(w,g);
  CRtpsChange *c;
  I64 i;
  if (rp)
    return;
  rp=CAlloc(sizeof(CRtpsReaderProxy));
  MemCpy(&rp->guid,g,sizeof(CRtpsGuid));
  MemCpy(&rp->loc,loc,sizeof(CRtpsLoc));
  rp->reliable=reliable && w->reliable;
  RosVecPushI64(w->readers,rp);
  if (w->transient) {
    //late joiner: send what we keep
    rp->acked=RtpsFirstSeq(w)-1;
    for (i=0;i<w->hist->len;i++) {
      c=RosVecGetI64(w->hist,i);
      RtpsSendChange(pt,w,rp,c);
    }
  } else {
    //volatile: old samples are not for this reader
    rp->acked=w->seq;
    if (rp->reliable)
      RtpsSendGap(pt,w,rp,1,w->seq+1);
  }
  if (rp->reliable)
    RtpsSendHeartbeat(pt,w,rp,FALSE);
}

U0 RtpsWriterUnmatchPrefix(CRtpsWriter *w,U8 *prefix)
{
  I64 i=0;
  CRtpsReaderProxy *rp;
  while (i<w->readers->len) {
    rp=RosVecGetI64(w->readers,i);
    if (RtpsPrefixEq(rp->guid.prefix,prefix)) {
      RosVecRemove(w->readers,i);
      Free(rp);
    } else
      i++;
  }
}

U0 RtpsWriterDel(CRtpsPart *pt,CRtpsWriter *w)
{
  I64 i;
  RosVecRemovePtr(pt->writers,w);
  for (i=0;i<w->hist->len;i++)
    RtpsChangeFree(RosVecGetI64(w->hist,i));
  for (i=0;i<w->readers->len;i++)
    Free(RosVecGetI64(w->readers,i));
  RosVecDel(w->hist);
  RosVecDel(w->readers);
  Free(w->topic);
  Free(w->type);
  Free(w);
}

//ACKNACK from a remote reader.
U0 RtpsOnAckNack(CRtpsPart *pt,CRtpsWriter *w,CRtpsReaderProxy *rp,I64 base,I64 nbits,
	U8 *bits,Bool be,I64 count)
{
  I64 i,s,word=0,gap_start=0;
  CRtpsChange *c;
  CRtpsIn in;
  if (count<=rp->last_count)
    return;	//old or repeated
  rp->last_count=count;
  if (base-1>rp->acked)
    rp->acked=base-1;
  RtpsInInit(&in,bits,(nbits+31)/32*4,be);
  for (i=0;i<nbits;i++) {
    if (!(i%32))
      word=RtpsRN(&in,4);
    if (!(word&(1<<(31-i%32))))
      goto next_dds_1;
    s=base+i;
    c=RtpsWriterFind(w,s);
    if (c) {
      if (gap_start) {
        RtpsSendGap(pt,w,rp,gap_start,s);
        gap_start=0;
      }
      RtpsSendChange(pt,w,rp,c);
    } else if (!gap_start)
      gap_start=s;
    next_dds_1:;
  }
  if (gap_start)
    RtpsSendGap(pt,w,rp,gap_start,base+nbits);
}

//---- readers

CRtpsReader *RtpsReaderNew(CRtpsPart *pt,I64 eid,U8 *topic,U8 *type,Bool reliable,
	Bool transient,U0 (*on_data)(U8 *r,CRtpsGuid *from,CRtpsChange *c),U8 *user)
{
  CRtpsReader *r=CAlloc(sizeof(CRtpsReader));
  MemCpy(r->guid.prefix,pt->prefix,12);
  r->guid.eid=eid;
  r->topic=StrNew(topic);
  r->type=StrNew(type);
  r->reliable=reliable;
  r->transient=transient;
  r->writers=RosVecNew(8);
  r->on_data=on_data;
  r->user=user;
  RosVecPushI64(pt->readers,r);
  return r;
}

CRtpsWriterProxy *RtpsReaderProxyFor(CRtpsReader *r,U8 *prefix,I64 eid)
{
  I64 i;
  CRtpsWriterProxy *wp;
  for (i=0;i<r->writers->len;i++) {
    wp=RosVecGetI64(r->writers,i);
    if (wp->guid.eid==eid && RtpsPrefixEq(wp->guid.prefix,prefix))
      return wp;
  }
  return NULL;
}

U0 RtpsWpFree(CRtpsWriterProxy *wp)
{
  I64 i;
  for (i=0;i<wp->pending->len;i++)
    RtpsChangeFree(RosVecGetI64(wp->pending,i));
  RosVecDel(wp->pending);
  Free(wp->frag_buf);
  Free(wp->frag_have);
  Free(wp);
}

U0 RtpsReaderMatch(CRtpsPart *pt,CRtpsReader *r,CRtpsGuid *g,CRtpsLoc *loc,Bool reliable)
{
  CRtpsWriterProxy *wp=RtpsReaderProxyFor(r,g->prefix,g->eid);
  if (wp)
    return;
  wp=CAlloc(sizeof(CRtpsWriterProxy));
  MemCpy(&wp->guid,g,sizeof(CRtpsGuid));
  MemCpy(&wp->loc,loc,sizeof(CRtpsLoc));
  wp->reliable=reliable && r->reliable;
  wp->expected=0;	//0: not synced yet, take the first we see
  wp->pending=RosVecNew(8);
  RosVecPushI64(r->writers,wp);
  if (wp->reliable)	//say hello so the writer heartbeats us
    RtpsSendAckNack(pt,r,wp,-1);
}

U0 RtpsReaderUnmatchPrefix(CRtpsReader *r,U8 *prefix)
{
  I64 i=0;
  CRtpsWriterProxy *wp;
  while (i<r->writers->len) {
    wp=RosVecGetI64(r->writers,i);
    if (RtpsPrefixEq(wp->guid.prefix,prefix)) {
      RosVecRemove(r->writers,i);
      RtpsWpFree(wp);
    } else
      i++;
  }
}

U0 RtpsReaderDel(CRtpsPart *pt,CRtpsReader *r)
{
  I64 i;
  RosVecRemovePtr(pt->readers,r);
  for (i=0;i<r->writers->len;i++)
    RtpsWpFree(RosVecGetI64(r->writers,i));
  RosVecDel(r->writers);
  Free(r->topic);
  Free(r->type);
  Free(r);
}

U0 RtpsDeliver(CRtpsReader *r,CRtpsWriterProxy *wp,CRtpsChange *c)
{
  r->received++;
  if (r->on_data)
    (*r->on_data)(r,&wp->guid,c);
}

//Deliver whatever is now in order.
U0 RtpsDrain(CRtpsReader *r,CRtpsWriterProxy *wp)
{
  I64 i;
  Bool more=TRUE;
  CRtpsChange *c;
  while (more) {
    more=FALSE;
    for (i=0;i<wp->pending->len;i++) {
      c=RosVecGetI64(wp->pending,i);
      if (c->seq<wp->expected) {	//skipped (GAP)
        RosVecRemove(wp->pending,i);
        RtpsChangeFree(c);
        more=TRUE;
        break;
      }
      if (c->seq==wp->expected) {
        RosVecRemove(wp->pending,i);
        RtpsDeliver(r,wp,c);
        RtpsChangeFree(c);
        wp->expected++;
        more=TRUE;
        break;
      }
    }
  }
}

//A complete sample arrived from wp. Takes ownership of c.
U0 RtpsOnChange(CRtpsReader *r,CRtpsWriterProxy *wp,CRtpsChange *c)
{
  I64 i;
  CRtpsChange *o;
  if (!wp->expected)	//first one we see from this writer
    wp->expected=c->seq;
  if (!wp->reliable) {
    if (c->seq>=wp->expected) {
      wp->expected=c->seq+1;
      RtpsDeliver(r,wp,c);
    }
    RtpsChangeFree(c);
    return;
  }
  if (c->seq<wp->expected) {	//duplicate
    RtpsChangeFree(c);
    return;
  }
  if (c->seq==wp->expected) {
    RtpsDeliver(r,wp,c);
    RtpsChangeFree(c);
    wp->expected++;
    RtpsDrain(r,wp);
    return;
  }
  for (i=0;i<wp->pending->len;i++) {
    o=RosVecGetI64(wp->pending,i);
    if (o->seq==c->seq) {
      RtpsChangeFree(c);
      return;
    }
  }
  if (wp->pending->len>=RTPS_MAX_PENDING) {
    RtpsChangeFree(c);
    return;
  }
  RosVecPushI64(wp->pending,c);
}

//---- the participant

CRtpsPart *ros_dds=NULL;

U8 *RtpsTopicNameDup(U8 *s)
{
  return StrNew(s);
}

//Remote participant by prefix, or NULL.
CRtpsRemotePart *RtpsRemoteFind(CRtpsPart *pt,U8 *prefix)
{
  I64 i;
  CRtpsRemotePart *rp;
  for (i=0;i<pt->remote->len;i++) {
    rp=RosVecGetI64(pt->remote,i);
    if (RtpsPrefixEq(rp->prefix,prefix))
      return rp;
  }
  return NULL;
}

Bool RtpsQosOk(Bool w_rel,Bool w_tl,Bool r_rel,Bool r_tl)
{
  if (r_rel && !w_rel)
    return FALSE;
  if (r_tl && !w_tl)
    return FALSE;
  return TRUE;
}

//Match a remote endpoint against our user endpoints.
U0 RtpsMatchRemoteEp(CRtpsPart *pt,CRtpsRemoteEp *e)
{
  I64 i;
  CRtpsWriter *w;
  CRtpsReader *r;
  if (e->is_writer) {
    for (i=0;i<pt->readers->len;i++) {
      r=RosVecGetI64(pt->readers,i);
      if (!r->builtin && !StrCmp(r->topic,e->topic) && !StrCmp(r->type,e->type) &&
	  RtpsQosOk(e->reliable,e->transient,r->reliable,r->transient))
        RtpsReaderMatch(pt,r,&e->guid,&e->loc,e->reliable);
    }
  } else {
    for (i=0;i<pt->writers->len;i++) {
      w=RosVecGetI64(pt->writers,i);
      if (!w->builtin && !StrCmp(w->topic,e->topic) && !StrCmp(w->type,e->type) &&
	  RtpsQosOk(w->reliable,w->transient,e->reliable,e->transient))
        RtpsWriterMatch(pt,w,&e->guid,&e->loc,e->reliable);
    }
  }
}

//Match one of our user endpoints against everything remote.
U0 RtpsMatchLocal(CRtpsPart *pt,CRtpsWriter *w,CRtpsReader *r)
{
  I64 i;
  CRtpsRemoteEp *e;
  for (i=0;i<pt->remote_eps->len;i++) {
    e=RosVecGetI64(pt->remote_eps,i);
    if (w && !e->is_writer && !StrCmp(w->topic,e->topic) && !StrCmp(w->type,e->type) &&
	RtpsQosOk(w->reliable,w->transient,e->reliable,e->transient))
      RtpsWriterMatch(pt,w,&e->guid,&e->loc,e->reliable);
    if (r && e->is_writer && !StrCmp(r->topic,e->topic) && !StrCmp(r->type,e->type) &&
	RtpsQosOk(e->reliable,e->transient,r->reliable,r->transient))
      RtpsReaderMatch(pt,r,&e->guid,&e->loc,e->reliable);
  }
}

U0 RtpsRemoteEpFree(CRtpsRemoteEp *e)
{
  Free(e->topic);
  Free(e->type);
  Free(e->type_hash);
  Free(e);
}

//Forget a remote participant and everything it had.
U0 RtpsRemoteDrop(CRtpsPart *pt,CRtpsRemotePart *rp)
{
  I64 i=0;
  CRtpsRemoteEp *e;
  for (i=0;i<pt->writers->len;i++)
    RtpsWriterUnmatchPrefix(RosVecGetI64(pt->writers,i),rp->prefix);
  for (i=0;i<pt->readers->len;i++)
    RtpsReaderUnmatchPrefix(RosVecGetI64(pt->readers,i),rp->prefix);
  i=0;
  while (i<pt->remote_eps->len) {
    e=RosVecGetI64(pt->remote_eps,i);
    if (RtpsPrefixEq(e->guid.prefix,rp->prefix)) {
      RosVecRemove(pt->remote_eps,i);
      RtpsRemoteEpFree(e);
    } else
      i++;
  }
  RosVecRemovePtr(pt->remote,rp);
  Free(rp->name);
  Free(rp);
}

//---- SPDP (participant discovery)

U8 *RtpsSpdpData(CRtpsPart *pt,I64 *len)
{
  CRosBuf *b=RosBufNew();
  I64 at;
  U8 *d;
  CRtpsLoc mc;
  RtpsPlBegin(b);
  at=RtpsPlParam(b,PID_PROTOCOL_VERSION);
  RtpsW8(b,2);
  RtpsW8(b,3);
  RtpsPlEnd(b,at);
  at=RtpsPlParam(b,PID_VENDORID);
  RtpsW8(b,RTPS_VENDOR_HI);
  RtpsW8(b,RTPS_VENDOR_LO);
  RtpsPlEnd(b,at);
  RtpsPlGuid(b,PID_PARTICIPANT_GUID,pt->prefix,RTPS_EID_PARTICIPANT);
  RtpsPlU32(b,PID_BUILTIN_ENDPOINTS,RTPS_BUILTIN_SET);
  RtpsPlU32(b,PID_DOMAIN_ID,pt->domain);
  RtpsPlLoc(b,PID_META_UC_LOC,&pt->meta_loc);
  RtpsPlLoc(b,PID_DEFAULT_UC_LOC,&pt->user_loc);
  if (pt->multicast) {
    mc.ip=RTPS_MC_ADDR;
    mc.port=RtpsPortSpdpMc(pt->domain);
    RtpsPlLoc(b,PID_META_MC_LOC,&mc);
    mc.port=RtpsPortUserMc(pt->domain);
    RtpsPlLoc(b,PID_DEFAULT_MC_LOC,&mc);
  }
  at=RtpsPlParam(b,PID_PARTICIPANT_LEASE);
  RtpsW32(b,RTPS_LEASE_NS/ROS_NS_PER_SEC);
  RtpsW32(b,0);
  RtpsPlEnd(b,at);
  RtpsPlOctets(b,PID_USER_DATA,"enclave=/;");
  if (pt->name)
    RtpsPlStr(b,PID_ENTITY_NAME,pt->name);
  RtpsPlEndList(b);
  *len=b->len;
  d=b->data;
  b->owned=FALSE;
  RosBufDel(b);
  return d;
}

//Send our participant data to everyone we know of.
U0 RtpsSpdpAnnounce(CRtpsPart *pt)
{
  CRtpsChange c;
  CRosBuf *b;
  CRtpsLoc l;
  CRtpsRemotePart *rp;
  I64 i,j;
  MemSet(&c,0,sizeof(CRtpsChange));
  pt->spdp_w->seq++;
  c.seq=pt->spdp_w->seq;
  c.data=RtpsSpdpData(pt,&c.len);
  MemCpy(c.key,pt->prefix,12);
  c.key[15]=0xC1;
  c.key[14]=1;
  c.has_key=TRUE;
  b=RtpsMsgNew(pt,NULL);
  RtpsWInfoTs(b,RosTimeSystemNs());
  RtpsWData(b,pt->spdp_w,RTPS_EID_SPDP_R,&c,0,1);
  if (pt->multicast) {
    l.ip=RTPS_MC_ADDR;
    l.port=RtpsPortSpdpMc(pt->domain);
    RtpsSendTo(pt,&l,b);
  }
  for (i=0;i<pt->peers->len;i++)	//unicast discovery: every likely index
    for (j=0;j<10;j++) {
      l.ip=RosVecGetI64(pt->peers,i);
      l.port=RtpsPortMetaUc(pt->domain,j);
      if (l.ip!=pt->meta_loc.ip || l.port!=pt->meta_loc.port)
        RtpsSendTo(pt,&l,b);
    }
  for (i=0;i<pt->remote->len;i++) {
    rp=RosVecGetI64(pt->remote,i);
    RtpsSendTo(pt,&rp->meta_uc,b);
  }
  RosBufDel(b);
  Free(c.data);
}

//Builtin SEDP endpoints of a new remote participant.
U0 RtpsMatchBuiltins(CRtpsPart *pt,CRtpsRemotePart *rp)
{
  CRtpsGuid g;
  MemCpy(g.prefix,rp->prefix,12);
  if (rp->endpoints&8) {	//publications detector
    g.eid=RTPS_EID_SEDP_PUB_R;
    RtpsWriterMatch(pt,pt->pub_w,&g,&rp->meta_uc,TRUE);
  }
  if (rp->endpoints&32) {	//subscriptions detector
    g.eid=RTPS_EID_SEDP_SUB_R;
    RtpsWriterMatch(pt,pt->sub_w,&g,&rp->meta_uc,TRUE);
  }
  if (rp->endpoints&4) {	//publications announcer
    g.eid=RTPS_EID_SEDP_PUB_W;
    RtpsReaderMatch(pt,pt->pub_r,&g,&rp->meta_uc,TRUE);
  }
  if (rp->endpoints&16) {	//subscriptions announcer
    g.eid=RTPS_EID_SEDP_SUB_W;
    RtpsReaderMatch(pt,pt->sub_r,&g,&rp->meta_uc,TRUE);
  }
}

U0 RtpsOnSpdp(CRtpsReader *r,CRtpsGuid *from,CRtpsChange *c)
{
  CRtpsPart *pt=r->user;
  CRtpsIn in,v;
  I64 pid,domain=pt->domain;
  CRtpsGuid g;
  CRtpsRemotePart *rp,tmp;
  Bool fresh=FALSE;
  if (c->len<4 || (c->data[1]!=3 && c->data[1]!=2))
    return;
  MemSet(&tmp,0,sizeof(CRtpsRemotePart));
  MemSet(&g,0,sizeof(CRtpsGuid));
  tmp.lease_ns=RTPS_LEASE_NS;
  RtpsInInit(&in,c->data+4,c->len-4,c->data[1]==2);
  while (RtpsPlNext(&in,&pid,&v)) {
    switch (pid) {
      case PID_PARTICIPANT_GUID: RtpsPlReadGuid(&v,&g); break;
      case PID_META_UC_LOC:
        if (!tmp.meta_uc.port)
          RtpsPlReadLoc(&v,&tmp.meta_uc);
        break;
      case PID_DEFAULT_UC_LOC:
        if (!tmp.user_uc.port)
          RtpsPlReadLoc(&v,&tmp.user_uc);
        break;
      case PID_BUILTIN_ENDPOINTS: tmp.endpoints=RtpsRN(&v,4); break;
      case PID_DOMAIN_ID: domain=RtpsRN(&v,4); break;
      case PID_PARTICIPANT_LEASE:
        tmp.lease_ns=RtpsRN(&v,4)*ROS_NS_PER_SEC;
        break;
      case PID_ENTITY_NAME: tmp.name=RtpsPlReadStr(&v); break;
      case PID_VENDORID:
        tmp.vendor=RtpsRN(&v,1)<<8;
        tmp.vendor|=RtpsRN(&v,1);
        break;
    }
  }
  if (domain!=pt->domain || RtpsPrefixEq(g.prefix,pt->prefix) || !tmp.meta_uc.port) {
    Free(tmp.name);
    return;
  }
  if (c->dispose) {	//participant leaving
    rp=RtpsRemoteFind(pt,g.prefix);
    if (rp)
      RtpsRemoteDrop(pt,rp);
    Free(tmp.name);
    return;
  }
  rp=RtpsRemoteFind(pt,g.prefix);
  if (!rp) {
    rp=CAlloc(sizeof(CRtpsRemotePart));
    MemCpy(rp->prefix,g.prefix,12);
    RosVecPushI64(pt->remote,rp);
    fresh=TRUE;
  } else
    Free(rp->name);
  MemCpy(&rp->meta_uc,&tmp.meta_uc,sizeof(CRtpsLoc));
  MemCpy(&rp->user_uc,&tmp.user_uc,sizeof(CRtpsLoc));
  if (!rp->user_uc.port)
    MemCpy(&rp->user_uc,&rp->meta_uc,sizeof(CRtpsLoc));
  rp->endpoints=tmp.endpoints;
  rp->vendor=tmp.vendor;
  rp->lease_ns=tmp.lease_ns;
  rp->name=tmp.name;
  rp->last_ns=RosTimeSteadyNs();
  if (fresh) {
    RtpsMatchBuiltins(pt,rp);
    pt->spdp_next_ns=0;	//introduce ourselves right away
  }
}

U0 RtpsWriterUnmatchGuid(CRtpsWriter *w,CRtpsGuid *g)
{
  CRtpsReaderProxy *rp=RtpsWriterProxyFor(w,g);
  if (rp) {
    RosVecRemovePtr(w->readers,rp);
    Free(rp);
  }
}

U0 RtpsReaderUnmatchGuid(CRtpsReader *r,CRtpsGuid *g)
{
  CRtpsWriterProxy *wp=RtpsReaderProxyFor(r,g->prefix,g->eid);
  if (wp) {
    RosVecRemovePtr(r->writers,wp);
    RtpsWpFree(wp);
  }
}

//---- SEDP (endpoint discovery)

//Jazzy puts "typehash=RIHS01_...;" in every endpoint's USER_DATA. The
//bridge sets this to look the hash up from a DDS type name.
U8 *(*rtps_hook_type_hash)(U8 *dds_type)=NULL;

U8 *RtpsSedpData(CRtpsPart *pt,CRtpsGuid *g,U8 *topic,U8 *type,Bool reliable,
	Bool transient,I64 *len)
{
  CRosBuf *b=RosBufNew();
  I64 at;
  U8 *d,*h,*ud;
  RtpsPlBegin(b);
  RtpsPlGuid(b,PID_ENDPOINT_GUID,g->prefix,g->eid);
  RtpsPlStr(b,PID_TOPIC_NAME,topic);
  RtpsPlStr(b,PID_TYPE_NAME,type);
  at=RtpsPlParam(b,PID_RELIABILITY);
  if (reliable)
    RtpsW32(b,2);
  else
    RtpsW32(b,1);
  RtpsW32(b,0);		//max_blocking_time 0.1 s
  RtpsW32(b,429496730);
  RtpsPlEnd(b,at);
  at=RtpsPlParam(b,PID_DURABILITY);
  if (transient)
    RtpsW32(b,1);
  else
    RtpsW32(b,0);
  RtpsPlEnd(b,at);
  RtpsPlLoc(b,PID_UNICAST_LOCATOR,&pt->user_loc);
  if (rtps_hook_type_hash) {
    h=(*rtps_hook_type_hash)(type);
    if (h) {
      ud=MStrPrint("typehash=%s;",h);
      RtpsPlOctets(b,PID_USER_DATA,ud);
      Free(ud);
    }
  }
  RtpsPlEndList(b);
  *len=b->len;
  d=b->data;
  b->owned=FALSE;
  RosBufDel(b);
  return d;
}

//Announce (or withdraw) one of our endpoints through SEDP.
U0 RtpsSedpAnnounce(CRtpsPart *pt,CRtpsGuid *g,U8 *topic,U8 *type,Bool reliable,
	Bool transient,Bool is_writer,Bool dispose)
{
  CRtpsWriter *sw=pt->sub_w;
  CRtpsChange *c;
  U8 *data,key[16];
  I64 len,i;
  if (is_writer)
    sw=pt->pub_w;
  MemCpy(key,g->prefix,12);
  key[12]=(g->eid>>24)&255;
  key[13]=(g->eid>>16)&255;
  key[14]=(g->eid>>8)&255;
  key[15]=g->eid&255;
  //one sample per endpoint: drop the old one from history
  for (i=0;i<sw->hist->len;i++) {
    c=RosVecGetI64(sw->hist,i);
    if (!MemCmp(c->key,key,16)) {
      RosVecRemove(sw->hist,i);
      RtpsChangeFree(c);
      break;
    }
  }
  if (dispose) {
    data=NULL;
    len=0;
  } else
    data=RtpsSedpData(pt,g,topic,type,reliable,transient,&len);
  RtpsWrite(pt,sw,data,len,key,dispose);
  if (dispose) {	//nothing to keep for a disposed endpoint
    c=RosVecGetI64(sw->hist,sw->hist->len-1);
    RosVecRemove(sw->hist,sw->hist->len-1);
    RtpsChangeFree(c);
  }
}

U0 RtpsOnSedp(CRtpsReader *r,CRtpsGuid *from,CRtpsChange *c)
{
  CRtpsPart *pt=r->user;
  CRtpsIn in,v;
  I64 pid,i,kind;
  CRtpsRemoteEp *e,*old;
  CRtpsRemotePart *rp;
  CRtpsGuid g;
  Bool have_guid=FALSE;
  if (c->dispose || !c->data) {	//endpoint gone: unmatch by key
    if (!c->has_key)
      return;
    MemCpy(g.prefix,c->key,12);
    g.eid=(c->key[12]<<24)|(c->key[13]<<16)|(c->key[14]<<8)|c->key[15];
    for (i=0;i<pt->remote_eps->len;i++) {
      old=RosVecGetI64(pt->remote_eps,i);
      if (RtpsGuidEq(&old->guid,&g)) {
        RosVecRemove(pt->remote_eps,i);
        RtpsRemoteEpFree(old);
        break;
      }
    }
    for (i=0;i<pt->writers->len;i++)
      RtpsWriterUnmatchGuid(RosVecGetI64(pt->writers,i),&g);
    for (i=0;i<pt->readers->len;i++)
      RtpsReaderUnmatchGuid(RosVecGetI64(pt->readers,i),&g);
    return;
  }
  if (c->len<4 || (c->data[1]!=3 && c->data[1]!=2))
    return;
  e=CAlloc(sizeof(CRtpsRemoteEp));
  e->is_writer=r==pt->pub_r;
  e->reliable=e->is_writer;	//spec defaults: writers reliable, readers best effort
  RtpsInInit(&in,c->data+4,c->len-4,c->data[1]==2);
  while (RtpsPlNext(&in,&pid,&v)) {
    switch (pid) {
      case PID_ENDPOINT_GUID:
        RtpsPlReadGuid(&v,&e->guid);
        have_guid=TRUE;
        break;
      case PID_TOPIC_NAME: Free(e->topic); e->topic=RtpsPlReadStr(&v); break;
      case PID_TYPE_NAME: Free(e->type); e->type=RtpsPlReadStr(&v); break;
      case PID_USER_DATA:
        Free(e->type_hash);
        e->type_hash=RtpsPlReadUserKey(&v,"typehash");
        break;
      case PID_RELIABILITY:
        kind=RtpsRN(&v,4);
        e->reliable=kind==2;
        break;
      case PID_DURABILITY:
        kind=RtpsRN(&v,4);
        e->transient=kind>=1;
        break;
      case PID_UNICAST_LOCATOR:
        if (!e->loc.port)
          RtpsPlReadLoc(&v,&e->loc);
        break;
    }
  }
  if (!have_guid || !e->topic || !e->type) {
    RtpsRemoteEpFree(e);
    return;
  }
  rp=RtpsRemoteFind(pt,e->guid.prefix);
  if (!e->loc.port && rp)
    MemCpy(&e->loc,&rp->user_uc,sizeof(CRtpsLoc));
  for (i=0;i<pt->remote_eps->len;i++) {	//already known: replace
    old=RosVecGetI64(pt->remote_eps,i);
    if (RtpsGuidEq(&old->guid,&e->guid)) {
      RosVecRemove(pt->remote_eps,i);
      RtpsRemoteEpFree(old);
      break;
    }
  }
  RosVecPushI64(pt->remote_eps,e);
  RtpsMatchRemoteEp(pt,e);
}


//---- receiving

class CRtpsCtx	//per incoming message
{
  U8 *src;	//sender's guid prefix
  Bool for_us;
};

//Pull key hash / status info out of inline QoS at in.
U0 RtpsReadInlineQos(CRtpsIn *in,CRtpsChange *c)
{
  I64 pid;
  CRtpsIn v;
  U8 *p;
  while (RtpsPlNext(in,&pid,&v)) {
    if (pid==PID_KEY_HASH) {
      p=RtpsRBytes(&v,16);
      if (p) {
        MemCpy(c->key,p,16);
        c->has_key=TRUE;
      }
    } else if (pid==PID_STATUS_INFO) {
      if (RtpsR32BE(&v)&3)
        c->dispose=TRUE;
    } else if ((pid==PID_RELATED_SAMPLE_IDENTITY || pid==PID_CUSTOM_RELATED_SAMPLE_IDENTITY) &&
	!c->has_rel) {
      p=RtpsRBytes(&v,16);
      if (p) {
        MemCpy(c->rel,p,16);
        c->rel_seq=RtpsRN(&v,4);	//high
        if (c->rel_seq&0x80000000)
          c->rel_seq-=0x100000000;
        c->rel_seq=(c->rel_seq<<32)|RtpsRN(&v,4);
        c->has_rel=!v.err;
      }
    }
  }
}

U0 RtpsOnDataSub(CRtpsPart *pt,CRtpsCtx *x,U8 *body,I64 n,I64 flags,Bool frag)
{
  CRtpsIn in;
  I64 otq,rid,wid,sn,i,fstart=0,fcount=0,fsize=0,total=0,off,k,len;
  CRtpsChange *c,base;
  CRtpsReader *r;
  CRtpsWriterProxy *wp;
  U8 *payload=NULL;
  Bool be=!(flags&1);
  RtpsInInit(&in,body,n,be);
  RtpsRN(&in,2);
  otq=RtpsRN(&in,2);
  rid=RtpsR32BE(&in);
  wid=RtpsR32BE(&in);
  sn=RtpsRSn(&in);
  if (frag) {
    fstart=RtpsRN(&in,4);
    fcount=RtpsRN(&in,2);
    fsize=RtpsRN(&in,2);
    total=RtpsRN(&in,4);
  }
  if (in.err)
    return;
  MemSet(&base,0,sizeof(CRtpsChange));
  base.seq=sn;
  MemCpy(base.src,x->src,12);
  base.src[12]=(wid>>24)&255;
  base.src[13]=(wid>>16)&255;
  base.src[14]=(wid>>8)&255;
  base.src[15]=wid&255;
  in.pos=4+otq;
  if (flags&2)
    RtpsReadInlineQos(&in,&base);
  if (in.err || in.pos>n)
    return;
  len=n-in.pos;
  if (frag || flags&12)	//D or K: payload
    payload=body+in.pos;
  else
    len=0;
  if (wid==RTPS_EID_SPDP_W) {	//stateless: no proxy
    if (frag || !payload)
      return;
    c=CAlloc(sizeof(CRtpsChange));
    MemCpy(c,&base,sizeof(CRtpsChange));
    c->data=MAlloc(len);
    MemCpy(c->data,payload,len);
    c->len=len;
    RtpsOnSpdp(pt->spdp_r,NULL,c);
    RtpsChangeFree(c);
    return;
  }
  for (i=0;i<pt->readers->len;i++) {
    r=RosVecGetI64(pt->readers,i);
    if (rid!=RTPS_EID_UNKNOWN && rid!=r->guid.eid)
      goto next_dds_2;
    wp=RtpsReaderProxyFor(r,x->src,wid);
    if (!wp)
      goto next_dds_2;
    if (frag) {
      if (total<=0 || total>64*1024*1024 || fsize<=0 || sn<wp->expected)
        goto next_dds_2;
      if (wp->frag_seq!=sn || wp->frag_total!=total) {
        Free(wp->frag_buf);
        Free(wp->frag_have);
        wp->frag_seq=sn;
        wp->frag_total=total;
        wp->frag_size=fsize;
        wp->frag_n=(total+fsize-1)/fsize;
        wp->frag_got=0;
        wp->frag_buf=MAlloc(total);
        wp->frag_have=CAlloc(wp->frag_n);
      }
      for (k=0;k<fcount;k++) {
        off=(fstart-1+k)*wp->frag_size;
        if (fstart-1+k>=wp->frag_n || off>=total)
          break;
        if (wp->frag_have[fstart-1+k])
          goto next_dds_3;
        if (off+wp->frag_size<=total) {
          if ((k+1)*wp->frag_size>len)
            break;
          MemCpy(wp->frag_buf+off,payload+k*wp->frag_size,wp->frag_size);
        } else {
          if (k*wp->frag_size+total-off>len)
            break;
          MemCpy(wp->frag_buf+off,payload+k*wp->frag_size,total-off);
        }
        wp->frag_have[fstart-1+k]=1;
        wp->frag_got++;
        next_dds_3:;
      }
      if (ros_dds_debug)
        Print("frag: seq %d start %d count %d size %d total %d got %d/%d expected %d\n",
	      sn,fstart,fcount,fsize,total,wp->frag_got,wp->frag_n,wp->expected);
      if (wp->frag_got<wp->frag_n)
        goto next_dds_2;
      c=CAlloc(sizeof(CRtpsChange));
      MemCpy(c,&base,sizeof(CRtpsChange));
      c->data=wp->frag_buf;
      c->len=total;
      wp->frag_buf=NULL;
      Free(wp->frag_have);
      wp->frag_have=NULL;
      wp->frag_seq=0;
      RtpsOnChange(r,wp,c);
      goto next_dds_2;
    }
    c=CAlloc(sizeof(CRtpsChange));
    MemCpy(c,&base,sizeof(CRtpsChange));
    if (payload) {
      c->data=MAlloc(len);
      MemCpy(c->data,payload,len);
      c->len=len;
    }
    RtpsOnChange(r,wp,c);
    next_dds_2:;
  }
}

U0 RtpsOnHeartbeatSub(CRtpsPart *pt,CRtpsCtx *x,U8 *body,I64 n,I64 flags)
{
  CRtpsIn in;
  I64 rid,wid,first,last,count,i;
  CRtpsReader *r;
  CRtpsWriterProxy *wp;
  RtpsInInit(&in,body,n,!(flags&1));
  rid=RtpsR32BE(&in);
  wid=RtpsR32BE(&in);
  first=RtpsRSn(&in);
  last=RtpsRSn(&in);
  count=RtpsRN(&in,4);
  if (in.err)
    return;
  for (i=0;i<pt->readers->len;i++) {
    r=RosVecGetI64(pt->readers,i);
    if (rid!=RTPS_EID_UNKNOWN && rid!=r->guid.eid)
      goto next_dds_4;
    wp=RtpsReaderProxyFor(r,x->src,wid);
    if (!wp || !wp->reliable || count<=wp->hb_count)
      goto next_dds_4;
    wp->hb_count=count;
    if (!wp->expected)
      wp->expected=first;
    if (first>wp->expected) {
      wp->expected=first;
      RtpsDrain(r,wp);
    }
    if (last>=wp->expected || !(flags&2))
      RtpsSendAckNack(pt,r,wp,last);
    if (wp->frag_seq && wp->frag_seq<=last)
      RtpsSendNackFrag(pt,r,wp);
    next_dds_4:;
  }
}

U0 RtpsOnHeartbeatFragSub(CRtpsPart *pt,CRtpsCtx *x,U8 *body,I64 n,I64 flags)
{
  CRtpsIn in;
  I64 rid,wid,sn,i;
  CRtpsReader *r;
  CRtpsWriterProxy *wp;
  RtpsInInit(&in,body,n,!(flags&1));
  rid=RtpsR32BE(&in);
  wid=RtpsR32BE(&in);
  sn=RtpsRSn(&in);
  if (in.err)
    return;
  for (i=0;i<pt->readers->len;i++) {
    r=RosVecGetI64(pt->readers,i);
    if (rid!=RTPS_EID_UNKNOWN && rid!=r->guid.eid)
      goto next_dds_5;
    wp=RtpsReaderProxyFor(r,x->src,wid);
    if (wp && wp->frag_seq==sn)
      RtpsSendNackFrag(pt,r,wp);
    next_dds_5:;
  }
}

//NACK_FRAG from a remote reader: resend the fragments it lists.
U0 RtpsOnNackFragSub(CRtpsPart *pt,CRtpsCtx *x,U8 *body,I64 n,I64 flags)
{
  CRtpsIn in,bin;
  I64 rid,wid,sn,base,nbits,i,j,word=0,nf;
  U8 *bits;
  CRtpsWriter *w;
  CRtpsReaderProxy *rp;
  CRtpsChange *c;
  CRtpsGuid g;
  RtpsInInit(&in,body,n,!(flags&1));
  rid=RtpsR32BE(&in);
  wid=RtpsR32BE(&in);
  sn=RtpsRSn(&in);
  base=RtpsRN(&in,4);
  nbits=RtpsRN(&in,4);
  if (nbits>256)
    return;
  bits=RtpsRBytes(&in,(nbits+31)/32*4);
  if (in.err)
    return;
  MemCpy(g.prefix,x->src,12);
  g.eid=rid;
  for (i=0;i<pt->writers->len;i++) {
    w=RosVecGetI64(pt->writers,i);
    if (w->guid.eid!=wid)
      goto next_dds_6;
    rp=RtpsWriterProxyFor(w,&g);
    c=RtpsWriterFind(w,sn);
    if (!rp || !c)
      goto next_dds_6;
    nf=RtpsFragCount(c);
    RtpsInInit(&bin,bits,(nbits+31)/32*4,!(flags&1));
    for (j=0;j<nbits;j++) {
      if (!(j%32))
        word=RtpsRN(&bin,4);
      if ((word&(1<<(31-j%32))) && base-1+j<nf)
        RtpsSendFrag(pt,w,rp,c,base-1+j);
    }
    next_dds_6:;
  }
}

U0 RtpsOnAckNackSub(CRtpsPart *pt,CRtpsCtx *x,U8 *body,I64 n,I64 flags)
{
  CRtpsIn in;
  I64 rid,wid,base,nbits,count,i;
  U8 *bits;
  CRtpsWriter *w;
  CRtpsReaderProxy *rp;
  CRtpsGuid g;
  RtpsInInit(&in,body,n,!(flags&1));
  rid=RtpsR32BE(&in);
  wid=RtpsR32BE(&in);
  base=RtpsRSn(&in);
  nbits=RtpsRN(&in,4);
  if (nbits>256)
    return;
  bits=RtpsRBytes(&in,(nbits+31)/32*4);
  count=RtpsRN(&in,4);
  if (in.err)
    return;
  MemCpy(g.prefix,x->src,12);
  g.eid=rid;
  for (i=0;i<pt->writers->len;i++) {
    w=RosVecGetI64(pt->writers,i);
    if (w->guid.eid!=wid)
      goto next_dds_7;
    rp=RtpsWriterProxyFor(w,&g);
    if (rp)
      RtpsOnAckNack(pt,w,rp,base,nbits,bits,!(flags&1),count);
    next_dds_7:;
  }
}

U0 RtpsOnGapSub(CRtpsPart *pt,CRtpsCtx *x,U8 *body,I64 n,I64 flags)
{
  CRtpsIn in,bin;
  I64 rid,wid,start,base,nbits,i,j,word=0;
  U8 *bits;
  CRtpsReader *r;
  CRtpsWriterProxy *wp;
  RtpsInInit(&in,body,n,!(flags&1));
  rid=RtpsR32BE(&in);
  wid=RtpsR32BE(&in);
  start=RtpsRSn(&in);
  base=RtpsRSn(&in);
  nbits=RtpsRN(&in,4);
  if (nbits>256)
    return;
  bits=RtpsRBytes(&in,(nbits+31)/32*4);
  if (in.err)
    return;
  for (i=0;i<pt->readers->len;i++) {
    r=RosVecGetI64(pt->readers,i);
    if (rid!=RTPS_EID_UNKNOWN && rid!=r->guid.eid)
      goto next_dds_8;
    wp=RtpsReaderProxyFor(r,x->src,wid);
    if (!wp)
      goto next_dds_8;
    if (!wp->expected)
      wp->expected=start;
    if (start<=wp->expected && base>wp->expected)
      wp->expected=base;
    RtpsInInit(&bin,bits,(nbits+31)/32*4,!(flags&1));
    for (j=0;j<nbits;j++) {
      if (!(j%32))
        word=RtpsRN(&bin,4);
      if ((word&(1<<(31-j%32))) && base+j==wp->expected)
        wp->expected++;
    }
    RtpsDrain(r,wp);
    next_dds_8:;
  }
}

U0 RtpsHandleMsg(CRtpsPart *pt,U8 *p,I64 len)
{
  CRtpsCtx x;
  I64 pos=20,id,flags,sub;
  U8 zero[12];
  if (len<20 || p[0]!='R' || p[1]!='T' || p[2]!='P' || p[3]!='S' || p[4]!=2)
    return;
  x.src=p+8;
  if (RtpsPrefixEq(x.src,pt->prefix))
    return;	//our own (multicast loopback)
  x.for_us=TRUE;
  MemSet(zero,0,12);
  while (pos+4<=len) {
    id=p[pos];
    flags=p[pos+1];
    if (flags&1)
      sub=p[pos+2]|(p[pos+3]<<8);
    else
      sub=(p[pos+2]<<8)|p[pos+3];
    if (!sub && id!=RTPS_PAD && id!=RTPS_INFO_TS)
      sub=len-pos-4;	//0 = to the end of the message
    if (pos+4+sub>len)
      break;
    if (ros_dds_debug)
      Print("rtps rx: sub 0x%X flags 0x%X len %d for_us %d\n",id,flags,sub,x.for_us);
    switch (id) {
      case RTPS_INFO_DST:
        if (sub>=12)
          x.for_us=RtpsPrefixEq(p+pos+4,pt->prefix) || RtpsPrefixEq(p+pos+4,zero);
        break;
      case RTPS_DATA:
        if (x.for_us)
          RtpsOnDataSub(pt,&x,p+pos+4,sub,flags,FALSE);
        break;
      case RTPS_DATA_FRAG:
        if (x.for_us)
          RtpsOnDataSub(pt,&x,p+pos+4,sub,flags,TRUE);
        break;
      case RTPS_HEARTBEAT:
        if (x.for_us)
          RtpsOnHeartbeatSub(pt,&x,p+pos+4,sub,flags);
        break;
      case RTPS_ACKNACK:
        if (x.for_us)
          RtpsOnAckNackSub(pt,&x,p+pos+4,sub,flags);
        break;
      case RTPS_GAP:
        if (x.for_us)
          RtpsOnGapSub(pt,&x,p+pos+4,sub,flags);
        break;
      case RTPS_HEARTBEAT_FRAG:
        if (x.for_us)
          RtpsOnHeartbeatFragSub(pt,&x,p+pos+4,sub,flags);
        break;
      case RTPS_NACK_FRAG:
        if (x.for_us)
          RtpsOnNackFragSub(pt,&x,p+pos+4,sub,flags);
        break;
    }
    pos+=4+sub;
  }
}

//---- participant lifetime

//New participant on domain d. multicast=FALSE: discovery only via
//peers added with RosDdsAddPeer (works without multicast routing).
CRtpsPart *RosDdsNew(I64 domain,U8 *name,Bool multicast)
{
  CRtpsPart *pt=CAlloc(sizeof(CRtpsPart));
  I64 i,t,addr=pt;
  pt->domain=domain;
  pt->multicast=multicast;
  if (name)
    pt->name=StrNew(name);
  //unique prefix: vendor + some time-based noise
  t=RosTimeSteadyNs()^RosTimeSystemNs()^addr;
  pt->prefix[0]=RTPS_VENDOR_HI;
  pt->prefix[1]=RTPS_VENDOR_LO;
  for (i=2;i<12;i++) {
    t=t*6364136223846793005+1442695040888963407;
    pt->prefix[i]=(t>>33)&255;
  }
  for (i=0;i<120 && !pt->meta_uc;i++) {
    pt->meta_uc=RosUdpOpen(RtpsPortMetaUc(domain,i),FALSE);
    if (pt->meta_uc) {
      pt->user_uc=RosUdpOpen(RtpsPortUserUc(domain,i),FALSE);
      if (!pt->user_uc) {
        RosUdpClose(pt->meta_uc);
        pt->meta_uc=NULL;
      } else
        pt->pid=i;
    }
  }
  if (!pt->meta_uc) {
    Free(pt);
    return NULL;
  }
  if (multicast) {
    pt->meta_mc=RosUdpOpen(RtpsPortSpdpMc(domain),TRUE);
    pt->user_mc=RosUdpOpen(RtpsPortUserMc(domain),TRUE);
    if (pt->meta_mc)
      RosUdpJoin(pt->meta_mc,RTPS_MC_ADDR);
    if (pt->user_mc)
      RosUdpJoin(pt->user_mc,RTPS_MC_ADDR);
  }
  pt->meta_loc.ip=ros_net_ip;
  pt->meta_loc.port=pt->meta_uc->port;
  pt->user_loc.ip=ros_net_ip;
  pt->user_loc.port=pt->user_uc->port;
  pt->peers=RosVecNew(8);
  pt->remote=RosVecNew(8);
  pt->remote_eps=RosVecNew(8);
  pt->writers=RosVecNew(8);
  pt->readers=RosVecNew(8);
  pt->rx=MAlloc(RTPS_MAX_DGRAM+16);
  pt->next_key=1;
  pt->spdp_w=RtpsWriterNew(pt,RTPS_EID_SPDP_W,"DCPSParticipant","ParticipantBuiltinTopicData",
	FALSE,FALSE,1);
  pt->pub_w=RtpsWriterNew(pt,RTPS_EID_SEDP_PUB_W,"DCPSPublication","PublicationBuiltinTopicData",
	TRUE,TRUE,1);
  pt->sub_w=RtpsWriterNew(pt,RTPS_EID_SEDP_SUB_W,"DCPSSubscription",
	"SubscriptionBuiltinTopicData",TRUE,TRUE,1);
  pt->spdp_r=RtpsReaderNew(pt,RTPS_EID_SPDP_R,"DCPSParticipant","ParticipantBuiltinTopicData",
	FALSE,FALSE,&RtpsOnSpdp,pt);
  pt->pub_r=RtpsReaderNew(pt,RTPS_EID_SEDP_PUB_R,"DCPSPublication",
	"PublicationBuiltinTopicData",TRUE,TRUE,&RtpsOnSedp,pt);
  pt->sub_r=RtpsReaderNew(pt,RTPS_EID_SEDP_SUB_R,"DCPSSubscription",
	"SubscriptionBuiltinTopicData",TRUE,TRUE,&RtpsOnSedp,pt);
  pt->spdp_w->builtin=TRUE;
  pt->pub_w->builtin=TRUE;
  pt->sub_w->builtin=TRUE;
  pt->pub_w->keyed=TRUE;
  pt->sub_w->keyed=TRUE;
  pt->spdp_r->builtin=TRUE;
  pt->pub_r->builtin=TRUE;
  pt->sub_r->builtin=TRUE;
  return pt;
}

//Also look for participants at this address (unicast discovery).
U0 RosDdsAddPeer(CRtpsPart *pt,I64 ip)
{
  RosLockGet(&pt->lock);
  RosVecPushI64(pt->peers,ip);
  pt->spdp_next_ns=0;
  RosLockPut(&pt->lock);
}

//Receive and handle everything waiting, send what's due. Returns how
//many datagrams were handled.
I64 RosDdsSpinOnce(CRtpsPart *pt)
{
  I64 n,ip,port,got=0,i,j,now;
  CRosUdp *socks[4];
  CRtpsWriter *w;
  CRtpsReaderProxy *rp;
  CRtpsRemotePart *rem;
  Bool unacked;
  RosLockGet(&pt->lock);
  socks[0]=pt->meta_uc;
  socks[1]=pt->user_uc;
  socks[2]=pt->meta_mc;
  socks[3]=pt->user_mc;
  for (i=0;i<4;i++) {
    if (!socks[i])
      goto next_dds_9;
    while (got<1000) {
      n=RosUdpRecv(socks[i],pt->rx,RTPS_MAX_DGRAM,&ip,&port);
      if (n<0)
        break;
      got++;
      pt->recvd++;
      RtpsHandleMsg(pt,pt->rx,n);
    }
    next_dds_9:;
  }
  now=RosTimeSteadyNs();
  if (now>=pt->spdp_next_ns) {
    RtpsSpdpAnnounce(pt);
    pt->spdp_next_ns=now+RTPS_SPDP_PERIOD_NS;
  }
  for (i=0;i<pt->writers->len;i++) {
    w=RosVecGetI64(pt->writers,i);
    if (!w->reliable || now<w->next_hb_ns)
      goto next_dds_10;
    unacked=FALSE;
    for (j=0;j<w->readers->len;j++) {
      rp=RosVecGetI64(w->readers,j);
      //unacked data, or a reader that hasn't acked us yet (it may not
      //know us yet: keep asking quickly until it does)
      if (rp->reliable && (rp->acked<w->seq || !rp->last_count)) {
        unacked=TRUE;
        RtpsSendHeartbeat(pt,w,rp,FALSE);
      }
    }
    if (unacked)
      w->next_hb_ns=now+RTPS_HB_PERIOD_NS;
    else
      w->next_hb_ns=now+RTPS_HB_IDLE_NS;
    next_dds_10:;
  }
  i=0;
  while (i<pt->remote->len) {	//leases
    rem=RosVecGetI64(pt->remote,i);
    if (now-rem->last_ns>rem->lease_ns)
      RtpsRemoteDrop(pt,rem);
    else
      i++;
  }
  RosLockPut(&pt->lock);
  return got;
}

//Tell everyone we're leaving, then free it all.
U0 RosDdsDel(CRtpsPart *pt)
{
  CRtpsChange c;
  CRosBuf *b;
  CRtpsRemotePart *rp;
  I64 i;
  if (!pt)
    return;
  RosLockGet(&pt->lock);
  MemSet(&c,0,sizeof(CRtpsChange));
  pt->spdp_w->seq++;
  c.seq=pt->spdp_w->seq;
  MemCpy(c.key,pt->prefix,12);
  c.key[14]=1;
  c.key[15]=0xC1;
  c.has_key=TRUE;
  c.dispose=TRUE;
  b=RtpsMsgNew(pt,NULL);
  RtpsWData(b,pt->spdp_w,RTPS_EID_SPDP_R,&c,0,1);
  for (i=0;i<pt->remote->len;i++) {
    rp=RosVecGetI64(pt->remote,i);
    RtpsSendTo(pt,&rp->meta_uc,b);
  }
  RosBufDel(b);
  while (pt->remote->len)
    RtpsRemoteDrop(pt,RosVecGetI64(pt->remote,0));
  while (pt->writers->len)
    RtpsWriterDel(pt,RosVecGetI64(pt->writers,0));
  while (pt->readers->len)
    RtpsReaderDel(pt,RosVecGetI64(pt->readers,0));
  RosVecDel(pt->writers);
  RosVecDel(pt->readers);
  RosVecDel(pt->remote);
  RosVecDel(pt->remote_eps);
  RosVecDel(pt->peers);
  RosUdpClose(pt->meta_uc);
  RosUdpClose(pt->user_uc);
  RosUdpClose(pt->meta_mc);
  RosUdpClose(pt->user_mc);
  Free(pt->rx);
  Free(pt->name);
  RosLockPut(&pt->lock);
  Free(pt);
}
