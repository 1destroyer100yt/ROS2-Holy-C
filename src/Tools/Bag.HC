//Bag: rosbag2 recording and playback in MCAP, the default format of
//ROS 2 Jazzy. A bag is a directory with name_0.mcap + metadata.yaml,
//so `ros2 bag info` / `ros2 bag play` on Linux read our bags, and we
//read theirs (uncompressed chunks; zstd/lz4 chunks aren't supported).
//
//  RosBagRecord("~/bag1","/scan /tf",0);   //until a key; NULL = all topics
//  CRosBagRec *r=RosBagRecordStart("~/bag1",NULL);  //or in the background:
//  ...spin RosNodeExec(r->node)...
//  RosBagRecordStop(r);
//  RosBagPlay("~/bag1",1.0);       //publish it again, at real speed
//  RosBagInfo("~/bag1");           //like ros2 bag info
//
//Messages are kept in memory and the file is written when recording
//stops: sorted by time, in chunks with message and chunk indexes, the
//same shape rosbag2 writes (so every MCAP reader can seek in it).

#define MCAP_HEADER	0x01
#define MCAP_FOOTER	0x02
#define MCAP_SCHEMA	0x03
#define MCAP_CHANNEL	0x04
#define MCAP_MESSAGE	0x05
#define MCAP_CHUNK	0x06
#define MCAP_MSGINDEX	0x07
#define MCAP_CHUNKINDEX	0x08
#define MCAP_STATS	0x0B
#define MCAP_SUMOFF	0x0E
#define MCAP_DATAEND	0x0F
#define MCAP_CHUNK_SIZE	786432	//bytes of messages per chunk, like rosbag2

class CRosBagChan
{
  I64 id,schema_id,count;
  U8 *topic,*type;
  CRosType *t;
  CRosPub *pub;	//playback
};

class CRosBagMsg
{
  I64 chan,stamp,len;
  U8 *data;
};

class CRosBag
{
  U8 *dir,*name;
  CRosBuf *out;		//writing
  CRosVec *chans;	//CRosBagChan *
  CRosVec *schemas;	//U8 * type names (schema id = index+1)
  CRosVec *msgs;	//CRosBagMsg *, sorted by time when writing
  I64 count,start,end;
  Bool writing;
};

U8 ros_bag_error[160];

U0 RosBagErr(U8 *what,U8 *detail)
{
  U8 *s=MStrPrint("%s: %s",what,detail);
  if (StrLen(s)>150)
    s[150]=0;
  StrCpy(ros_bag_error,s);
  Free(s);
}

//---- little-endian writers (MCAP is all little endian)

U0 McapW8(CRosBuf *b,I64 v)
{
  RosBufReserve(b,1);
  b->data[b->len]=v&255;
  b->len++;
}

U0 McapWN(CRosBuf *b,U64 v,I64 n)
{
  I64 i;
  for (i=0;i<n;i++) {
    McapW8(b,v);
    v=v>>8;
  }
}

U0 McapWStr(CRosBuf *b,U8 *s)
{
  I64 n=StrLen(s);
  McapWN(b,n,4);
  RosBufPutBytes(b,s,n);
}

//Start a record; returns where its length goes.
I64 McapRec(CRosBuf *b,I64 op)
{
  I64 at;
  McapW8(b,op);
  at=b->len;
  McapWN(b,0,8);
  return at;
}

U0 McapRecEnd(CRosBuf *b,I64 at)
{
  U64 n=b->len-at-8;
  I64 i;
  for (i=0;i<8;i++) {
    b->data[at+i]=n&255;
    n=n>>8;
  }
}

U0 McapMagic(CRosBuf *b)
{
  McapW8(b,0x89);
  RosBufPutBytes(b,"MCAP0\r\n",7);
}

//"geometry_msgs/msg/Twist" text for a schema: the .msg plus every
//type it uses, as rosbag2 writes it ("MSG: pkg/Name" blocks).
U0 RosBagSchemaDeps(CRosStrBuf *sb,CRosType *t,CRosVec *done)
{
  I64 i,j;
  CRosField *f;
  CRosType *d;
  Bool seen;
  for (i=0;i<t->fields->len;i++) {
    f=RosVecGetI64(t->fields,i);
    if (f->kind!=ROS_T_MSG)
      continue;
    seen=FALSE;
    for (j=0;j<done->len;j++) {
      d=RosVecGetI64(done,j);
      if (d==f->sub)
        seen=TRUE;
    }
    if (seen)
      continue;
    RosVecPushI64(done,f->sub);
    RosStrBufAdd(sb,"================================================================================\nMSG: ");
    RosStrBufAdd(sb,f->sub->pkg);
    RosStrBufAddC(sb,'/');
    RosStrBufAdd(sb,f->sub->base);
    RosStrBufAddC(sb,'\n');
    RosStrBufAdd(sb,f->sub->def);
    if (!RosStrEndsWith(f->sub->def,"\n"))
      RosStrBufAddC(sb,'\n');
    RosBagSchemaDeps(sb,f->sub,done);
  }
}

U8 *RosBagSchemaText(CRosType *t)
{
  CRosStrBuf *sb=RosStrBufNew();
  CRosVec *done=RosVecNew(8);
  RosStrBufAdd(sb,t->def);
  if (!RosStrEndsWith(t->def,"\n"))
    RosStrBufAddC(sb,'\n');
  RosBagSchemaDeps(sb,t,done);
  RosVecDel(done);
  return RosStrBufTake(sb);
}

U0 McapWSchema(CRosBuf *b,I64 id,CRosType *t)
{
  I64 at=McapRec(b,MCAP_SCHEMA);
  U8 *text=RosBagSchemaText(t);
  McapWN(b,id,2);
  McapWStr(b,t->name);
  McapWStr(b,"ros2msg");
  McapWStr(b,text);
  Free(text);
  McapRecEnd(b,at);
}

U0 McapWChannel(CRosBuf *b,CRosBagChan *c)
{
  I64 at=McapRec(b,MCAP_CHANNEL),m;
  McapWN(b,c->id,2);
  McapWN(b,c->schema_id,2);
  McapWStr(b,c->topic);
  McapWStr(b,"cdr");
  m=b->len;		//map<string,string>: byte length, then pairs
  McapWN(b,0,4);
  McapWStr(b,"offered_qos_profiles");
  McapWStr(b,"[]");
  b->data[m]=(b->len-m-4)&255;
  b->data[m+1]=((b->len-m-4)>>8)&255;
  McapRecEnd(b,at);
}

//---- writing

CRosBag *RosBagCreate(U8 *dir)
{
  CRosBag *bag=CAlloc(sizeof(CRosBag));
  I64 n=StrLen(dir),i=n,at;
  while (i>0 && dir[i-1]!='/')
    i--;
  bag->dir=StrNew(dir);
  bag->name=StrNew(dir+i);
  bag->chans=RosVecNew(8);
  bag->schemas=RosVecNew(8);
  bag->msgs=RosVecNew(8);
  bag->out=RosBufNew();
  bag->writing=TRUE;
  McapMagic(bag->out);
  at=McapRec(bag->out,MCAP_HEADER);
  McapWStr(bag->out,"ros2");
  McapWStr(bag->out,"ros2-holyc");
  McapRecEnd(bag->out,at);
  return bag;
}

CRosBagChan *RosBagChanFind(CRosBag *bag,U8 *topic)
{
  I64 i;
  CRosBagChan *c;
  for (i=0;i<bag->chans->len;i++) {
    c=RosVecGetI64(bag->chans,i);
    if (!StrCmp(c->topic,topic))
      return c;
  }
  return NULL;
}

//Add one message (msg of type t) at stamp (unix ns).
U0 RosBagWriteMsg(CRosBag *bag,U8 *topic,CRosType *t,I64 stamp,U8 *msg)
{
  CRosBagChan *c=RosBagChanFind(bag,topic);
  CRosBuf *cdr;
  CRosType *st;
  CRosBagMsg *m,*prev;
  I64 i,sid=0;
  if (!c) {
    for (i=0;i<bag->schemas->len;i++) {
      st=RosVecGetI64(bag->schemas,i);
      if (st==t)
        sid=i+1;
    }
    if (!sid) {
      RosVecPushI64(bag->schemas,t);
      sid=bag->schemas->len;
      McapWSchema(bag->out,sid,t);
    }
    c=CAlloc(sizeof(CRosBagChan));
    c->id=bag->chans->len+1;
    c->schema_id=sid;
    c->topic=StrNew(topic);
    c->type=StrNew(t->name);
    c->t=t;
    RosVecPushI64(bag->chans,c);
    McapWChannel(bag->out,c);
  }
  cdr=RosBufNew();
  RosCdrSer(t,msg,cdr);
  m=CAlloc(sizeof(CRosBagMsg));
  m->chan=c->id;
  m->stamp=stamp;
  m->len=cdr->len;
  m->data=MAlloc(cdr->len+1);
  MemCpy(m->data,cdr->data,cdr->len);
  RosBufDel(cdr);
  i=bag->msgs->len;	//keep them in time order (almost always an append)
  while (i>0) {
    prev=RosVecGetI64(bag->msgs,i-1);
    if (prev->stamp<=stamp)
      break;
    i--;
  }
  RosVecInsert(bag->msgs,i,&m);
  c->count++;
  if (!bag->count || stamp<bag->start)
    bag->start=stamp;
  if (stamp>bag->end)
    bag->end=stamp;
  bag->count++;
}

class CMcapChunkInfo
{
  I64 start,end,offset,len,idx_len,raw_len;
  CRosBuf *idx_offsets;	//map<u16,u64> body
};

//Write msgs[from,to) as one chunk + its message indexes.
U0 McapWChunk(CRosBag *bag,I64 from,I64 to,CMcapChunkInfo *ci)
{
  CRosBuf *b=bag->out,*recs=RosBufNew();
  CRosBagMsg *m;
  CRosBagChan *c;
  CRosVec *offs=RosVecNew(8);	//record offset of each message
  I64 i,j,at,idx_start,cnt;
  for (i=from;i<to;i++) {
    m=RosVecGetI64(bag->msgs,i);
    RosVecPushI64(offs,recs->len);
    at=McapRec(recs,MCAP_MESSAGE);
    McapWN(recs,m->chan,2);
    McapWN(recs,i,4);		//sequence
    McapWN(recs,m->stamp,8);	//log time
    McapWN(recs,m->stamp,8);	//publish time
    RosBufPutBytes(recs,m->data,m->len);
    McapRecEnd(recs,at);
  }
  m=RosVecGetI64(bag->msgs,from);
  ci->start=m->stamp;
  m=RosVecGetI64(bag->msgs,to-1);
  ci->end=m->stamp;
  ci->offset=b->len;
  ci->raw_len=recs->len;
  at=McapRec(b,MCAP_CHUNK);
  McapWN(b,ci->start,8);
  McapWN(b,ci->end,8);
  McapWN(b,recs->len,8);	//uncompressed size
  McapWN(b,0,4);		//no CRC
  McapWStr(b,"");		//not compressed
  McapWN(b,recs->len,8);
  RosBufPutBytes(b,recs->data,recs->len);
  McapRecEnd(b,at);
  ci->len=b->len-ci->offset;
  idx_start=b->len;
  ci->idx_offsets=RosBufNew();
  for (j=0;j<bag->chans->len;j++) {	//one message index per channel used
    c=RosVecGetI64(bag->chans,j);
    cnt=0;
    for (i=from;i<to;i++) {
      m=RosVecGetI64(bag->msgs,i);
      if (m->chan==c->id)
        cnt++;
    }
    if (!cnt)
      continue;
    McapWN(ci->idx_offsets,c->id,2);
    McapWN(ci->idx_offsets,b->len,8);
    at=McapRec(b,MCAP_MSGINDEX);
    McapWN(b,c->id,2);
    McapWN(b,cnt*16,4);
    for (i=from;i<to;i++) {
      m=RosVecGetI64(bag->msgs,i);
      if (m->chan!=c->id)
        continue;
      McapWN(b,m->stamp,8);
      McapWN(b,RosVecGetI64(offs,i-from),8);
    }
    McapRecEnd(b,at);
  }
  ci->idx_len=b->len-idx_start;
  RosVecDel(offs);
  RosBufDel(recs);
}

U0 McapWChunkIndex(CRosBuf *b,CMcapChunkInfo *ci)
{
  I64 at=McapRec(b,MCAP_CHUNKINDEX);
  McapWN(b,ci->start,8);
  McapWN(b,ci->end,8);
  McapWN(b,ci->offset,8);
  McapWN(b,ci->len,8);
  McapWN(b,ci->idx_offsets->len,4);
  RosBufPutBytes(b,ci->idx_offsets->data,ci->idx_offsets->len);
  McapWN(b,ci->idx_len,8);
  McapWStr(b,"");
  McapWN(b,ci->raw_len,8);	//compressed size (= uncompressed)
  McapWN(b,ci->raw_len,8);
  McapRecEnd(b,at);
}

U8 *RosBagMetadata(CRosBag *bag)
{
  CRosStrBuf *sb=RosStrBufNew();
  CRosBagChan *c;
  I64 i;
  RosStrBufAdd(sb,"rosbag2_bagfile_information:\n  version: 9\n  storage_identifier: mcap\n");
  RosStrBufAdd(sb,"  duration:\n    nanoseconds: ");
  RosStrBufAddI64(sb,bag->end-bag->start);
  RosStrBufAdd(sb,"\n  starting_time:\n    nanoseconds_since_epoch: ");
  RosStrBufAddI64(sb,bag->start);
  RosStrBufAdd(sb,"\n  message_count: ");
  RosStrBufAddI64(sb,bag->count);
  RosStrBufAdd(sb,"\n  topics_with_message_count:\n");
  for (i=0;i<bag->chans->len;i++) {
    c=RosVecGetI64(bag->chans,i);
    RosStrBufAdd(sb,"    - topic_metadata:\n        name: ");
    RosStrBufAdd(sb,c->topic);
    RosStrBufAdd(sb,"\n        type: ");
    RosStrBufAdd(sb,c->type);
    RosStrBufAdd(sb,"\n        serialization_format: cdr\n        offered_qos_profiles: []\n");
    RosStrBufAdd(sb,"        type_description_hash: ");
    RosStrBufAdd(sb,RosTypeHash(c->t));
    RosStrBufAdd(sb,"\n      message_count: ");
    RosStrBufAddI64(sb,c->count);
    RosStrBufAddC(sb,'\n');
  }
  RosStrBufAdd(sb,"  compression_format: \"\"\n  compression_mode: \"\"\n  relative_file_paths:\n    - ");
  RosStrBufAdd(sb,bag->name);
  RosStrBufAdd(sb,"_0.mcap\n  files:\n    - path: ");
  RosStrBufAdd(sb,bag->name);
  RosStrBufAdd(sb,"_0.mcap\n      starting_time:\n        nanoseconds_since_epoch: ");
  RosStrBufAddI64(sb,bag->start);
  RosStrBufAdd(sb,"\n      duration:\n        nanoseconds: ");
  RosStrBufAddI64(sb,bag->end-bag->start);
  RosStrBufAdd(sb,"\n      message_count: ");
  RosStrBufAddI64(sb,bag->count);
  RosStrBufAdd(sb,"\n  custom_data: ~\n  ros_distro: jazzy\n");
  return RosStrBufTake(sb);
}

//Finish the file (summary + footer) and write the bag directory.
Bool RosBagClose(CRosBag *bag)
{
  CRosBuf *b=bag->out;
  CRosBagChan *c;
  CRosVec *chunks=RosVecNew(sizeof(CMcapChunkInfo));
  CMcapChunkInfo ci,*cp;
  CRosBagMsg *m;
  I64 i,j,sz,at,sum_start,off_start,g_schema,g_chan,g_stats,g_ci,len_schema,len_chan,len_stats,
	len_ci;
  U8 *path,*meta;
  Bool ok;
  i=0;
  while (i<bag->msgs->len) {	//chunks of about MCAP_CHUNK_SIZE
    sz=0;
    j=i;
    while (j<bag->msgs->len && (j==i || sz<MCAP_CHUNK_SIZE)) {
      m=RosVecGetI64(bag->msgs,j);
      sz+=m->len+31;
      j++;
    }
    McapWChunk(bag,i,j,&ci);
    RosVecPush(chunks,&ci);
    i=j;
  }
  at=McapRec(b,MCAP_DATAEND);
  McapWN(b,0,4);
  McapRecEnd(b,at);
  sum_start=b->len;	//summary: schemas, channels, statistics
  g_schema=b->len;
  for (i=0;i<bag->schemas->len;i++)
    McapWSchema(b,i+1,RosVecGetI64(bag->schemas,i));
  len_schema=b->len-g_schema;
  g_chan=b->len;
  for (i=0;i<bag->chans->len;i++)
    McapWChannel(b,RosVecGetI64(bag->chans,i));
  len_chan=b->len-g_chan;
  g_stats=b->len;
  at=McapRec(b,MCAP_STATS);
  McapWN(b,bag->count,8);
  McapWN(b,bag->schemas->len,2);
  McapWN(b,bag->chans->len,4);
  McapWN(b,0,4);		//attachments
  McapWN(b,0,4);		//metadata
  McapWN(b,chunks->len,4);	//chunks
  McapWN(b,bag->start,8);
  McapWN(b,bag->end,8);
  McapWN(b,bag->chans->len*10,4);
  for (i=0;i<bag->chans->len;i++) {
    c=RosVecGetI64(bag->chans,i);
    McapWN(b,c->id,2);
    McapWN(b,c->count,8);
  }
  McapRecEnd(b,at);
  len_stats=b->len-g_stats;
  g_ci=b->len;
  for (i=0;i<chunks->len;i++) {
    cp=RosVecAt(chunks,i);
    McapWChunkIndex(b,cp);
    RosBufDel(cp->idx_offsets);
  }
  len_ci=b->len-g_ci;
  RosVecDel(chunks);
  off_start=b->len;
  if (bag->schemas->len) {
    at=McapRec(b,MCAP_SUMOFF);
    McapW8(b,MCAP_SCHEMA);
    McapWN(b,g_schema,8);
    McapWN(b,len_schema,8);
    McapRecEnd(b,at);
  }
  if (bag->chans->len) {
    at=McapRec(b,MCAP_SUMOFF);
    McapW8(b,MCAP_CHANNEL);
    McapWN(b,g_chan,8);
    McapWN(b,len_chan,8);
    McapRecEnd(b,at);
  }
  at=McapRec(b,MCAP_SUMOFF);
  McapW8(b,MCAP_STATS);
  McapWN(b,g_stats,8);
  McapWN(b,len_stats,8);
  McapRecEnd(b,at);
  if (len_ci) {
    at=McapRec(b,MCAP_SUMOFF);
    McapW8(b,MCAP_CHUNKINDEX);
    McapWN(b,g_ci,8);
    McapWN(b,len_ci,8);
    McapRecEnd(b,at);
  }
  at=McapRec(b,MCAP_FOOTER);
  McapWN(b,sum_start,8);
  McapWN(b,off_start,8);
  McapWN(b,0,4);		//no CRC
  McapRecEnd(b,at);
  McapMagic(b);
  RosDirMake(bag->dir);
  path=MStrPrint("%s/%s_0.mcap",bag->dir,bag->name);
  ok=RosFileWrite(path,b->data,b->len);
  Free(path);
  meta=RosBagMetadata(bag);
  path=MStrPrint("%s/metadata.yaml",bag->dir);
  ok=ok && RosFileWrite(path,meta,StrLen(meta));
  Free(path);
  Free(meta);
  if (!ok)
    RosBagErr("can't write bag",bag->dir);
  return ok;
}

U0 RosBagFree(CRosBag *bag)
{
  I64 i;
  CRosBagChan *c;
  CRosBagMsg *m;
  if (!bag)
    return;
  for (i=0;i<bag->chans->len;i++) {
    c=RosVecGetI64(bag->chans,i);
    Free(c->topic);
    Free(c->type);
    Free(c);
  }
  if (bag->msgs) {
    for (i=0;i<bag->msgs->len;i++) {
      m=RosVecGetI64(bag->msgs,i);
      Free(m->data);
      Free(m);
    }
    RosVecDel(bag->msgs);
  }
  RosVecDel(bag->chans);
  if (bag->writing)
    RosVecDel(bag->schemas);
  else
    RosStrVecDel(bag->schemas);
  RosBufDel(bag->out);
  Free(bag->dir);
  Free(bag->name);
  Free(bag);
}

//---- reading

class CMcapIn
{
  U8 *p;
  I64 len,pos;
  Bool err;
};

U64 McapRN(CMcapIn *r,I64 n)
{
  U64 v=0;
  I64 i;
  if (r->err || r->pos+n>r->len) {
    r->err=TRUE;
    return 0;
  }
  for (i=n-1;i>=0;i--)
    v=(v<<8)|r->p[r->pos+i];
  r->pos+=n;
  return v;
}

//New string (or NULL + err).
U8 *McapRStr(CMcapIn *r)
{
  I64 n=McapRN(r,4);
  U8 *s;
  if (r->err || n<0 || r->pos+n>r->len) {
    r->err=TRUE;
    return NULL;
  }
  s=MAlloc(n+1);
  MemCpy(s,r->p+r->pos,n);
  s[n]=0;
  r->pos+=n;
  return s;
}

//Define every type in a rosbag2 schema text we don't know yet, so
//bags with custom messages still play. TRUE if name is known after.
Bool RosBagLearnSchema(U8 *name,U8 *text)
{
  CRosVec *blocks,*names;
  I64 i,j,progress=1;
  U8 *s=text,*sep="================================================================================\nMSG: ",*nl;
  CRosStrBuf *sb;
  Bool done=FALSE;
  if (RosTypeFind(name))
    return TRUE;
  blocks=RosVecNew(8);
  names=RosVecNew(8);
  RosVecPushI64(names,StrNew(name));
  i=RosStrFind(s,sep);
  while (TRUE) {
    if (i<0) {
      RosVecPushI64(blocks,StrNew(s));
      break;
    }
    RosVecPushI64(blocks,RosStrSub(s,0,i));
    s+=i+StrLen(sep);
    j=RosStrFindC(s,'\n');
    if (j<0)
      break;
    nl=RosStrSub(s,0,j);
    if (RosStrFind(nl,"/msg/")<0) {	//"pkg/Name" -> "pkg/msg/Name"
      sb=RosStrBufNew();
      RosStrBufAddN(sb,nl,RosStrFindC(nl,'/'));
      RosStrBufAdd(sb,"/msg");
      RosStrBufAdd(sb,nl+RosStrFindC(nl,'/'));
      Free(nl);
      nl=RosStrBufTake(sb);
    }
    RosVecPushI64(names,nl);
    s+=j+1;
    i=RosStrFind(s,sep);
  }
  while (progress && !done) {	//dependencies first: retry until stuck
    progress=0;
    for (i=blocks->len-1;i>=0;i--)
      if (!RosTypeFind(RosVecGetI64(names,i)) &&
	  RosMsgDefine(RosVecGetI64(names,i),RosVecGetI64(blocks,i)))
        progress++;
    done=RosTypeFind(name)!=NULL;
  }
  RosStrVecDel(blocks);
  RosStrVecDel(names);
  if (!done)
    RosBagErr("can't understand the message definition",name);
  return done;
}

//Read records in [p, p+len). Chunks recurse.
Bool RosBagParse(CRosBag *bag,U8 *p,I64 len)
{
  CMcapIn r,c;
  I64 op,n,end,sid,cid,i;
  U8 *s,*name,*enc,*text,*comp,**arr;
  CRosBagChan *ch;
  CRosBagMsg *m;
  r.p=p;
  r.len=len;
  r.pos=0;
  r.err=FALSE;
  while (r.pos+9<=len && !r.err) {
    op=McapRN(&r,1);
    n=McapRN(&r,8);
    if (n<0 || r.pos+n>len) {
      RosBagErr("bad MCAP record","length");
      return FALSE;
    }
    end=r.pos+n;
    c.p=p+r.pos;
    c.len=n;
    c.pos=0;
    c.err=FALSE;
    switch (op) {
      case MCAP_SCHEMA:
        sid=McapRN(&c,2);
        name=McapRStr(&c);
        enc=McapRStr(&c);
        text=McapRStr(&c);
        if (!c.err) {
          while (bag->schemas->len<sid)
            RosVecPushI64(bag->schemas,NULL);
          arr=bag->schemas->data;
          if (!arr[sid-1])
            arr[sid-1]=StrNew(name);
          if (!StrCmp(enc,"ros2msg"))
            RosBagLearnSchema(name,text);
        }
        Free(name);
        Free(enc);
        Free(text);
        break;
      case MCAP_CHANNEL:
        cid=McapRN(&c,2);
        sid=McapRN(&c,2);
        s=McapRStr(&c);
        if (!c.err && !RosBagChanFind(bag,s) && sid>=1 && sid<=bag->schemas->len) {
          ch=CAlloc(sizeof(CRosBagChan));
          ch->id=cid;
          ch->schema_id=sid;
          ch->topic=StrNew(s);
          ch->type=StrNew(RosVecGetI64(bag->schemas,sid-1));
          ch->t=RosTypeFind(ch->type);
          RosVecPushI64(bag->chans,ch);
        }
        Free(s);
        break;
      case MCAP_MESSAGE:
        m=CAlloc(sizeof(CRosBagMsg));
        m->chan=McapRN(&c,2);
        McapRN(&c,4);
        m->stamp=McapRN(&c,8);
        McapRN(&c,8);
        m->len=n-c.pos;
        if (c.err || m->len<0) {
          Free(m);
          break;
        }
        m->data=MAlloc(m->len+1);
        MemCpy(m->data,c.p+c.pos,m->len);
        RosVecPushI64(bag->msgs,m);
        if (!bag->count || m->stamp<bag->start)
          bag->start=m->stamp;
        if (m->stamp>bag->end)
          bag->end=m->stamp;
        bag->count++;
        for (i=0;i<bag->chans->len;i++) {
          ch=RosVecGetI64(bag->chans,i);
          if (ch->id==m->chan)
            ch->count++;
        }
        break;
      case MCAP_CHUNK:
        McapRN(&c,8);
        McapRN(&c,8);
        McapRN(&c,8);	//uncompressed size
        McapRN(&c,4);	//crc
        comp=McapRStr(&c);
        i=McapRN(&c,8);	//records length
        if (!c.err && !StrLen(comp) && c.pos+i<=n) {
          if (!RosBagParse(bag,c.p+c.pos,i)) {
            Free(comp);
            return FALSE;
          }
        } else if (!c.err) {
          RosBagErr("compressed chunks aren't supported",comp);
          Free(comp);
          return FALSE;
        }
        Free(comp);
        break;
      case MCAP_DATAEND:	//the summary repeats what we have
        return TRUE;
    }
    r.pos=end;
  }
  return !r.err;
}

//The .mcap file of a bag directory: the first of relative_file_paths
//in metadata.yaml, else name_0.mcap. (Split bags: only the first file.)
U8 *RosBagFileOf(U8 *dir)
{
  U8 *path=MStrPrint("%s/metadata.yaml",dir),*meta,*s,*e,*q,*f=NULL,*name;
  I64 i;
  meta=RosFileRead(path,NULL);
  Free(path);
  if (meta) {
    i=RosStrFind(meta,"relative_file_paths:");
    if (i>=0) {
      s=meta+i;
      i=RosStrFind(s,"- ");
      if (i>=0) {
        s+=i+2;
        e=s;
        while (*e && *e!='\n' && *e!='\r')
          e++;
        *e=0;
        q=RosStrTrim(s);
        f=RosStrUnquote(q);
        Free(q);
        q=f;
        f=MStrPrint("%s/%s",dir,q);
        Free(q);
      }
    }
    Free(meta);
  }
  if (!f) {
    i=StrLen(dir);
    while (i>0 && dir[i-1]!='/')
      i--;
    name=dir+i;
    f=MStrPrint("%s/%s_0.mcap",dir,name);
  }
  return f;
}

//Open a bag directory (or a .mcap file) for reading.
CRosBag *RosBagOpen(U8 *path)
{
  CRosBag *bag;
  U8 *file,*data;
  I64 len;
  if (RosStrEndsWith(path,".mcap"))
    file=StrNew(path);
  else
    file=RosBagFileOf(path);
  data=RosFileRead(file,&len);
  if (!data) {
    RosBagErr("can't read",file);
    Free(file);
    return NULL;
  }
  Free(file);
  bag=CAlloc(sizeof(CRosBag));
  bag->dir=StrNew(path);
  bag->name=StrNew("");
  bag->chans=RosVecNew(8);
  bag->schemas=RosVecNew(8);
  bag->msgs=RosVecNew(8);
  if (len<16 || data[0]!=0x89 || MemCmp(data+1,"MCAP0\r\n",7) ||
      !RosBagParse(bag,data+8,len-8)) {
    if (len<16 || data[0]!=0x89)
      RosBagErr("not an MCAP file",path);
    Free(data);
    RosBagFree(bag);
    return NULL;
  }
  Free(data);
  return bag;
}

CRosBagChan *RosBagChanById(CRosBag *bag,I64 id)
{
  I64 i;
  CRosBagChan *c;
  for (i=0;i<bag->chans->len;i++) {
    c=RosVecGetI64(bag->chans,i);
    if (c->id==id)
      return c;
  }
  return NULL;
}

//Like `ros2 bag info`.
U8 *RosBagInfoStr(U8 *path)
{
  CRosBag *bag=RosBagOpen(path);
  CRosStrBuf *sb;
  CRosBagChan *c;
  I64 i;
  if (!bag)
    return MStrPrint("Error: %s\n",ros_bag_error);
  sb=RosStrBufNew();
  RosStrBufAdd(sb,"Storage id:        mcap\nDuration:          ");
  RosStrBufAddFixed(sb,RosTimeToSec(bag->end-bag->start),9);
  RosStrBufAdd(sb,"s\nMessages:          ");
  RosStrBufAddI64(sb,bag->count);
  RosStrBufAdd(sb,"\nTopic information: ");
  for (i=0;i<bag->chans->len;i++) {
    c=RosVecGetI64(bag->chans,i);
    if (i)
      RosStrBufAdd(sb,"                   ");
    RosStrBufAdd(sb,"Topic: ");
    RosStrBufAdd(sb,c->topic);
    RosStrBufAdd(sb," | Type: ");
    RosStrBufAdd(sb,c->type);
    RosStrBufAdd(sb," | Count: ");
    RosStrBufAddI64(sb,c->count);
    RosStrBufAdd(sb," | Serialization Format: cdr\n");
  }
  if (!bag->chans->len)
    RosStrBufAddC(sb,'\n');
  RosBagFree(bag);
  return RosStrBufTake(sb);
}

U0 RosBagInfo(U8 *path)
{
  RosCliPrint(RosBagInfoStr(path));
}

//Publish everything in a bag again with the original timing (rate 2.0
//= twice as fast; 0 = as fast as possible). Returns messages played.
I64 RosBagPlay(U8 *path,F64 rate)
{
  CRosBag *bag=RosBagOpen(path);
  CRosNode *n;
  CRosBagChan *c;
  CRosBagMsg *m;
  CRosShared *sh;
  I64 i,played=0,t0,wait;
  if (!bag) {
    Print("Error: %s\n",ros_bag_error);
    return 0;
  }
  n=RosCliNodeNew();
  for (i=0;i<bag->chans->len;i++) {
    c=RosVecGetI64(bag->chans,i);
    if (c->t)
      c->pub=RosPubNew(n,c->topic,c->t,NULL);
    else
      Print("skipping %s: unknown type %s\n",c->topic,c->type);
  }
  t0=RosTimeSteadyNs();
  for (i=0;i<bag->msgs->len && ros_ok && !RosCliKey();i++) {
    m=RosVecGetI64(bag->msgs,i);
    c=RosBagChanById(bag,m->chan);
    if (!c || !c->pub)
      continue;
    if (rate>0.0) {
      wait=t0+ToI64(ToF64(m->stamp-bag->start)/rate)-RosTimeSteadyNs();
      while (wait>0 && ros_ok) {
        if (wait>10*ROS_NS_PER_MS)
          wait=10*ROS_NS_PER_MS;
        RosCliSpinOnce(n,wait);
        wait=t0+ToI64(ToF64(m->stamp-bag->start)/rate)-RosTimeSteadyNs();
      }
    }
    sh=RosSharedNew(c->t);
    if (RosCdrDeser(c->t,m->data,m->len,RosSharedMsg(sh))) {
      RosPublishLoaned(c->pub,RosSharedMsg(sh));
      played++;
    } else
      RosSharedRelease(sh);
    RosCliSpinOnce(n,0);
  }
  RosCliNodeDel(n);
  RosBagFree(bag);
  return played;
}

//---- recording

class CRosBagSub
{
  U8 *rec;		//CRosBagRec *
  U8 *topic;
  CRosType *t;
};

class CRosBagRec
{
  CRosBag *bag;
  CRosNode *node;
  CRosVec *subs;	//CRosBagSub *
};

U0 RosBagOnMsg(U8 *msg,CRosBagSub *bs)
{
  CRosBagRec *r=bs->rec;
  RosBagWriteMsg(r->bag,bs->topic,bs->t,RosTimeSystemNs(),msg);
}

//Record topics ("/a /b", or NULL for every topic there is now) into
//the bag directory path. The callbacks run in the recorder node's
//executor (RosNodeExec(r->node)), which you spin.
CRosBagRec *RosBagRecordStart(U8 *path,U8 *topics)
{
  CRosBagRec *r=CAlloc(sizeof(CRosBagRec));
  CRosVec *names;
  CRosBagSub *bs;
  CRosQoS q;
  CRosType *t;
  I64 i;
  r->bag=RosBagCreate(path);
  r->node=RosCliNodeNew();
  r->subs=RosVecNew(8);
  if (topics)
    names=RosStrSplit(topics,' ');
  else
    names=RosCliAllNames(RosGraph()->topics,FALSE);
  for (i=0;i<names->len;i++) {
    if (!StrLen(RosVecGetI64(names,i)))
      continue;
    t=RosTopicTypeOf(RosVecGetI64(names,i));
    if (!t) {
      Print("not recording %s: unknown topic\n",RosVecGetI64(names,i));
      continue;
    }
    bs=CAlloc(sizeof(CRosBagSub));
    bs->rec=r;
    bs->topic=StrNew(RosVecGetI64(names,i));
    bs->t=t;
    RosQoSDefault(&q);
    q.depth=1000;
    RosSubNew(r->node,bs->topic,t,&q,&RosBagOnMsg,bs);
    RosVecPushI64(r->subs,bs);
  }
  RosStrVecDel(names);
  return r;
}

//Stop and write the bag. TRUE if it was written.
Bool RosBagRecordStop(CRosBagRec *r)
{
  Bool ok;
  I64 i;
  CRosBagSub *bs;
  RosExecSpinSome(RosNodeExec(r->node));	//take what's queued
  ok=RosBagClose(r->bag);
  RosCliNodeDel(r->node);
  for (i=0;i<r->subs->len;i++) {
    bs=RosVecGetI64(r->subs,i);
    Free(bs->topic);
    Free(bs);
  }
  RosVecDel(r->subs);
  RosBagFree(r->bag);
  Free(r);
  return ok;
}

//Like `ros2 bag record`: record until a key is pressed (or for
//max_ns if it isn't 0). Returns messages recorded.
I64 RosBagRecord(U8 *path,U8 *topics,I64 max_ns)
{
  CRosBagRec *r=RosBagRecordStart(path,topics);
  I64 end=RosTimeSteadyNs()+max_ns,n;
  while (ros_ok && !RosCliKey() && (!max_ns || RosTimeSteadyNs()<end))
    RosCliSpinOnce(r->node,10*ROS_NS_PER_MS);
  n=r->bag->count;
  RosBagRecordStop(r);
  return n;
}
