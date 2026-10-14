//Hash: Jazzy type hashes ("RIHS01_" + SHA-256 of the type
//description as JSON), the same as rosidl_generator_type_description.
//Jazzy sends them in DDS discovery and stores them in bags.
//
//  U8 *h=RosTypeHash(ros_t_geometry_msgs_Twist);     //cached, don't Free
//  U8 *s=RosSrvTypeHash(st);                        //services (cached too)
//
//Like rosidl, .msg "char" is uint8 and "byte" is octet.

#define ROS_FT_NESTED	1
#define ROS_FT_STRING	17
#define ROS_FT_BSTRING	21

CRosMap *ros_type_hashes=NULL;	//type or service name -> "RIHS01_..."

I64 RosHashTypeId(CRosField *f)
{
  I64 id=0;
  switch (f->kind) {
    case ROS_T_BOOL:	id=15; break;
    case ROS_T_BYTE:	id=16; break;
    case ROS_T_CHAR:	id=3; break;	//rosidl maps .msg char to uint8
    case ROS_T_INT8:	id=2; break;
    case ROS_T_UINT8:	id=3; break;
    case ROS_T_INT16:	id=4; break;
    case ROS_T_UINT16:	id=5; break;
    case ROS_T_INT32:	id=6; break;
    case ROS_T_UINT32:	id=7; break;
    case ROS_T_INT64:	id=8; break;
    case ROS_T_UINT64:	id=9; break;
    case ROS_T_FLOAT32:	id=10; break;
    case ROS_T_FLOAT64:	id=11; break;
    case ROS_T_STRING:
      if (f->str_bound)
        id=ROS_FT_BSTRING;
      else
        id=ROS_FT_STRING;
      break;
    case ROS_T_MSG:	id=ROS_FT_NESTED; break;
  }
  if (f->arr==ROS_ARR_FIXED)
    id+=48;
  else if (f->arr==ROS_ARR_SEQ && f->arr_len)
    id+=96;
  else if (f->arr==ROS_ARR_SEQ)
    id+=144;
  return id;
}

//One field: {"name": ..., "type": {...}}
U0 RosHashField(CRosStrBuf *sb,U8 *name,I64 id,I64 cap,I64 scap,U8 *nested)
{
  RosStrBufAdd(sb,"{\"name\": \"");
  RosStrBufAdd(sb,name);
  RosStrBufAdd(sb,"\", \"type\": {\"type_id\": ");
  RosStrBufAddI64(sb,id);
  RosStrBufAdd(sb,", \"capacity\": ");
  RosStrBufAddI64(sb,cap);
  RosStrBufAdd(sb,", \"string_capacity\": ");
  RosStrBufAddI64(sb,scap);
  RosStrBufAdd(sb,", \"nested_type_name\": \"");
  RosStrBufAdd(sb,nested);
  RosStrBufAdd(sb,"\"}}");
}

//{"type_name": ..., "fields": [...]} of a message type.
U8 *RosHashStruct(CRosType *t)
{
  CRosStrBuf *sb=RosStrBufNew();
  CRosField *f;
  I64 i,cap;
  U8 *nested;
  RosStrBufAdd(sb,"{\"type_name\": \"");
  RosStrBufAdd(sb,t->name);
  RosStrBufAdd(sb,"\", \"fields\": [");
  for (i=0;i<t->fields->len;i++) {
    f=RosVecGetI64(t->fields,i);
    if (i)
      RosStrBufAdd(sb,", ");
    cap=0;
    if (f->arr!=ROS_ARR_NONE)
      cap=f->arr_len;
    nested="";
    if (f->kind==ROS_T_MSG)
      nested=f->sub->name;
    RosHashField(sb,f->name,RosHashTypeId(f),cap,f->str_bound,nested);
  }
  RosStrBufAdd(sb,"]}");
  return RosStrBufTake(sb);
}

//Add t and every type it uses to deps (CRosType *, no repeats).
U0 RosHashDeps(CRosType *t,CRosVec *deps)
{
  I64 i;
  CRosField *f;
  CRosType *d;
  for (i=0;i<deps->len;i++) {
    d=RosVecGetI64(deps,i);
    if (d==t)
      return;
  }
  RosVecPushI64(deps,t);
  for (i=0;i<t->fields->len;i++) {
    f=RosVecGetI64(t->fields,i);
    if (f->kind==ROS_T_MSG)
      RosHashDeps(f->sub,deps);
  }
}

//Hash of {"type_description": main, "referenced_type_descriptions":
//[refs sorted by name]}. refs holds "name\tjson" strings.
U8 *RosHashFinish(U8 *main,CRosVec *refs)
{
  CRosStrBuf *sb=RosStrBufNew();
  I64 i;
  U8 *s,*hex,*out;
  RosStrVecSort(refs);
  RosStrBufAdd(sb,"{\"type_description\": ");
  RosStrBufAdd(sb,main);
  RosStrBufAdd(sb,", \"referenced_type_descriptions\": [");
  for (i=0;i<refs->len;i++) {
    if (i)
      RosStrBufAdd(sb,", ");
    s=RosVecGetI64(refs,i);
    RosStrBufAdd(sb,s+RosStrFindC(s,'\t')+1);
  }
  RosStrBufAdd(sb,"]}");
  hex=RosSha256Hex(sb->buf,sb->len);
  out=MStrPrint("RIHS01_%s",hex);
  Free(hex);
  RosStrBufDel(sb);
  return out;
}

//Every type in deps but skip, as "name\tjson".
U0 RosHashRefs(CRosVec *deps,CRosType *skip,CRosVec *refs)
{
  I64 i;
  CRosType *d;
  U8 *js;
  for (i=0;i<deps->len;i++) {
    d=RosVecGetI64(deps,i);
    if (d==skip)
      goto next_hash_1;
    js=RosHashStruct(d);
    RosVecPushI64(refs,MStrPrint("%s\t%s",d->name,js));
    Free(js);
    next_hash_1:;
  }
}

U8 *RosHashCached(U8 *name)
{
  if (!ros_type_hashes)
    return NULL;
  return RosMapGet(ros_type_hashes,name);
}

U8 *RosHashCache(U8 *name,U8 *hash)
{
  if (!ros_type_hashes)
    ros_type_hashes=RosMapNew();
  RosMapSet(ros_type_hashes,name,hash);
  return hash;
}

//"RIHS01_..." of a message type (cached; don't Free).
U8 *RosTypeHash(CRosType *t)
{
  CRosVec *deps,*refs;
  U8 *main,*h=RosHashCached(t->name);
  if (h)
    return h;
  deps=RosVecNew(8);
  refs=RosVecNew(8);
  RosHashDeps(t,deps);
  RosHashRefs(deps,t,refs);
  main=RosHashStruct(t);
  h=RosHashFinish(main,refs);
  Free(main);
  RosVecDel(deps);
  RosStrVecDel(refs);
  return RosHashCache(t->name,h);
}

//"RIHS01_..." of a service. rosidl describes a service as a struct of
//request_message, response_message and event_message, where the event
//is {service_msgs/ServiceEventInfo info, Request[<=1], Response[<=1]}.
//VERIFY against a real Jazzy install (messages are checked against rosbags).
U8 *RosSrvTypeHash(CRosSrvType *st)
{
  CRosVec *deps,*refs;
  CRosStrBuf *sb;
  CRosType *info=RosTypeFind("service_msgs/msg/ServiceEventInfo");
  U8 *main,*ev,*h=RosHashCached(st->name);
  if (h)
    return h;
  if (!info)
    return NULL;
  ev=MStrPrint("%s_Event",st->name);
  deps=RosVecNew(8);
  refs=RosVecNew(8);
  RosHashDeps(st->req,deps);
  RosHashDeps(st->resp,deps);
  RosHashDeps(info,deps);
  RosHashRefs(deps,NULL,refs);
  sb=RosStrBufNew();	//the event message
  RosStrBufAdd(sb,ev);
  RosStrBufAdd(sb,"\t{\"type_name\": \"");
  RosStrBufAdd(sb,ev);
  RosStrBufAdd(sb,"\", \"fields\": [");
  RosHashField(sb,"info",ROS_FT_NESTED,0,0,info->name);
  RosStrBufAdd(sb,", ");
  RosHashField(sb,"request",ROS_FT_NESTED+96,1,0,st->req->name);
  RosStrBufAdd(sb,", ");
  RosHashField(sb,"response",ROS_FT_NESTED+96,1,0,st->resp->name);
  RosStrBufAdd(sb,"]}");
  RosVecPushI64(refs,RosStrBufTake(sb));
  sb=RosStrBufNew();	//the service itself
  RosStrBufAdd(sb,"{\"type_name\": \"");
  RosStrBufAdd(sb,st->name);
  RosStrBufAdd(sb,"\", \"fields\": [");
  RosHashField(sb,"request_message",ROS_FT_NESTED,0,0,st->req->name);
  RosStrBufAdd(sb,", ");
  RosHashField(sb,"response_message",ROS_FT_NESTED,0,0,st->resp->name);
  RosStrBufAdd(sb,", ");
  RosHashField(sb,"event_message",ROS_FT_NESTED,0,0,ev);
  RosStrBufAdd(sb,"]}");
  main=RosStrBufTake(sb);
  h=RosHashFinish(main,refs);
  Free(main);
  Free(ev);
  RosVecDel(deps);
  RosStrVecDel(refs);
  return RosHashCache(st->name,h);
}
