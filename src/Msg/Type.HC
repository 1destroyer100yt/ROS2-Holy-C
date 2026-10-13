//Type: ROS message type descriptions, built at load time by parsing
//.msg text (RosMsgDefine). Everything else in src/Msg (CDR, YAML,
//copy, generated classes) works from these descriptions.
//
//How a message is laid out in memory (the generated classes in
//interfaces/*/Types.HC follow exactly this, see Gen.HC):
//  - every field takes a multiple of 8 bytes, so the layout is the
//    same whether or not the compiler pads
//  - scalars: 8 bytes. Integers/bool as I64 (U64 for uint64), float32
//    and float64 as F64, string as U8 * (NULL means "")
//  - nested message: embedded inline
//  - fixed array T[N]: N elements inline. Byte-sized types (bool byte
//    char int8 uint8) take 1 byte each, padded to 8; others 8 bytes
//    each, messages their own size
//  - sequence T[] / T[<=N]: a CRosSeq {data,len,cap}; elements use
//    the same per-element size as fixed arrays

#define ROS_T_BOOL	1
#define ROS_T_BYTE	2
#define ROS_T_CHAR	3
#define ROS_T_INT8	4
#define ROS_T_UINT8	5
#define ROS_T_INT16	6
#define ROS_T_UINT16	7
#define ROS_T_INT32	8
#define ROS_T_UINT32	9
#define ROS_T_INT64	10
#define ROS_T_UINT64	11
#define ROS_T_FLOAT32	12
#define ROS_T_FLOAT64	13
#define ROS_T_STRING	14
#define ROS_T_MSG	15

#define ROS_ARR_NONE	0
#define ROS_ARR_FIXED	1
#define ROS_ARR_SEQ	2

class CRosSeq
{
  U8 *data;
  I64 len,cap;
};

class CRosType
{
  U8 *name;		//"geometry_msgs/msg/Twist"
  U8 *pkg;		//"geometry_msgs"
  U8 *base;		//"Twist"
  U8 *def;		//the .msg text it was built from
  CRosVec *fields;	//CRosField *
  CRosVec *consts;	//CRosConst *
  I64 size;		//bytes of one message object
  Bool owns_mem;	//has strings or sequences somewhere (needs Fini)
};

class CRosField
{
  U8 *name;
  I64 kind;		//ROS_T_*
  CRosType *sub;	//for ROS_T_MSG
  I64 arr;		//ROS_ARR_*
  I64 arr_len;		//fixed length, or sequence bound (0 = unbounded)
  I64 str_bound;	//string<=N, 0 = unbounded
  I64 offset,size;	//position and bytes inside the message object
  I64 stride;		//bytes per array/sequence element
  U8 *def;		//default value text, or NULL
};

class CRosConst
{
  U8 *name;
  I64 kind;
  I64 i;
  F64 f;
  U8 *s;
};

class CRosSrvType
{
  U8 *name;		//"example_interfaces/srv/AddTwoInts"
  CRosType *req,*resp;
};

CRosMap *ros_types=NULL;	//name -> CRosType *
CRosMap *ros_srv_types=NULL;	//name -> CRosSrvType *
U8 ros_msg_error[256];		//last error from parse/CDR/set

U0 RosMsgErr(U8 *what,U8 *detail)
{
  I64 n=0,k;
  k=StrLen(what);
  if (k>200)
    k=200;
  MemCpy(ros_msg_error,what,k);
  n=k;
  if (detail) {
    ros_msg_error[n]=':';
    ros_msg_error[n+1]=' ';
    n+=2;
    k=StrLen(detail);
    if (k>250-n)
      k=250-n;
    MemCpy(ros_msg_error+n,detail,k);
    n+=k;
  }
  ros_msg_error[n]=0;
}

//Kind for a primitive type name, 0 if it isn't one.
I64 RosKindFromName(U8 *s)
{
  if (!StrCmp(s,"bool")) return ROS_T_BOOL;
  if (!StrCmp(s,"byte")) return ROS_T_BYTE;
  if (!StrCmp(s,"char")) return ROS_T_CHAR;
  if (!StrCmp(s,"int8")) return ROS_T_INT8;
  if (!StrCmp(s,"uint8")) return ROS_T_UINT8;
  if (!StrCmp(s,"int16")) return ROS_T_INT16;
  if (!StrCmp(s,"uint16")) return ROS_T_UINT16;
  if (!StrCmp(s,"int32")) return ROS_T_INT32;
  if (!StrCmp(s,"uint32")) return ROS_T_UINT32;
  if (!StrCmp(s,"int64")) return ROS_T_INT64;
  if (!StrCmp(s,"uint64")) return ROS_T_UINT64;
  if (!StrCmp(s,"float32")) return ROS_T_FLOAT32;
  if (!StrCmp(s,"float64")) return ROS_T_FLOAT64;
  if (!StrCmp(s,"string")) return ROS_T_STRING;
  return 0;
}

U8 *RosKindName(I64 kind)
{
  switch (kind) {
    case ROS_T_BOOL: return "bool";
    case ROS_T_BYTE: return "byte";
    case ROS_T_CHAR: return "char";
    case ROS_T_INT8: return "int8";
    case ROS_T_UINT8: return "uint8";
    case ROS_T_INT16: return "int16";
    case ROS_T_UINT16: return "uint16";
    case ROS_T_INT32: return "int32";
    case ROS_T_UINT32: return "uint32";
    case ROS_T_INT64: return "int64";
    case ROS_T_UINT64: return "uint64";
    case ROS_T_FLOAT32: return "float32";
    case ROS_T_FLOAT64: return "float64";
    case ROS_T_STRING: return "string";
  }
  return "msg";
}

//Types stored 1 byte per element inside arrays and sequences.
Bool RosKindIsByte(I64 kind)
{
  return kind>=ROS_T_BOOL && kind<=ROS_T_UINT8;
}

Bool RosKindIsInt(I64 kind)
{
  return kind>=ROS_T_BOOL && kind<=ROS_T_UINT64;
}

Bool RosKindIsFloat(I64 kind)
{
  return kind==ROS_T_FLOAT32 || kind==ROS_T_FLOAT64;
}

//Wrap an integer to the range of its ROS type (like a C cast would).
I64 RosIntNorm(I64 kind,I64 v)
{
  switch (kind) {
    case ROS_T_BOOL:
      if (v)
        return 1;
      return 0;
    case ROS_T_INT8:
      v=v&0xFF;
      if (v>=0x80)
        v-=0x100;
      return v;
    case ROS_T_BYTE:
    case ROS_T_CHAR:
    case ROS_T_UINT8:
      return v&0xFF;
    case ROS_T_INT16:
      v=v&0xFFFF;
      if (v>=0x8000)
        v-=0x10000;
      return v;
    case ROS_T_UINT16:
      return v&0xFFFF;
    case ROS_T_INT32:
      v=v&0xFFFFFFFF;
      if (v>=0x80000000)
        v-=0x100000000;
      return v;
    case ROS_T_UINT32:
      return v&0xFFFFFFFF;
  }
  return v;
}

//"pkg/Name" or "pkg/msg/Name" -> new "pkg/msg/Name".
U8 *RosTypeFullName(U8 *name)
{
  CRosStrBuf *sb;
  I64 i=RosStrFindC(name,'/');
  if (i<0 || RosStrFindC(name+i+1,'/')>=0)
    return StrNew(name);
  sb=RosStrBufNew();
  RosStrBufAddN(sb,name,i);
  RosStrBufAdd(sb,"/msg/");
  RosStrBufAdd(sb,name+i+1);
  return RosStrBufTake(sb);
}

//Look up a type by "pkg/msg/Name" or "pkg/Name". NULL if unknown.
CRosType *RosTypeFind(U8 *name)
{
  U8 *full;
  CRosType *t;
  if (!ros_types)
    return NULL;
  full=RosTypeFullName(name);
  t=RosMapGet(ros_types,full);
  Free(full);
  return t;
}

CRosField *RosTypeField(CRosType *t,U8 *name)
{
  I64 i;
  CRosField *f;
  for (i=0;i<t->fields->len;i++) {
    f=RosVecGetI64(t->fields,i);
    if (!StrCmp(f->name,name))
      return f;
  }
  return NULL;
}

CRosConst *RosTypeConst(CRosType *t,U8 *name)
{
  I64 i;
  CRosConst *c;
  for (i=0;i<t->consts->len;i++) {
    c=RosVecGetI64(t->consts,i);
    if (!StrCmp(c->name,name))
      return c;
  }
  return NULL;
}

//Parse one scalar value of kind into *i / *f / *s (new string).
//FALSE (and ros_msg_error) on bad text.
Bool RosValParse(I64 kind,U8 *text,I64 *i,F64 *f,U8 **s)
{
  U8 *t=RosStrTrim(text);
  Bool ok=TRUE;
  if (kind==ROS_T_STRING) {
    *s=RosStrUnquote(t);
  } else if (kind==ROS_T_BOOL) {
    if (!StrCmp(t,"true") || !StrCmp(t,"True") || !StrCmp(t,"1"))
      *i=1;
    else if (!StrCmp(t,"false") || !StrCmp(t,"False") || !StrCmp(t,"0"))
      *i=0;
    else
      ok=FALSE;
  } else if (RosKindIsFloat(kind)) {
    ok=RosStrToF64(t,f);
  } else {
    ok=RosStrToI64(t,i);
    if (ok && kind!=ROS_T_UINT64 && kind!=ROS_T_INT64 && RosIntNorm(kind,*i)!=*i)
      ok=FALSE;
  }
  if (!ok)
    RosMsgErr("bad value",t);
  Free(t);
  return ok;
}

//Split "[a, 'b,c', d]" into a vec of new trimmed strings. NULL if
//text isn't a [...] list.
CRosVec *RosListSplit(U8 *text)
{
  U8 *t=RosStrTrim(text),q=0;
  I64 n=StrLen(t),i,start;
  CRosVec *v,*out;
  if (n<2 || t[0]!='[' || t[n-1]!=']') {
    Free(t);
    return NULL;
  }
  v=RosVecNew(8);
  start=1;
  for (i=1;i<n;i++) {
    if (q) {
      if (t[i]==q)
        q=0;
    } else if (t[i]=='\'' || t[i]=='"')
      q=t[i];
    else if (t[i]==',' || i==n-1) {
      RosVecPushI64(v,RosStrSub(t,start,i-start));
      start=i+1;
    }
  }
  Free(t);
  out=RosVecNew(8);
  for (i=0;i<v->len;i++) {
    t=RosVecGetI64(v,i);
    RosVecPushI64(out,RosStrTrim(t));
    Free(t);
  }
  RosVecDel(v);
  //"[]" gives one empty item: drop it
  if (out->len==1 && !StrLen(RosVecGetI64(out,0))) {
    Free(RosVecGetI64(out,0));
    out->len=0;
  }
  return out;
}
