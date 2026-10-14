//Cdr: serialize messages the way ROS 2 does on the wire (OMG CDR,
//"XCDR1", what rmw_fastrtps / rmw_cyclonedds use for ROS messages).
//
//  4-byte encapsulation header: 00 01 00 00 = little endian CDR
//  (00 00 = big endian, which we can read too). After the header:
//  - every primitive is aligned to its own size (8-byte types to 8),
//    counted from the end of the header
//  - bool/byte/char/int8/uint8: 1 byte, int16: 2, int32/float32: 4,
//    int64/float64: 8
//  - string: uint32 length including the NUL, the bytes, the NUL
//  - sequence: uint32 count, then the elements
//  - fixed array: just the elements; nested message: just its fields
//
//The reader never trusts a length from the wire: everything is
//bounds-checked, because a crash here takes down the whole OS.

class CRosBuf
{
  U8 *data;
  I64 len,cap;	//writer: bytes used / allocated
  I64 pos;	//reader: next byte
  I64 origin;	//alignment is counted from here (after the header)
  Bool swap;	//reader: data is big endian
  Bool err;	//reader: ran off the end / bad data
  Bool owned;	//data belongs to this buffer
};

CRosBuf *RosBufNew()
{
  CRosBuf *b=CAlloc(sizeof(CRosBuf));
  b->cap=64;
  b->data=MAlloc(b->cap);
  b->owned=TRUE;
  return b;
}

//Reader over existing bytes (not copied, not freed).
CRosBuf *RosBufReader(U8 *data,I64 len)
{
  CRosBuf *b=CAlloc(sizeof(CRosBuf));
  b->data=data;
  b->len=len;
  return b;
}

U0 RosBufDel(CRosBuf *b)
{
  if (!b)
    return;
  if (b->owned)
    Free(b->data);
  Free(b);
}

U0 RosBufReserve(CRosBuf *b,I64 n)
{
  U8 *d;
  I64 cap=b->cap;
  if (b->len+n<=cap)
    return;
  while (b->len+n>cap)
    cap*=2;
  d=MAlloc(cap);
  MemCpy(d,b->data,b->len);
  Free(b->data);
  b->data=d;
  b->cap=cap;
}

//---- writer

U0 RosBufPutAlign(CRosBuf *b,I64 a)
{
  while ((b->len-b->origin)%a) {
    RosBufReserve(b,1);
    b->data[b->len]=0;
    b->len++;
  }
}

//n bytes of v, little endian, aligned to n.
U0 RosBufPutN(CRosBuf *b,U64 v,I64 n)
{
  I64 i;
  RosBufPutAlign(b,n);
  RosBufReserve(b,n);
  for (i=0;i<n;i++) {
    b->data[b->len]=v&0xFF;
    v=v>>8;
    b->len++;
  }
}

U0 RosBufPutBytes(CRosBuf *b,U8 *p,I64 n)
{
  RosBufReserve(b,n);
  MemCpy(b->data+b->len,p,n);
  b->len+=n;
}

U0 RosBufPutStr(CRosBuf *b,U8 *s)
{
  I64 n=StrLen(s);
  RosBufPutN(b,n+1,4);
  RosBufPutBytes(b,s,n);
  RosBufPutN(b,0,1);
}

//---- reader

U0 RosBufGetAlign(CRosBuf *b,I64 a)
{
  while ((b->pos-b->origin)%a)
    b->pos++;
  if (b->pos>b->len)
    b->err=TRUE;
}

//n bytes, aligned to n. 0 (and err) if past the end.
U64 RosBufGetN(CRosBuf *b,I64 n)
{
  U64 v=0;
  I64 i;
  RosBufGetAlign(b,n);
  if (b->err || b->pos+n>b->len) {
    b->err=TRUE;
    return 0;
  }
  if (b->swap)
    for (i=0;i<n;i++)
      v=(v<<8)|b->data[b->pos+i];
  else
    for (i=n-1;i>=0;i--)
      v=(v<<8)|b->data[b->pos+i];
  b->pos+=n;
  return v;
}

//New string (yours to Free), or NULL + err.
U8 *RosBufGetStr(CRosBuf *b)
{
  I64 n=RosBufGetN(b,4);
  U8 *s;
  if (b->err)
    return NULL;
  if (!n)		//some writers send 0 for ""
    return StrNew("");
  if (n>b->len-b->pos || b->data[b->pos+n-1]) {
    b->err=TRUE;
    return NULL;
  }
  s=MAlloc(n);
  MemCpy(s,b->data+b->pos,n);
  b->pos+=n;
  return s;
}

//---- messages

//Wire size of one primitive.
I64 RosKindWireSize(I64 kind)
{
  switch (kind) {
    case ROS_T_INT16:
    case ROS_T_UINT16:
      return 2;
    case ROS_T_INT32:
    case ROS_T_UINT32:
    case ROS_T_FLOAT32:
      return 4;
    case ROS_T_INT64:
    case ROS_T_UINT64:
    case ROS_T_FLOAT64:
      return 8;
  }
  return 1;
}

extern Bool RosCdrPutFields(CRosBuf *b,CRosType *t,U8 *m);
extern Bool RosCdrGetFields(CRosBuf *b,CRosType *t,U8 *m);

Bool RosCdrPutElem(CRosBuf *b,CRosField *f,U8 *p)
{
  U8 *s;
  switch (f->kind) {
    case ROS_T_MSG:
      return RosCdrPutFields(b,f->sub,p);
    case ROS_T_STRING:
      s=RosElemGetS(p);
      if (!s)
        s="";
      if (f->str_bound && StrLen(s)>f->str_bound) {
        RosMsgErr("string longer than its bound",f->name);
        return FALSE;
      }
      RosBufPutStr(b,s);
      return TRUE;
    case ROS_T_FLOAT32:
      RosBufPutN(b,RosF64ToF32Bits(RosElemGetF(p)),4);
      return TRUE;
    case ROS_T_FLOAT64:
      RosBufPutN(b,RosF64ToBits(RosElemGetF(p)),8);
      return TRUE;
  }
  RosBufPutN(b,RosElemGetI(f,p),RosKindWireSize(f->kind));
  return TRUE;
}

Bool RosCdrPutFields(CRosBuf *b,CRosType *t,U8 *m)
{
  I64 i,j,n;
  CRosField *f;
  for (i=0;i<t->fields->len;i++) {
    f=RosVecGetI64(t->fields,i);
    n=RosFieldLen(f,m);
    if (f->arr==ROS_ARR_SEQ) {
      if (f->arr_len && n>f->arr_len) {
        RosMsgErr("sequence longer than its bound",f->name);
        return FALSE;
      }
      RosBufPutN(b,n,4);
      if (f->stride==1 && f->kind!=ROS_T_BOOL) {	//byte arrays: one copy
        RosBufPutBytes(b,RosFieldElem(f,m,0),n);
        goto next_cdr_1;
      }
    }
    for (j=0;j<n;j++)
      if (!RosCdrPutElem(b,f,RosFieldElem(f,m,j)))
        return FALSE;
    next_cdr_1:;
  }
  return TRUE;
}

//Serialize m (with the 4-byte header) onto the end of b.
Bool RosCdrSer(CRosType *t,U8 *m,CRosBuf *b)
{
  RosBufPutN(b,0x0100,2);	//00 01: CDR little endian
  RosBufPutN(b,0,2);		//options
  b->origin=b->len;
  return RosCdrPutFields(b,t,m);
}

Bool RosCdrGetElem(CRosBuf *b,CRosField *f,U8 *p)
{
  U64 v;
  switch (f->kind) {
    case ROS_T_MSG:
      return RosCdrGetFields(b,f->sub,p);
    case ROS_T_STRING:
      RosElemSetS(p,RosBufGetStr(b));
      if (b->err)
        return FALSE;
      if (f->str_bound && StrLen(RosElemGetS(p))>f->str_bound) {
        RosMsgErr("string longer than its bound",f->name);
        return FALSE;
      }
      return TRUE;
    case ROS_T_FLOAT32:
      v=RosBufGetN(b,4);
      RosElemSetF(f,p,RosF32BitsToF64(v));
      return !b->err;
    case ROS_T_FLOAT64:
      v=RosBufGetN(b,8);
      RosElemSetF(f,p,RosBitsToF64(v));
      return !b->err;
  }
  v=RosBufGetN(b,RosKindWireSize(f->kind));
  RosElemSetI(f,p,v);
  return !b->err;
}

Bool RosCdrGetFields(CRosBuf *b,CRosType *t,U8 *m)
{
  I64 i,j,n;
  CRosField *f;
  for (i=0;i<t->fields->len;i++) {
    f=RosVecGetI64(t->fields,i);
    if (f->arr==ROS_ARR_SEQ) {
      n=RosBufGetN(b,4);
      if (b->err)
        return FALSE;
      //every element takes at least one byte, so a count bigger than
      //what's left is a lie: refuse before allocating anything
      if (n>b->len-b->pos || (f->arr_len && n>f->arr_len)) {
        RosMsgErr("bad sequence length",f->name);
        b->err=TRUE;
        return FALSE;
      }
      RosSeqResize(f,m,n);
      if (f->stride==1 && f->kind!=ROS_T_BOOL) {
        MemCpy(RosFieldElem(f,m,0),b->data+b->pos,n);
        b->pos+=n;
        goto next_cdr_2;
      }
    } else
      n=RosFieldLen(f,m);
    for (j=0;j<n;j++)
      if (!RosCdrGetElem(b,f,RosFieldElem(f,m,j))) {
        if (!b->err)
          return FALSE;
        RosMsgErr("message is cut short",f->name);
        return FALSE;
      }
    next_cdr_2:;
  }
  return TRUE;
}

//Deserialize len bytes (with header) into m, which must be a message
//of type t (from RosMsgNew); its old contents are replaced.
//FALSE + ros_msg_error on bad data.
Bool RosCdrDeser(CRosType *t,U8 *data,I64 len,U8 *m)
{
  CRosBuf *b;
  Bool ok;
  if (len<4 || data[0] || (data[1]!=0 && data[1]!=1)) {
    RosMsgErr("not plain CDR data",NULL);
    return FALSE;
  }
  RosMsgFini(t,m);
  MemSet(m,0,t->size);
  RosMsgInit(t,m);
  b=RosBufReader(data,len);
  b->swap=data[1]==0;
  b->pos=4;
  b->origin=4;
  ok=RosCdrGetFields(b,t,m);
  RosBufDel(b);
  return ok;
}
