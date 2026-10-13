//Msg: create / free / copy / print / set messages of any type,
//driven by the CRosType description.
//
//  CRosType *t=RosTypeFind("geometry_msgs/msg/Twist");
//  CMsg_geometry_msgs_Twist *m=RosMsgNew(t);
//  RosMsgSet(t,m,"linear.x","0.5");
//  RosMsgPrint(t,m);
//  RosMsgDel(t,m);

//Pointer to a field's storage inside message m.
U8 *RosFieldPtr(CRosField *f,U8 *m)
{
  return m+f->offset;
}

//Number of elements in an array/sequence field.
I64 RosFieldLen(CRosField *f,U8 *m)
{
  CRosSeq *q;
  if (f->arr==ROS_ARR_FIXED)
    return f->arr_len;
  if (f->arr==ROS_ARR_SEQ) {
    q=m+f->offset;
    return q->len;
  }
  return 1;
}

//Pointer to element i of field f (i=0 for scalars).
U8 *RosFieldElem(CRosField *f,U8 *m,I64 i)
{
  CRosSeq *q;
  if (f->arr==ROS_ARR_SEQ) {
    q=m+f->offset;
    return q->data+i*f->stride;
  }
  return m+f->offset+i*f->stride;
}

//Read/write an integer element (any int kind, bool, byte, char).
I64 RosElemGetI(CRosField *f,U8 *p)
{
  I64 *ip=p;
  if (f->stride==1)
    return RosIntNorm(f->kind,*p);
  return *ip;
}

U0 RosElemSetI(CRosField *f,U8 *p,I64 v)
{
  I64 *ip=p;
  v=RosIntNorm(f->kind,v);
  if (f->stride==1)
    *p=v;
  else
    *ip=v;
}

F64 RosElemGetF(U8 *p)
{
  F64 *fp=p;
  return *fp;
}

U0 RosElemSetF(CRosField *f,U8 *p,F64 v)
{
  F64 *fp=p;
  if (f->kind==ROS_T_FLOAT32)
    v=RosF32Round(v);	//what the wire can carry
  *fp=v;
}

U8 *RosElemGetS(U8 *p)
{
  U8 **sp=p;
  return *sp;
}

//Takes ownership of s.
U0 RosElemSetS(U8 *p,U8 *s)
{
  U8 **sp=p;
  Free(*sp);
  *sp=s;
}

extern U0 RosMsgInit(CRosType *t,U8 *m);
extern U0 RosMsgFini(CRosType *t,U8 *m);

//Resize sequence field f of m to n elements. New elements are zeroed
//(and initialized, for messages). Removed elements are freed.
U0 RosSeqResize(CRosField *f,U8 *m,I64 n)
{
  CRosSeq *q=m+f->offset;
  U8 *d,*p;
  I64 i,cap;
  for (i=n;i<q->len;i++) {
    p=q->data+i*f->stride;
    if (f->kind==ROS_T_MSG)
      RosMsgFini(f->sub,p);
    else if (f->kind==ROS_T_STRING)
      RosElemSetS(p,NULL);
  }
  if (n>q->cap) {
    cap=q->cap*2;
    if (cap<n)
      cap=n;
    d=CAlloc(cap*f->stride);
    if (q->len)
      MemCpy(d,q->data,q->len*f->stride);
    Free(q->data);
    q->data=d;
    q->cap=cap;
  }
  if (n>q->len) {
    MemSet(q->data+q->len*f->stride,0,(n-q->len)*f->stride);
    if (f->kind==ROS_T_MSG)
      for (i=q->len;i<n;i++)
        RosMsgInit(f->sub,q->data+i*f->stride);
  }
  q->len=n;
}

//Set element i of f from text. FALSE on bad text.
Bool RosElemSetText(CRosField *f,U8 *p,U8 *text)
{
  I64 iv=0;
  F64 fv=0;
  U8 *sv=NULL;
  if (!RosValParse(f->kind,text,&iv,&fv,&sv))
    return FALSE;
  if (f->kind==ROS_T_STRING) {
    if (f->str_bound && StrLen(sv)>f->str_bound) {
      RosMsgErr("string longer than its bound",f->name);
      Free(sv);
      return FALSE;
    }
    RosElemSetS(p,sv);
  } else if (RosKindIsFloat(f->kind))
    RosElemSetF(f,p,fv);
  else
    RosElemSetI(f,p,iv);
  return TRUE;
}

//Set a whole array/sequence field from "[a, b, c]".
Bool RosFieldSetList(CRosField *f,U8 *m,U8 *text)
{
  CRosVec *items=RosListSplit(text);
  I64 i;
  Bool ok=TRUE;
  if (!items) {
    RosMsgErr("expected a [a, b] list",f->name);
    return FALSE;
  }
  if (f->arr==ROS_ARR_FIXED && items->len!=f->arr_len) {
    RosMsgErr("wrong number of values for fixed array",f->name);
    ok=FALSE;
  } else if (f->arr==ROS_ARR_SEQ && f->arr_len && items->len>f->arr_len) {
    RosMsgErr("too many values for bounded sequence",f->name);
    ok=FALSE;
  }
  if (ok && f->arr==ROS_ARR_SEQ)
    RosSeqResize(f,m,items->len);
  for (i=0;ok && i<items->len;i++)
    ok=RosElemSetText(f,RosFieldElem(f,m,i),RosVecGetI64(items,i));
  RosStrVecDel(items);
  return ok;
}

//Fill in default values and set up nested messages. m must be zeroed.
U0 RosMsgInit(CRosType *t,U8 *m)
{
  I64 i,j,n;
  CRosField *f;
  for (i=0;i<t->fields->len;i++) {
    f=RosVecGetI64(t->fields,i);
    if (f->kind==ROS_T_MSG && f->arr!=ROS_ARR_SEQ) {
      n=RosFieldLen(f,m);
      for (j=0;j<n;j++)
        RosMsgInit(f->sub,RosFieldElem(f,m,j));
    }
    if (f->def) {
      if (f->arr==ROS_ARR_NONE)
        RosElemSetText(f,RosFieldPtr(f,m),f->def);
      else
        RosFieldSetList(f,m,f->def);
    }
  }
}

//Free everything m owns (strings, sequences), not m itself.
U0 RosMsgFini(CRosType *t,U8 *m)
{
  I64 i,j,n;
  CRosField *f;
  CRosSeq *q;
  if (!t->owns_mem)
    return;
  for (i=0;i<t->fields->len;i++) {
    f=RosVecGetI64(t->fields,i);
    if (f->kind==ROS_T_STRING || f->kind==ROS_T_MSG) {
      n=RosFieldLen(f,m);
      for (j=0;j<n;j++) {
        if (f->kind==ROS_T_STRING)
          RosElemSetS(RosFieldElem(f,m,j),NULL);
        else
          RosMsgFini(f->sub,RosFieldElem(f,m,j));
      }
    }
    if (f->arr==ROS_ARR_SEQ) {
      q=m+f->offset;
      Free(q->data);
      q->data=NULL;
      q->len=0;
      q->cap=0;
    }
  }
}

//New message with defaults filled in.
U8 *RosMsgNew(CRosType *t)
{
  U8 *m=CAlloc(t->size);
  RosMsgInit(t,m);
  return m;
}

U0 RosMsgDel(CRosType *t,U8 *m)
{
  if (!m)
    return;
  RosMsgFini(t,m);
  Free(m);
}

//After a raw MemCpy of a message, give dst its own copies of
//everything it points to.
U0 RosMsgCopyFix(CRosType *t,U8 *m)
{
  I64 i,j,n;
  CRosField *f;
  CRosSeq *q;
  U8 *d,**sp;
  if (!t->owns_mem)
    return;
  for (i=0;i<t->fields->len;i++) {
    f=RosVecGetI64(t->fields,i);
    if (f->arr==ROS_ARR_SEQ) {
      q=m+f->offset;
      if (q->len) {
        d=MAlloc(q->len*f->stride);
        MemCpy(d,q->data,q->len*f->stride);
        q->data=d;
        q->cap=q->len;
      } else {
        q->data=NULL;
        q->cap=0;
      }
    }
    if (f->kind==ROS_T_STRING || f->kind==ROS_T_MSG) {
      n=RosFieldLen(f,m);
      for (j=0;j<n;j++) {
        if (f->kind==ROS_T_STRING) {
          sp=RosFieldElem(f,m,j);
          if (*sp)
            *sp=StrNew(*sp);
        } else
          RosMsgCopyFix(f->sub,RosFieldElem(f,m,j));
      }
    }
  }
}

//Deep copy src into dst (dst's old contents are freed).
U0 RosMsgCopy(CRosType *t,U8 *dst,U8 *src)
{
  if (dst==src)
    return;
  RosMsgFini(t,dst);
  MemCpy(dst,src,t->size);
  RosMsgCopyFix(t,dst);
}

//New deep copy of src.
U8 *RosMsgDup(CRosType *t,U8 *src)
{
  U8 *m=CAlloc(t->size);
  MemCpy(m,src,t->size);
  RosMsgCopyFix(t,m);
  return m;
}

//---- YAML (same shape as ros2 topic echo)

U0 RosYamlIndent(CRosStrBuf *sb,I64 n)
{
  while (n>0) {
    RosStrBufAddC(sb,' ');
    n--;
  }
}

U0 RosYamlScalar(CRosStrBuf *sb,CRosField *f,U8 *p)
{
  if (f->kind==ROS_T_STRING)
    RosStrBufAddQuoted(sb,RosElemGetS(p));
  else if (RosKindIsFloat(f->kind))
    RosStrBufAddF64(sb,RosElemGetF(p));
  else if (f->kind==ROS_T_BOOL) {
    if (RosElemGetI(f,p))
      RosStrBufAdd(sb,"true");
    else
      RosStrBufAdd(sb,"false");
  } else if (f->kind==ROS_T_UINT64)
    RosStrBufAddU64(sb,RosElemGetI(f,p));
  else
    RosStrBufAddI64(sb,RosElemGetI(f,p));
}

extern U0 RosYamlFields(CRosStrBuf *sb,CRosType *t,U8 *m,I64 indent,Bool first_inline);

//Write one field "name: value" (or a nested block) at indent. If
//first_inline, the caller already wrote the indent (a "- " list item).
U0 RosYamlField(CRosStrBuf *sb,CRosField *f,U8 *m,I64 indent,Bool in_list)
{
  I64 n,i;
  if (!in_list)
    RosYamlIndent(sb,indent);
  RosStrBufAdd(sb,f->name);
  RosStrBufAddC(sb,':');
  if (f->arr==ROS_ARR_NONE) {
    if (f->kind==ROS_T_MSG) {
      RosStrBufAddC(sb,'\n');
      RosYamlFields(sb,f->sub,RosFieldPtr(f,m),indent+2,FALSE);
    } else {
      RosStrBufAddC(sb,' ');
      RosYamlScalar(sb,f,RosFieldPtr(f,m));
      RosStrBufAddC(sb,'\n');
    }
    return;
  }
  n=RosFieldLen(f,m);
  if (!n) {
    RosStrBufAdd(sb," []\n");
    return;
  }
  RosStrBufAddC(sb,'\n');
  for (i=0;i<n;i++) {
    RosYamlIndent(sb,indent);
    RosStrBufAdd(sb,"- ");
    if (f->kind==ROS_T_MSG)
      RosYamlFields(sb,f->sub,RosFieldElem(f,m,i),indent+2,TRUE);
    else {
      RosYamlScalar(sb,f,RosFieldElem(f,m,i));
      RosStrBufAddC(sb,'\n');
    }
  }
}

U0 RosYamlFields(CRosStrBuf *sb,CRosType *t,U8 *m,I64 indent,Bool first_inline)
{
  I64 i;
  for (i=0;i<t->fields->len;i++)
    RosYamlField(sb,RosVecGetI64(t->fields,i),m,indent,first_inline && i==0);
}

//Message as YAML text (yours to Free).
U8 *RosMsgToYaml(CRosType *t,U8 *m)
{
  CRosStrBuf *sb=RosStrBufNew();
  RosYamlFields(sb,t,m,0,FALSE);
  return RosStrBufTake(sb);
}

//Print like ros2 topic echo (with the --- separator).
U0 RosMsgPrint(CRosType *t,U8 *m)
{
  U8 *s=RosMsgToYaml(t,m);
  Print("%s---\n",s);
  Free(s);
}

//---- paths: "linear.x", "poses[2].pose.position.y", "ranges"

//Find the field a path points at. On success *mp is the message that
//holds it and *idx the element index (-1 if no [i] was given).
//grow=TRUE makes sequences big enough for the index.
CRosField *RosMsgPath(CRosType *t,U8 *m,U8 *path,U8 **mp,I64 *idx,Bool grow)
{
  CRosVec *parts=RosStrSplit(path,'.');
  CRosField *f=NULL;
  U8 *part,*name;
  I64 i,lb,ix;
  for (i=0;i<parts->len;i++) {
    part=RosVecGetI64(parts,i);
    ix=-1;
    lb=RosStrFindC(part,'[');
    if (lb>=0) {
      name=RosStrSub(part,lb+1,StrLen(part)-lb-2);
      if (part[StrLen(part)-1]!=']' || !RosStrToI64(name,&ix) || ix<0) {
        RosMsgErr("bad index",part);
        Free(name);
        f=NULL;
        break;
      }
      Free(name);
      name=RosStrSub(part,0,lb);
    } else
      name=StrNew(part);
    f=RosTypeField(t,name);
    Free(name);
    if (!f) {
      RosMsgErr("no such field",part);
      break;
    }
    if (ix>=0) {
      if (f->arr==ROS_ARR_NONE) {
        RosMsgErr("not an array",part);
        f=NULL;
        break;
      }
      if (ix>=RosFieldLen(f,m)) {
        if (f->arr==ROS_ARR_FIXED || !grow ||
	    (f->arr_len && ix>=f->arr_len)) {
          RosMsgErr("index out of range",part);
          f=NULL;
          break;
        }
        RosSeqResize(f,m,ix+1);
      }
    }
    if (i==parts->len-1) {
      *mp=m;
      *idx=ix;
      break;
    }
    if (f->kind!=ROS_T_MSG || (f->arr!=ROS_ARR_NONE && ix<0)) {
      RosMsgErr("can't go inside",part);
      f=NULL;
      break;
    }
    if (ix<0)
      ix=0;
    m=RosFieldElem(f,m,ix);
    t=f->sub;
  }
  RosStrVecDel(parts);
  return f;
}

//Set a field from text: RosMsgSet(t,m,"linear.x","0.5"),
//(t,m,"ranges","[1.0, 2.0]"), (t,m,"poses[0].header.frame_id","map").
//Sequences grow to fit an index. FALSE + ros_msg_error on failure.
Bool RosMsgSet(CRosType *t,U8 *m,U8 *path,U8 *text)
{
  U8 *mm;
  I64 idx;
  CRosField *f=RosMsgPath(t,m,path,&mm,&idx,TRUE);
  if (!f)
    return FALSE;
  if (f->kind==ROS_T_MSG) {
    RosMsgErr("can't set a whole message from text",path);
    return FALSE;
  }
  if (f->arr!=ROS_ARR_NONE && idx<0)
    return RosFieldSetList(f,mm,text);
  if (idx<0)
    idx=0;
  return RosElemSetText(f,RosFieldElem(f,mm,idx),text);
}

//YAML text of the value at path (yours to Free), NULL if bad path.
U8 *RosMsgGet(CRosType *t,U8 *m,U8 *path)
{
  U8 *mm;
  I64 idx;
  CRosStrBuf *sb;
  CRosField *f=RosMsgPath(t,m,path,&mm,&idx,FALSE);
  if (!f)
    return NULL;
  sb=RosStrBufNew();
  if (idx<0 && (f->arr!=ROS_ARR_NONE || f->kind==ROS_T_MSG))
    RosYamlField(sb,f,mm,0,FALSE);
  else if (f->kind==ROS_T_MSG)
    RosYamlFields(sb,f->sub,RosFieldElem(f,mm,idx),0,FALSE);
  else {
    if (idx<0)
      idx=0;
    RosYamlScalar(sb,f,RosFieldElem(f,mm,idx));
  }
  return RosStrBufTake(sb);
}
