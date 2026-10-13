//Parse: .msg / .srv text -> CRosType.
//
//Supported: all primitive types, string<=N, nested messages
//(Name, pkg/Name, pkg/msg/Name, Header), fixed arrays T[N],
//sequences T[] and T[<=N], constants (TYPE NAME=value), default
//values (TYPE name value, arrays as [a, b]) and # comments.
//Not supported: wstring.

#define ROS_SEQ_SIZE	24	//sizeof(CRosSeq)

//Check a default value for field f. TRUE if it parses.
Bool RosFieldDefaultOk(CRosField *f)
{
  CRosVec *items;
  I64 i,iv;
  F64 fv;
  U8 *sv;
  Bool ok=TRUE;
  if (f->kind==ROS_T_MSG) {
    RosMsgErr("message fields can't have defaults",f->name);
    return FALSE;
  }
  if (f->arr==ROS_ARR_NONE) {
    sv=NULL;
    ok=RosValParse(f->kind,f->def,&iv,&fv,&sv);
    Free(sv);
    return ok;
  }
  items=RosListSplit(f->def);
  if (!items) {
    RosMsgErr("array default must look like [a, b]",f->name);
    return FALSE;
  }
  if (f->arr==ROS_ARR_FIXED && items->len!=f->arr_len) {
    RosMsgErr("wrong number of default values",f->name);
    ok=FALSE;
  }
  if (f->arr==ROS_ARR_SEQ && f->arr_len && items->len>f->arr_len) {
    RosMsgErr("too many default values",f->name);
    ok=FALSE;
  }
  for (i=0;ok && i<items->len;i++) {
    sv=NULL;
    ok=RosValParse(f->kind,RosVecGetI64(items,i),&iv,&fv,&sv);
    Free(sv);
  }
  RosStrVecDel(items);
  return ok;
}

//Parse "float64[<=3]" etc. into f. TRUE on success.
Bool RosFieldParseType(CRosType *t,CRosField *f,U8 *type_text)
{
  U8 *base,*inner,*full;
  I64 n=StrLen(type_text),lb,bound;
  CRosStrBuf *sb;
  base=StrNew(type_text);
  if (n && type_text[n-1]==']') {
    lb=n-1;
    while (lb>0 && type_text[lb]!='[')
      lb--;
    inner=RosStrSub(type_text,lb+1,n-lb-2);
    Free(base);
    base=RosStrSub(type_text,0,lb);
    if (!StrLen(inner))
      f->arr=ROS_ARR_SEQ;
    else if (RosStrStartsWith(inner,"<=")) {
      f->arr=ROS_ARR_SEQ;
      if (!RosStrToI64(inner+2,&f->arr_len) || f->arr_len<1) {
        RosMsgErr("bad sequence bound",type_text);
        Free(inner);
        Free(base);
        return FALSE;
      }
    } else {
      f->arr=ROS_ARR_FIXED;
      if (!RosStrToI64(inner,&f->arr_len) || f->arr_len<1) {
        RosMsgErr("bad array size",type_text);
        Free(inner);
        Free(base);
        return FALSE;
      }
    }
    Free(inner);
  }
  if (RosStrStartsWith(base,"string<=")) {
    f->kind=ROS_T_STRING;
    if (!RosStrToI64(base+8,&bound) || bound<1) {
      RosMsgErr("bad string bound",type_text);
      Free(base);
      return FALSE;
    }
    f->str_bound=bound;
  } else if (RosStrStartsWith(base,"wstring")) {
    RosMsgErr("wstring is not supported",type_text);
    Free(base);
    return FALSE;
  } else
    f->kind=RosKindFromName(base);
  if (!f->kind) {
    f->kind=ROS_T_MSG;
    if (!StrCmp(base,"Header"))
      full=StrNew("std_msgs/msg/Header");
    else if (RosStrFindC(base,'/')>=0)
      full=RosTypeFullName(base);
    else {
      sb=RosStrBufNew();
      RosStrBufAdd(sb,t->pkg);
      RosStrBufAdd(sb,"/msg/");
      RosStrBufAdd(sb,base);
      full=RosStrBufTake(sb);
    }
    f->sub=RosTypeFind(full);
    if (!f->sub) {
      RosMsgErr("unknown type (define it first)",full);
      Free(full);
      Free(base);
      return FALSE;
    }
    Free(full);
  }
  Free(base);
  return TRUE;
}

//Cut a # comment that isn't inside quotes.
U0 RosMsgCutComment(U8 *s)
{
  U8 q=0;
  while (*s) {
    if (q) {
      if (*s==q)
        q=0;
    } else if (*s=='\'' || *s=='"')
      q=*s;
    else if (*s=='#') {
      *s=0;
      return;
    }
    s++;
  }
}

//Parse one line of a .msg into t. TRUE if fine (blank lines too).
Bool RosMsgParseLine(CRosType *t,U8 *raw)
{
  U8 *line=StrNew(raw),*s,*type_text,*name,*rest;
  I64 i,j;
  CRosField *f;
  CRosConst *c;
  Bool ok=TRUE;
  RosMsgCutComment(line);
  s=RosStrTrim(line);
  Free(line);
  if (!StrLen(s)) {
    Free(s);
    return TRUE;
  }
  i=0;
  while (s[i] && !RosIsSpace(s[i]))
    i++;
  type_text=RosStrSub(s,0,i);
  while (RosIsSpace(s[i]))
    i++;
  j=i;
  while (s[j] && !RosIsSpace(s[j]) && s[j]!='=')
    j++;
  name=RosStrSub(s,i,j-i);
  rest=RosStrTrim(s+j);
  if (!StrLen(name)) {
    RosMsgErr("missing field name",s);
    ok=FALSE;
  } else if (rest[0]=='=') {
    c=CAlloc(sizeof(CRosConst));
    c->name=StrNew(name);
    c->kind=RosKindFromName(type_text);
    if (!c->kind && RosStrStartsWith(type_text,"string<="))
      c->kind=ROS_T_STRING;
    if (!c->kind) {
      RosMsgErr("constants must be a primitive type",s);
      ok=FALSE;
    } else
      ok=RosValParse(c->kind,rest+1,&c->i,&c->f,&c->s);
    if (ok)
      RosVecPushI64(t->consts,c);
    else {
      Free(c->name);
      Free(c);
    }
  } else {
    f=CAlloc(sizeof(CRosField));
    f->name=StrNew(name);
    ok=RosFieldParseType(t,f,type_text);
    if (ok && StrLen(rest)) {
      f->def=StrNew(rest);
      ok=RosFieldDefaultOk(f);
    }
    if (ok)
      RosVecPushI64(t->fields,f);
    else {
      Free(f->name);
      Free(f->def);
      Free(f);
    }
  }
  Free(type_text);
  Free(name);
  Free(rest);
  Free(s);
  return ok;
}

//Work out offsets and sizes (see the layout rules in Type.HC).
U0 RosTypeLayout(CRosType *t)
{
  I64 i,elem,off=0;
  CRosField *f;
  for (i=0;i<t->fields->len;i++) {
    f=RosVecGetI64(t->fields,i);
    if (f->kind==ROS_T_MSG)
      elem=f->sub->size;
    else if (RosKindIsByte(f->kind) && f->arr!=ROS_ARR_NONE)
      elem=1;
    else
      elem=8;
    f->stride=elem;
    if (f->arr==ROS_ARR_NONE)
      f->size=elem;
    else if (f->arr==ROS_ARR_FIXED)
      f->size=(elem*f->arr_len+7)/8*8;
    else
      f->size=ROS_SEQ_SIZE;
    f->offset=off;
    off+=f->size;
    if (f->kind==ROS_T_STRING || f->arr==ROS_ARR_SEQ ||
	(f->kind==ROS_T_MSG && f->sub->owns_mem))
      t->owns_mem=TRUE;
  }
  t->size=off;
}

U0 RosTypeFree(CRosType *t)
{
  I64 i;
  CRosField *f;
  CRosConst *c;
  for (i=0;i<t->fields->len;i++) {
    f=RosVecGetI64(t->fields,i);
    Free(f->name);
    Free(f->def);
    Free(f);
  }
  for (i=0;i<t->consts->len;i++) {
    c=RosVecGetI64(t->consts,i);
    Free(c->name);
    Free(c->s);
    Free(c);
  }
  RosVecDel(t->fields);
  RosVecDel(t->consts);
  Free(t->name);
  Free(t->pkg);
  Free(t->base);
  Free(t->def);
  Free(t);
}

//Define a message type from .msg text. name is "pkg/msg/Name" (or
//"pkg/Name"). Returns the type, or NULL with ros_msg_error set.
//Defining the same name again returns the existing type.
CRosType *RosMsgDefine(U8 *name,U8 *text)
{
  CRosType *t;
  CRosVec *lines;
  CRosField *f;
  U8 *full=RosTypeFullName(name);
  I64 i,slash;
  CRosStrBuf *sb;
  if (!ros_types)
    ros_types=RosMapNew();
  t=RosMapGet(ros_types,full);
  if (t) {
    Free(full);
    return t;
  }
  t=CAlloc(sizeof(CRosType));
  t->name=full;
  slash=RosStrFindC(full,'/');
  t->pkg=RosStrSub(full,0,slash);
  i=StrLen(full);
  while (i>0 && full[i-1]!='/')
    i--;
  t->base=StrNew(full+i);
  t->def=StrNew(text);
  t->fields=RosVecNew(8);
  t->consts=RosVecNew(8);
  lines=RosStrSplit(text,'\n');
  for (i=0;i<lines->len;i++)
    if (!RosMsgParseLine(t,RosVecGetI64(lines,i))) {
      sb=RosStrBufNew();
      RosStrBufAdd(sb,ros_msg_error);
      RosStrBufAdd(sb," (");
      RosStrBufAdd(sb,full);
      RosStrBufAdd(sb," line ");
      RosStrBufAddI64(sb,i+1);
      RosStrBufAdd(sb,")");
      RosMsgErr(sb->buf,NULL);
      RosStrBufDel(sb);
      RosStrVecDel(lines);
      RosTypeFree(t);
      return NULL;
    }
  RosStrVecDel(lines);
  if (!t->fields->len) {
    //same as rosidl: empty messages get one uint8
    f=CAlloc(sizeof(CRosField));
    f->name=StrNew("structure_needs_at_least_one_member");
    f->kind=ROS_T_UINT8;
    RosVecPushI64(t->fields,f);
  }
  RosTypeLayout(t);
  RosMapSet(ros_types,full,t);
  return t;
}

//Define a service "pkg/srv/Name" from .srv text (request --- response).
//Makes types "pkg/srv/Name_Request" and "pkg/srv/Name_Response".
CRosSrvType *RosSrvDefine(U8 *name,U8 *text)
{
  CRosSrvType *st;
  I64 i=0,cut=-1,n=StrLen(text);
  U8 *req_text,*resp_text;
  CRosStrBuf *sb;
  if (!ros_srv_types)
    ros_srv_types=RosMapNew();
  st=RosMapGet(ros_srv_types,name);
  if (st)
    return st;
  //find a line that is exactly ---
  while (i<n) {
    if ((i==0 || text[i-1]=='\n') && !StrNCmp(text+i,"---",3) &&
	(text[i+3]=='\n' || text[i+3]=='\r' || !text[i+3])) {
      cut=i;
      break;
    }
    i++;
  }
  if (cut<0) {
    RosMsgErr("service has no --- line",name);
    return NULL;
  }
  req_text=RosStrSub(text,0,cut);
  resp_text=StrNew(text+cut+3);
  st=CAlloc(sizeof(CRosSrvType));
  st->name=StrNew(name);
  sb=RosStrBufNew();
  RosStrBufAdd(sb,name);
  RosStrBufAdd(sb,"_Request");
  st->req=RosMsgDefine(sb->buf,req_text);
  RosStrBufClear(sb);
  RosStrBufAdd(sb,name);
  RosStrBufAdd(sb,"_Response");
  if (st->req)
    st->resp=RosMsgDefine(sb->buf,resp_text);
  RosStrBufDel(sb);
  Free(req_text);
  Free(resp_text);
  if (!st->req || !st->resp) {
    Free(st->name);
    Free(st);
    return NULL;
  }
  RosMapSet(ros_srv_types,name,st);
  return st;
}

CRosSrvType *RosSrvFind(U8 *name)
{
  if (!ros_srv_types)
    return NULL;
  return RosMapGet(ros_srv_types,name);
}
