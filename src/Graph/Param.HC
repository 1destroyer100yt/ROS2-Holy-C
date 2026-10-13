//Param: node parameters, like rclcpp.
//
//  RosParamDeclare(node,"rate","2.0");        //type comes from the value
//  F64 hz=RosParamF64(node,"rate");
//  RosParamSet(node,"rate","5.0");            //FALSE + ros_param_error if refused
//  RosParamLoadYaml(text);                    //ROS 2 params file -> overrides
//
//Values are rcl_interfaces/ParameterValue messages. Every node also
//serves get/set/list/describe_parameters (+ get_parameter_types,
//set_parameters_atomically) and publishes /parameter_events, so the
//ros2 param tools work the same way.
//
//Types follow ROS 2: a parameter keeps its type unless declared
//with dynamic typing. One exception: an integer is accepted for a
//double parameter (converted), since "rate: 2" in YAML is common.

#define ROS_PT_NOT_SET		0
#define ROS_PT_BOOL		1
#define ROS_PT_INTEGER		2
#define ROS_PT_DOUBLE		3
#define ROS_PT_STRING		4
#define ROS_PT_BYTE_ARRAY	5
#define ROS_PT_BOOL_ARRAY	6
#define ROS_PT_INTEGER_ARRAY	7
#define ROS_PT_DOUBLE_ARRAY	8
#define ROS_PT_STRING_ARRAY	9

class CRosParam
{
  U8 *name;
  CMsg_rcl_interfaces_ParameterValue *val;
  U8 *desc;
  Bool read_only,dynamic_typing;
};

class CRosOverride
{
  U8 *node_key,*name,*val;
};

U8 ros_param_error[128];
CRosVec *ros_param_overrides=NULL;	//CRosOverride *

U0 RosParamErr(U8 *what,U8 *name)
{
  CRosStrBuf *sb=RosStrBufNew();
  RosStrBufAdd(sb,what);
  if (name) {
    RosStrBufAdd(sb,": ");
    RosStrBufAdd(sb,name);
  }
  if (sb->len>120)
    sb->buf[120]=0;
  StrCpy(ros_param_error,sb->buf);
  RosStrBufDel(sb);
}

U8 *RosParamTypeName(I64 type)
{
  switch (type) {
    case ROS_PT_BOOL: return "bool";
    case ROS_PT_INTEGER: return "integer";
    case ROS_PT_DOUBLE: return "double";
    case ROS_PT_STRING: return "string";
    case ROS_PT_BYTE_ARRAY: return "byte_array";
    case ROS_PT_BOOL_ARRAY: return "bool_array";
    case ROS_PT_INTEGER_ARRAY: return "integer_array";
    case ROS_PT_DOUBLE_ARRAY: return "double_array";
    case ROS_PT_STRING_ARRAY: return "string_array";
  }
  return "not set";
}

//---- ParameterValue helpers

CMsg_rcl_interfaces_ParameterValue *RosPvNew()
{
  return RosMsgNew(ros_t_rcl_interfaces_ParameterValue);
}

U0 RosPvDel(CMsg_rcl_interfaces_ParameterValue *pv)
{
  RosMsgDel(ros_t_rcl_interfaces_ParameterValue,pv);
}

CMsg_rcl_interfaces_ParameterValue *RosPvDup(CMsg_rcl_interfaces_ParameterValue *pv)
{
  return RosMsgDup(ros_t_rcl_interfaces_ParameterValue,pv);
}

Bool RosTextIsBool(U8 *t)
{
  return !StrCmp(t,"true") || !StrCmp(t,"false") || !StrCmp(t,"True") || !StrCmp(t,"False");
}

//Parse value text into a new ParameterValue, working out the type:
//true/false, 3, 2.5, [1, 2], [1.5], [true], [a, 'b'], anything else
//is a string ('quoted' or not). NULL if a list can't be parsed.
CMsg_rcl_interfaces_ParameterValue *RosPvFromText(U8 *text)
{
  CMsg_rcl_interfaces_ParameterValue *pv=RosPvNew();
  CRosVec *items;
  U8 *t=RosStrTrim(text),*it,*field;
  I64 i,iv;
  F64 fv;
  Bool all_bool=TRUE,all_int=TRUE,all_num=TRUE,ok=TRUE;
  items=RosListSplit(t);
  if (items) {
    for (i=0;i<items->len;i++) {
      it=RosVecGetI64(items,i);
      if (!RosTextIsBool(it))
        all_bool=FALSE;
      if (!RosStrToI64(it,&iv))
        all_int=FALSE;
      if (!RosStrToF64(it,&fv))
        all_num=FALSE;
    }
    if (!items->len) {
      pv->type=ROS_PT_STRING_ARRAY;
      field="string_array_value";
    } else if (all_bool) {
      pv->type=ROS_PT_BOOL_ARRAY;
      field="bool_array_value";
    } else if (all_int) {
      pv->type=ROS_PT_INTEGER_ARRAY;
      field="integer_array_value";
    } else if (all_num) {
      pv->type=ROS_PT_DOUBLE_ARRAY;
      field="double_array_value";
    } else {
      pv->type=ROS_PT_STRING_ARRAY;
      field="string_array_value";
    }
    ok=RosMsgSet(ros_t_rcl_interfaces_ParameterValue,pv,field,t);
    RosStrVecDel(items);
  } else if (RosTextIsBool(t)) {
    pv->type=ROS_PT_BOOL;
    pv->bool_value=t[0]=='t' || t[0]=='T';
  } else if (RosStrToI64(t,&iv)) {
    pv->type=ROS_PT_INTEGER;
    pv->integer_value=iv;
  } else if (RosStrToF64(t,&fv)) {
    pv->type=ROS_PT_DOUBLE;
    pv->double_value=fv;
  } else {
    pv->type=ROS_PT_STRING;
    pv->string_value=RosStrUnquote(t);
  }
  Free(t);
  if (!ok) {
    RosParamErr("bad parameter value",text);
    RosPvDel(pv);
    return NULL;
  }
  return pv;
}

//Value as text that RosPvFromText reads back (strings 'quoted').
U8 *RosPvToText(CMsg_rcl_interfaces_ParameterValue *pv)
{
  CRosStrBuf *sb=RosStrBufNew();
  CRosField *f=NULL;
  I64 i,n;
  switch (pv->type) {
    case ROS_PT_BOOL:
      if (pv->bool_value)
        RosStrBufAdd(sb,"true");
      else
        RosStrBufAdd(sb,"false");
      break;
    case ROS_PT_INTEGER:
      RosStrBufAddI64(sb,pv->integer_value);
      break;
    case ROS_PT_DOUBLE:
      RosStrBufAddF64(sb,pv->double_value);
      break;
    case ROS_PT_STRING:
      RosStrBufAddQuoted(sb,pv->string_value);
      break;
    case ROS_PT_BYTE_ARRAY:
      f=RosTypeField(ros_t_rcl_interfaces_ParameterValue,"byte_array_value");
      break;
    case ROS_PT_BOOL_ARRAY:
      f=RosTypeField(ros_t_rcl_interfaces_ParameterValue,"bool_array_value");
      break;
    case ROS_PT_INTEGER_ARRAY:
      f=RosTypeField(ros_t_rcl_interfaces_ParameterValue,"integer_array_value");
      break;
    case ROS_PT_DOUBLE_ARRAY:
      f=RosTypeField(ros_t_rcl_interfaces_ParameterValue,"double_array_value");
      break;
    case ROS_PT_STRING_ARRAY:
      f=RosTypeField(ros_t_rcl_interfaces_ParameterValue,"string_array_value");
      break;
    default:
      RosStrBufAdd(sb,"(not set)");
  }
  if (f) {
    RosStrBufAddC(sb,'[');
    n=RosFieldLen(f,pv);
    for (i=0;i<n;i++) {
      if (i)
        RosStrBufAdd(sb,", ");
      RosYamlScalar(sb,f,RosFieldElem(f,pv,i));
    }
    RosStrBufAddC(sb,']');
  }
  return RosStrBufTake(sb);
}

//---- overrides (from YAML files / launch)

//Override value for node_key ("/ns/node", "node" or "/**") param name.
U0 RosParamOverride(U8 *node_key,U8 *name,U8 *val)
{
  CRosOverride *o=CAlloc(sizeof(CRosOverride));
  if (!ros_param_overrides)
    ros_param_overrides=RosVecNew(8);
  o->node_key=StrNew(node_key);
  o->name=StrNew(name);
  o->val=StrNew(val);
  RosVecPushI64(ros_param_overrides,o);
}

//Load a ROS 2 parameters YAML file's text:
//  /ns/node:  (or node:, or /**:)
//    ros__parameters:
//      name: value
//Applies to nodes created afterwards. FALSE + ros_param_error on error.
Bool RosParamLoadYaml(U8 *text)
{
  CRosVec *items=RosYamlFlatten(text);
  CRosYamlItem *it;
  U8 *key,*name;
  I64 i,cut;
  if (!items) {
    RosParamErr(ros_yaml_error,NULL);
    return FALSE;
  }
  for (i=0;i<items->len;i++) {
    it=RosVecGetI64(items,i);
    cut=RosStrFind(it->path,".ros__parameters.");
    if (cut<=0)
      continue;
    key=RosStrSub(it->path,0,cut);
    name=StrNew(it->path+cut+17);
    RosParamOverride(key,name,it->val);
    Free(key);
    Free(name);
  }
  RosYamlFree(items);
  return TRUE;
}

Bool RosParamFile(U8 *path)
{
  U8 *text=RosFileRead(path,NULL);
  Bool ok;
  if (!text) {
    RosParamErr("can't read",path);
    return FALSE;
  }
  ok=RosParamLoadYaml(text);
  Free(text);
  return ok;
}

Bool RosOverrideMatches(CRosNode *n,U8 *key)
{
  if (!StrCmp(key,"/**") || !StrCmp(key,"**") || !StrCmp(key,n->fqn))
    return TRUE;
  return key[0]!='/' && !StrCmp(key,n->fqn+1);
}

//Set an override for one node only (before declaring).
U0 RosNodeParamOverride(CRosNode *n,U8 *name,U8 *val)
{
  U8 *old=RosMapGet(n->overrides,name);
  Free(old);
  RosMapSet(n->overrides,name,StrNew(val));
}

//---- parameter events

U0 RosParamEventAdd(CMsg_rcl_interfaces_ParameterEvent *ev,U8 *list,U8 *name,
	CMsg_rcl_interfaces_ParameterValue *pv)
{
  CRosField *f=RosTypeField(ros_t_rcl_interfaces_ParameterEvent,list);
  CMsg_rcl_interfaces_Parameter *p;
  I64 n=RosFieldLen(f,ev);
  RosSeqResize(f,ev,n+1);
  p=RosFieldElem(f,ev,n);
  p->name=StrNew(name);
  if (pv)
    RosMsgCopy(ros_t_rcl_interfaces_ParameterValue,&p->value,pv);
}

//list is "new_parameters", "changed_parameters" or "deleted_parameters"
U0 RosParamEvent(CRosNode *n,U8 *list,U8 *name,CMsg_rcl_interfaces_ParameterValue *pv)
{
  CMsg_rcl_interfaces_ParameterEvent *ev;
  I64 now;
  if (!n->param_events_pub)
    return;
  ev=RosPubLoan(n->param_events_pub);
  now=RosTimeSystemNs();
  ev->stamp.sec=RosTimeSec(now);
  ev->stamp.nanosec=RosTimeNsec(now);
  ev->node=StrNew(n->fqn);
  RosParamEventAdd(ev,list,name,pv);
  RosPublishLoaned(n->param_events_pub,ev);
}

//---- declare / get / set

CRosParam *RosParamFind(CRosNode *n,U8 *name)
{
  return RosMapGet(n->params,name);
}

Bool RosParamHas(CRosNode *n,U8 *name)
{
  return RosParamFind(n,name)!=NULL;
}

//Integer given for a double: convert in place. TRUE if types now fit.
Bool RosPvFitType(CMsg_rcl_interfaces_ParameterValue *pv,I64 want)
{
  CRosField *fi,*fd;
  I64 i,len;
  I64 *ip;
  F64 *dp;
  if (pv->type==want)
    return TRUE;
  if (want==ROS_PT_DOUBLE && pv->type==ROS_PT_INTEGER) {
    pv->type=ROS_PT_DOUBLE;
    pv->double_value=ToF64(pv->integer_value);
    pv->integer_value=0;
    return TRUE;
  }
  if (want==ROS_PT_DOUBLE_ARRAY && pv->type==ROS_PT_INTEGER_ARRAY) {
    fi=RosTypeField(ros_t_rcl_interfaces_ParameterValue,"integer_array_value");
    fd=RosTypeField(ros_t_rcl_interfaces_ParameterValue,"double_array_value");
    len=RosFieldLen(fi,pv);
    RosSeqResize(fd,pv,len);
    ip=pv->integer_array_value.data;
    dp=pv->double_array_value.data;
    for (i=0;i<len;i++)
      dp[i]=ToF64(ip[i]);
    RosSeqResize(fi,pv,0);
    pv->type=ROS_PT_DOUBLE_ARRAY;
    return TRUE;
  }
  return FALSE;
}

//Declare with a default given as text (NULL: no default, any type
//allowed later). An override (YAML/launch) wins over the default.
//Returns the parameter, or NULL + ros_param_error.
CRosParam *RosParamDeclareEx(CRosNode *n,U8 *name,U8 *default_text,U8 *desc,Bool read_only)
{
  CRosParam *p;
  CMsg_rcl_interfaces_ParameterValue *pv,*def=NULL;
  U8 *ov;
  if (!name || !name[0]) {
    RosParamErr("empty parameter name",NULL);
    return NULL;
  }
  if (RosParamFind(n,name)) {
    RosParamErr("parameter already declared",name);
    return NULL;
  }
  if (default_text) {
    def=RosPvFromText(default_text);
    if (!def)
      return NULL;
  }
  ov=RosMapGet(n->overrides,name);
  if (ov) {
    pv=RosPvFromText(ov);
    if (!pv) {
      RosPvDel(def);
      return NULL;
    }
    if (def && !RosPvFitType(pv,def->type)) {
      RosParamErr("override has the wrong type",name);
      RosPvDel(pv);
      RosPvDel(def);
      return NULL;
    }
    RosPvDel(def);
  } else if (def)
    pv=def;
  else
    pv=RosPvNew();
  p=CAlloc(sizeof(CRosParam));
  p->name=StrNew(name);
  p->val=pv;
  p->dynamic_typing=default_text==NULL;
  p->read_only=read_only;
  if (desc)
    p->desc=StrNew(desc);
  RosMapSet(n->params,name,p);
  RosVecPushI64(n->param_order,p->name);
  RosParamEvent(n,"new_parameters",name,pv);
  return p;
}

CRosParam *RosParamDeclare(CRosNode *n,U8 *name,U8 *default_text)
{
  return RosParamDeclareEx(n,name,default_text,NULL,FALSE);
}

U0 RosParamFree(CRosParam *p)
{
  RosPvDel(p->val);
  Free(p->name);
  Free(p->desc);
  Free(p);
}

Bool RosParamUndeclare(CRosNode *n,U8 *name)
{
  CRosParam *p=RosParamFind(n,name);
  if (!p) {
    RosParamErr("not declared",name);
    return FALSE;
  }
  if (p->read_only) {
    RosParamErr("read only",name);
    return FALSE;
  }
  RosMapRemove(n->params,name);
  RosVecRemovePtr(n->param_order,p->name);
  RosParamEvent(n,"deleted_parameters",name,NULL);
  RosParamFree(p);
  return TRUE;
}

//Could pv be set on parameter name? (no changes made)
Bool RosParamCheck(CRosNode *n,U8 *name,CMsg_rcl_interfaces_ParameterValue *pv)
{
  CRosParam *p=RosParamFind(n,name);
  if (!p) {
    RosParamErr("parameter not declared",name);
    return FALSE;
  }
  if (p->read_only) {
    RosParamErr("parameter is read only",name);
    return FALSE;
  }
  if (!p->dynamic_typing && !RosPvFitType(pv,p->val->type)) {
    RosParamErr("wrong type for parameter",name);
    return FALSE;
  }
  if (n->param_cb && !(*n->param_cb)(n,name,pv,n->param_cb_user)) {
    RosParamErr("rejected by the node",name);
    return FALSE;
  }
  return TRUE;
}

U0 RosParamApply(CRosNode *n,U8 *name,CMsg_rcl_interfaces_ParameterValue *pv)
{
  CRosParam *p=RosParamFind(n,name);
  RosMsgCopy(ros_t_rcl_interfaces_ParameterValue,p->val,pv);
  RosParamEvent(n,"changed_parameters",name,p->val);
}

//Set from a ParameterValue (copied). FALSE + ros_param_error if refused.
Bool RosParamSetPv(CRosNode *n,U8 *name,CMsg_rcl_interfaces_ParameterValue *pv)
{
  CMsg_rcl_interfaces_ParameterValue *v=RosPvDup(pv);
  Bool ok=RosParamCheck(n,name,v);
  if (ok)
    RosParamApply(n,name,v);
  RosPvDel(v);
  return ok;
}

//Set from text, e.g. RosParamSet(n,"rate","5.0").
Bool RosParamSet(CRosNode *n,U8 *name,U8 *text)
{
  CMsg_rcl_interfaces_ParameterValue *pv=RosPvFromText(text);
  Bool ok;
  if (!pv)
    return FALSE;
  ok=RosParamSetPv(n,name,pv);
  RosPvDel(pv);
  return ok;
}

//Called before every set; return FALSE to refuse the new value.
U0 RosParamOnSet(CRosNode *n,Bool (*cb)(CRosNode *n,U8 *name,U8 *val,U8 *user),U8 *user)
{
  n->param_cb=cb;
  n->param_cb_user=user;
}

I64 RosParamType(CRosNode *n,U8 *name)
{
  CRosParam *p=RosParamFind(n,name);
  if (!p)
    return ROS_PT_NOT_SET;
  return p->val->type;
}

I64 RosParamI64(CRosNode *n,U8 *name)
{
  CRosParam *p=RosParamFind(n,name);
  if (!p)
    return 0;
  if (p->val->type==ROS_PT_DOUBLE)
    return ToI64(p->val->double_value);
  return p->val->integer_value;
}

F64 RosParamF64(CRosNode *n,U8 *name)
{
  CRosParam *p=RosParamFind(n,name);
  if (!p)
    return 0.0;
  if (p->val->type==ROS_PT_INTEGER)
    return ToF64(p->val->integer_value);
  return p->val->double_value;
}

Bool RosParamBool(CRosNode *n,U8 *name)
{
  CRosParam *p=RosParamFind(n,name);
  if (!p)
    return FALSE;
  return p->val->bool_value!=0;
}

//The string value (owned by the parameter, don't Free). "" if unset.
U8 *RosParamStr(CRosNode *n,U8 *name)
{
  CRosParam *p=RosParamFind(n,name);
  if (!p || !p->val->string_value)
    return "";
  return p->val->string_value;
}

//Value as text (yours to Free), NULL if not declared.
U8 *RosParamText(CRosNode *n,U8 *name)
{
  CRosParam *p=RosParamFind(n,name);
  if (!p)
    return NULL;
  return RosPvToText(p->val);
}

//---- the parameter services

U8 *RosSeqStr(CRosSeq *q,I64 i)
{
  U8 **s=q->data;
  return s[i];
}

//Add one element to sequence field fname of message m (type t).
U8 *RosSeqAppend(CRosType *t,U8 *m,U8 *fname)
{
  CRosField *f=RosTypeField(t,fname);
  I64 n=RosFieldLen(f,m);
  RosSeqResize(f,m,n+1);
  return RosFieldElem(f,m,n);
}

U0 RosParamSrvGet(CSrv_rcl_interfaces_GetParameters_Request *req,
	CSrv_rcl_interfaces_GetParameters_Response *resp,CRosNode *n)
{
  I64 i;
  CRosParam *p;
  U8 *e;
  for (i=0;i<req->names.len;i++) {
    e=RosSeqAppend(ros_t_rcl_interfaces_GetParameters_Response,resp,"values");
    p=RosParamFind(n,RosSeqStr(&req->names,i));
    if (p)
      RosMsgCopy(ros_t_rcl_interfaces_ParameterValue,e,p->val);
  }
}

U0 RosParamSrvTypes(CSrv_rcl_interfaces_GetParameterTypes_Request *req,
	CSrv_rcl_interfaces_GetParameterTypes_Response *resp,CRosNode *n)
{
  I64 i;
  U8 *e;
  for (i=0;i<req->names.len;i++) {
    e=RosSeqAppend(ros_t_rcl_interfaces_GetParameterTypes_Response,resp,"types");
    *e=RosParamType(n,RosSeqStr(&req->names,i));
  }
}

U0 RosParamSrvResult(CMsg_rcl_interfaces_SetParametersResult *r,Bool ok)
{
  r->successful=ok;
  if (ok)
    r->reason=StrNew("");
  else
    r->reason=StrNew(ros_param_error);
}

U0 RosParamSrvSet(CSrv_rcl_interfaces_SetParameters_Request *req,
	CSrv_rcl_interfaces_SetParameters_Response *resp,CRosNode *n)
{
  I64 i;
  CMsg_rcl_interfaces_Parameter *ps=req->parameters.data;
  for (i=0;i<req->parameters.len;i++)
    RosParamSrvResult(RosSeqAppend(ros_t_rcl_interfaces_SetParameters_Response,resp,"results"),
	  RosParamSetPv(n,ps[i].name,&ps[i].value));
}

U0 RosParamSrvSetAtomic(CSrv_rcl_interfaces_SetParametersAtomically_Request *req,
	CSrv_rcl_interfaces_SetParametersAtomically_Response *resp,CRosNode *n)
{
  I64 i;
  Bool ok=TRUE;
  CMsg_rcl_interfaces_Parameter *ps=req->parameters.data;
  for (i=0;ok && i<req->parameters.len;i++)
    ok=RosParamCheck(n,ps[i].name,&ps[i].value);
  if (ok)
    for (i=0;i<req->parameters.len;i++)
      RosParamApply(n,ps[i].name,&ps[i].value);
  RosParamSrvResult(&resp->result,ok);
}

//Number of "." in s.
I64 RosDots(U8 *s)
{
  I64 n=0;
  while (*s) {
    if (*s=='.')
      n++;
    s++;
  }
  return n;
}

U0 RosParamSrvList(CSrv_rcl_interfaces_ListParameters_Request *req,
	CSrv_rcl_interfaces_ListParameters_Response *resp,CRosNode *n)
{
  I64 i,j,k,cut;
  U8 *name,*pre,*pfx;
  Bool want,have;
  CRosType *t=ros_t_rcl_interfaces_ListParametersResult;
  for (i=0;i<n->param_order->len;i++) {
    name=RosVecGetI64(n->param_order,i);
    want=!req->prefixes.len;
    for (j=0;j<req->prefixes.len;j++) {
      pre=RosSeqStr(&req->prefixes,j);
      if (!StrCmp(name,pre) ||
	  (RosStrStartsWith(name,pre) && name[StrLen(pre)]=='.'))
        want=TRUE;
    }
    //depth 0 = recursive; else at most depth-1 dots (below the prefix)
    if (want && req->depth && RosDots(name)>=req->depth && !req->prefixes.len)
      want=FALSE;
    if (!want)
      continue;
    RosElemSetS(RosSeqAppend(t,&resp->result,"names"),StrNew(name));
    cut=StrLen(name);
    while (cut>0 && name[cut-1]!='.')
      cut--;
    if (cut>1) {
      pfx=RosStrSub(name,0,cut-1);
      have=FALSE;
      for (k=0;k<resp->result.prefixes.len;k++)
        if (!StrCmp(RosSeqStr(&resp->result.prefixes,k),pfx))
          have=TRUE;
      if (have)
        Free(pfx);
      else
        RosElemSetS(RosSeqAppend(t,&resp->result,"prefixes"),pfx);
    }
  }
}

U0 RosParamSrvDescribe(CSrv_rcl_interfaces_DescribeParameters_Request *req,
	CSrv_rcl_interfaces_DescribeParameters_Response *resp,CRosNode *n)
{
  I64 i;
  CRosParam *p;
  CMsg_rcl_interfaces_ParameterDescriptor *d;
  for (i=0;i<req->names.len;i++) {
    d=RosSeqAppend(ros_t_rcl_interfaces_DescribeParameters_Response,resp,"descriptors");
    d->name=StrNew(RosSeqStr(&req->names,i));
    p=RosParamFind(n,d->name);
    if (p) {
      d->type=p->val->type;
      d->read_only=p->read_only;
      d->dynamic_typing=p->dynamic_typing;
      if (p->desc)
        d->description=StrNew(p->desc);
    }
  }
}

U0 RosParamNodeInit(CRosNode *n)
{
  CRosQoS q;
  CRosOverride *o;
  I64 i;
  if (ros_param_overrides)
    for (i=0;i<ros_param_overrides->len;i++) {
      o=RosVecGetI64(ros_param_overrides,i);
      if (RosOverrideMatches(n,o->node_key))
        RosNodeParamOverride(n,o->name,o->val);
    }
  if (!ros_node_builtins)
    return;
  RosQoSParameters(&q);
  n->param_events_pub=RosPubNew(n,"/parameter_events",ros_t_rcl_interfaces_ParameterEvent,&q);
  RosServiceNew(n,"~/get_parameters",ros_srv_rcl_interfaces_GetParameters,&RosParamSrvGet,n);
  RosServiceNew(n,"~/get_parameter_types",ros_srv_rcl_interfaces_GetParameterTypes,
	&RosParamSrvTypes,n);
  RosServiceNew(n,"~/set_parameters",ros_srv_rcl_interfaces_SetParameters,&RosParamSrvSet,n);
  RosServiceNew(n,"~/set_parameters_atomically",ros_srv_rcl_interfaces_SetParametersAtomically,
	&RosParamSrvSetAtomic,n);
  RosServiceNew(n,"~/list_parameters",ros_srv_rcl_interfaces_ListParameters,&RosParamSrvList,n);
  RosServiceNew(n,"~/describe_parameters",ros_srv_rcl_interfaces_DescribeParameters,
	&RosParamSrvDescribe,n);
}

U0 RosParamNodeFini(CRosNode *n)
{
  I64 it=0;
  U8 *k,*v;
  while (RosMapNext(n->params,&it,&k,&v))
    RosParamFree(v);
  it=0;
  while (RosMapNext(n->overrides,&it,&k,&v))
    Free(v);
}
