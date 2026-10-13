//Name: ROS 2 names. Validation, expansion and remapping, following
//the rcl rules:
//  "chatter"   relative  -> "<ns>/chatter"
//  "/chatter"  absolute  -> "/chatter"
//  "~/state"   private   -> "<ns>/<node>/state"
//Tokens are [A-Za-z0-9_], can't start with a digit, no empty tokens.

U8 ros_name_error[128];

U0 RosNameErr(U8 *what,U8 *name)
{
  CRosStrBuf *sb=RosStrBufNew();
  RosStrBufAdd(sb,what);
  RosStrBufAdd(sb,": '");
  RosStrBufAdd(sb,name);
  RosStrBufAddC(sb,'\'');
  if (sb->len>120)
    sb->buf[120]=0;
  StrCpy(ros_name_error,sb->buf);
  RosStrBufDel(sb);
}

Bool RosNameCharOk(U8 c)
{
  return (c>='a' && c<='z') || (c>='A' && c<='Z') || (c>='0' && c<='9') || c=='_';
}

//A node name: one token.
Bool RosNodeNameOk(U8 *s)
{
  I64 i=0;
  if (!s[0] || (s[0]>='0' && s[0]<='9')) {
    RosNameErr("bad node name",s);
    return FALSE;
  }
  while (s[i]) {
    if (!RosNameCharOk(s[i])) {
      RosNameErr("bad node name",s);
      return FALSE;
    }
    i++;
  }
  return TRUE;
}

//A fully qualified name: "/" or "/tok/tok".
Bool RosFqnOk(U8 *s)
{
  I64 i=1;
  if (s[0]!='/') {
    RosNameErr("name must start with /",s);
    return FALSE;
  }
  if (!s[1])
    return TRUE;
  while (TRUE) {
    if (!s[i] || s[i]=='/' || (s[i]>='0' && s[i]<='9')) {
      RosNameErr("bad token in name",s);
      return FALSE;
    }
    while (s[i] && s[i]!='/') {
      if (!RosNameCharOk(s[i])) {
        RosNameErr("bad character in name",s);
        return FALSE;
      }
      i++;
    }
    if (!s[i])
      return TRUE;
    i++;	//skip '/'
  }
  return TRUE;
}

//"/ns" + "node" -> "/ns/node" (new string)
U8 *RosNameJoin(U8 *ns,U8 *name)
{
  CRosStrBuf *sb=RosStrBufNew();
  RosStrBufAdd(sb,ns);
  if (StrCmp(ns,"/"))
    RosStrBufAddC(sb,'/');
  RosStrBufAdd(sb,name);
  return RosStrBufTake(sb);
}

//Expand a topic/service name for a node in namespace ns. Returns a
//new fully qualified name, or NULL (ros_name_error) if invalid.
U8 *RosNameExpand(U8 *name,U8 *node_name,U8 *ns)
{
  U8 *fqn,*priv;
  CRosStrBuf *sb;
  if (!name || !name[0]) {
    RosNameErr("empty name",name);
    return NULL;
  }
  if (name[0]=='/')
    fqn=StrNew(name);
  else if (name[0]=='~') {
    if (name[1] && name[1]!='/') {
      RosNameErr("~ must be followed by /",name);
      return NULL;
    }
    priv=RosNameJoin(ns,node_name);
    sb=RosStrBufNew();
    RosStrBufAdd(sb,priv);
    RosStrBufAdd(sb,name+1);
    fqn=RosStrBufTake(sb);
    Free(priv);
  } else
    fqn=RosNameJoin(ns,name);
  if (StrLen(fqn)>1 && fqn[StrLen(fqn)-1]=='/') {
    RosNameErr("name can't end with /",name);
    Free(fqn);
    return NULL;
  }
  if (!RosFqnOk(fqn)) {
    Free(fqn);
    return NULL;
  }
  return fqn;
}

//Global remap rules (like --ros-args -r from:=to) for every node.
//Each entry is "from:=to" with names as written.
CRosVec *ros_global_remaps=NULL;

U0 RosRemapGlobal(U8 *from,U8 *to)
{
  if (!ros_global_remaps)
    ros_global_remaps=RosVecNew(8);
  RosVecPushI64(ros_global_remaps,StrNew(from));
  RosVecPushI64(ros_global_remaps,StrNew(to));
}

//Apply remap rules (vec of from,to pairs) to fqn. Rules are expanded
//for this node before comparing. Returns new string.
U8 *RosNameRemapWith(CRosVec *rules,U8 *fqn,U8 *node_name,U8 *ns)
{
  I64 i;
  U8 *from,*to;
  if (!rules)
    return StrNew(fqn);
  for (i=0;i+1<rules->len;i+=2) {
    from=RosNameExpand(RosVecGetI64(rules,i),node_name,ns);
    if (from && !StrCmp(from,fqn)) {
      Free(from);
      to=RosNameExpand(RosVecGetI64(rules,i+1),node_name,ns);
      if (to)
        return to;
      return StrNew(fqn);
    }
    Free(from);
  }
  return StrNew(fqn);
}
