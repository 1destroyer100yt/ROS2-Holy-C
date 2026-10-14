//Yaml: just enough YAML for ROS 2 parameter files.
//
//  talker:
//    ros__parameters:
//      rate: 2.0
//      names: [a, b]
//      limits:
//        - 1
//        - 2
//
//flattens to ("talker.ros__parameters.rate","2.0"),
//("talker.ros__parameters.names","[a, b]"),
//("talker.ros__parameters.limits","[1, 2]").
//Maps, scalars, flow lists and block lists of scalars. Values stay
//as text (quotes kept) for the caller to parse.

class CRosYamlItem
{
  U8 *path;
  U8 *val;
};

U8 ros_yaml_error[128];

U0 RosYamlErr(U8 *what,I64 line)
{
  CRosStrBuf *sb=RosStrBufNew();
  RosStrBufAdd(sb,what);
  RosStrBufAdd(sb," (line ");
  RosStrBufAddI64(sb,line);
  RosStrBufAddC(sb,')');
  if (sb->len>120)
    sb->buf[120]=0;
  StrCpy(ros_yaml_error,sb->buf);
  RosStrBufDel(sb);
}

U0 RosYamlAdd(CRosVec *out,U8 *path,U8 *val)
{
  CRosYamlItem *it=CAlloc(sizeof(CRosYamlItem));
  it->path=StrNew(path);
  it->val=val;		//takes ownership
  RosVecPushI64(out,it);
}

U0 RosYamlFree(CRosVec *v)
{
  I64 i;
  CRosYamlItem *it;
  if (!v)
    return;
  for (i=0;i<v->len;i++) {
    it=RosVecGetI64(v,i);
    Free(it->path);
    Free(it->val);
    Free(it);
  }
  RosVecDel(v);
}

//Cut a comment: # at the start or after a space, not in quotes.
U0 RosYamlCutComment(U8 *s)
{
  U8 q=0;
  I64 i=0;
  while (s[i]) {
    if (q) {
      if (s[i]==q)
        q=0;
    } else if (s[i]=='\'' || s[i]=='"')
      q=s[i];
    else if (s[i]=='#' && (i==0 || s[i-1]==' ')) {
      s[i]=0;
      return;
    }
    i++;
  }
}

//Index of the "key: value" colon (followed by space or end), -1 if none.
I64 RosYamlColon(U8 *s)
{
  U8 q=0;
  I64 i=0;
  while (s[i]) {
    if (q) {
      if (s[i]==q)
        q=0;
    } else if (s[i]=='\'' || s[i]=='"')
      q=s[i];
    else if (s[i]==':' && (s[i+1]==' ' || !s[i+1]))
      return i;
    i++;
  }
  return -1;
}

//Turn collected block-list items into one "[a, b]" value.
U0 RosYamlFlushList(CRosVec *out,U8 **list_path,CRosStrBuf **list)
{
  if (!*list_path)
    return;
  RosStrBufAddC(*list,']');
  RosYamlAdd(out,*list_path,RosStrBufTake(*list));
  Free(*list_path);
  *list_path=NULL;
  *list=NULL;
}

//Flatten YAML text. Returns a vec of CRosYamlItem * (free with
//RosYamlFree), or NULL with ros_yaml_error set.
CRosVec *RosYamlFlatten(U8 *text)
{
  CRosVec *out=RosVecNew(8),*lines=RosStrSplit(text,'\n');
  CRosVec *stk_ind=RosVecNew(8),*stk_path=RosVecNew(8);
  CRosStrBuf *sb,*list=NULL;
  U8 *line,*s,*key,*val,*path,*list_path=NULL,*parent=NULL;
  I64 i,ind,c,parent_ind=-1;
  Bool ok=TRUE;
  for (i=0;ok && i<lines->len;i++) {
    line=StrNew(RosVecGetI64(lines,i));
    RosYamlCutComment(line);
    ind=0;
    while (line[ind]==' ')
      ind++;
    if (line[ind]=='\t') {
      RosYamlErr("tabs are not allowed for indentation",i+1);
      ok=FALSE;
    }
    s=RosStrTrim(line);
    Free(line);
    if (!ok || !StrLen(s) || !StrCmp(s,"---") || !StrCmp(s,"...")) {
      Free(s);
      goto next_yaml_1;
    }
    if (s[0]=='-' && (s[1]==' ' || !s[1])) {	//block list item
      if (!list_path) {
        if (!parent || ind<parent_ind) {
          RosYamlErr("list item without a key",i+1);
          ok=FALSE;
        } else {
          list_path=StrNew(parent);
          list=RosStrBufNew();
          RosStrBufAddC(list,'[');
        }
      }
      if (ok) {
        val=RosStrTrim(s+1);
        if (RosYamlColon(val)>=0) {
          RosYamlErr("lists of maps are not supported",i+1);
          ok=FALSE;
        } else {
          if (list->len>1)
            RosStrBufAdd(list,", ");
          RosStrBufAdd(list,val);
        }
        Free(val);
      }
      Free(s);
      goto next_yaml_1;
    }
    RosYamlFlushList(out,&list_path,&list);
    c=RosYamlColon(s);
    if (c<0) {
      RosYamlErr("expected key: value",i+1);
      ok=FALSE;
      Free(s);
      goto next_yaml_1;
    }
    key=RosStrSub(s,0,c);
    val=RosStrTrim(s+c+1);
    Free(s);
    s=RosStrTrim(key);
    Free(key);
    key=RosStrUnquote(s);
    Free(s);
    while (stk_ind->len && RosVecGetI64(stk_ind,stk_ind->len-1)>=ind) {
      stk_ind->len--;
      path=RosVecGetI64(stk_path,stk_path->len-1);
      if (path==parent)
        parent=NULL;
      Free(path);
      stk_path->len--;
    }
    sb=RosStrBufNew();
    if (stk_path->len) {
      RosStrBufAdd(sb,RosVecGetI64(stk_path,stk_path->len-1));
      RosStrBufAddC(sb,'.');
    }
    RosStrBufAdd(sb,key);
    path=RosStrBufTake(sb);
    Free(key);
    if (StrLen(val)) {
      RosYamlAdd(out,path,val);
      Free(path);
    } else {
      Free(val);
      RosVecPushI64(stk_ind,ind);
      RosVecPushI64(stk_path,path);
      parent=path;
      parent_ind=ind;
    }
    next_yaml_1:;
  }
  if (ok)
    RosYamlFlushList(out,&list_path,&list);
  else if (list_path) {
    Free(list_path);
    RosStrBufDel(list);
  }
  RosStrVecDel(stk_path);
  RosVecDel(stk_ind);
  RosStrVecDel(lines);
  if (!ok) {
    RosYamlFree(out);
    return NULL;
  }
  return out;
}

//Skip a quoted string or a balanced [...] / {...} starting at s[*i].
U0 RosYamlSkipValue(U8 *s,I64 *i)
{
  I64 depth=0;
  U8 q=0;
  while (s[*i]) {
    if (q) {
      if (s[*i]==q)
        q=0;
    } else if (s[*i]=='\'' || s[*i]=='"')
      q=s[*i];
    else if (s[*i]=='[' || s[*i]=='{')
      depth++;
    else if (s[*i]==']' || s[*i]=='}') {
      if (!depth)
        return;
      depth--;
      if (!depth) {
        *i=*i+1;
        return;
      }
    } else if (s[*i]==',' && !depth)
      return;
    *i=*i+1;
  }
}

//Skip one space-separated token: 'quoted', [list], {map} or plain.
U0 RosYamlSkipToken(U8 *s,I64 *i)
{
  U8 q;
  I64 depth=0;
  if (s[*i]=='\'' || s[*i]=='"') {
    q=s[*i];
    *i=*i+1;
    while (s[*i] && s[*i]!=q)
      *i=*i+1;
    if (s[*i])
      *i=*i+1;
  } else if (s[*i]=='[' || s[*i]=='{') {
    q=0;
    while (s[*i]) {
      if (q) {
        if (s[*i]==q)
          q=0;
      } else if (s[*i]=='\'' || s[*i]=='"')
        q=s[*i];
      else if (s[*i]=='[' || s[*i]=='{')
        depth++;
      else if (s[*i]==']' || s[*i]=='}') {
        depth--;
        if (!depth) {
          *i=*i+1;
          break;
        }
      }
      *i=*i+1;
    }
  }
  while (s[*i] && !RosIsSpace(s[*i]))
    *i=*i+1;
}

Bool RosYamlFlowMap(U8 *s,I64 *i,U8 *prefix,CRosVec *out)
{
  I64 k0,v0;
  U8 *key,*val,*path;
  CRosStrBuf *sb;
  while (RosIsSpace(s[*i]))
    *i=*i+1;
  if (s[*i]!='{')
    return FALSE;
  *i=*i+1;
  while (TRUE) {
    while (RosIsSpace(s[*i]) || s[*i]==',')
      *i=*i+1;
    if (s[*i]=='}') {
      *i=*i+1;
      return TRUE;
    }
    if (!s[*i])
      return FALSE;
    k0=*i;
    while (s[*i] && s[*i]!=':' && s[*i]!='}')
      *i=*i+1;
    if (s[*i]!=':')
      return FALSE;
    val=RosStrSub(s,k0,*i-k0);
    key=RosStrTrim(val);
    Free(val);
    *i=*i+1;
    sb=RosStrBufNew();
    if (prefix) {
      RosStrBufAdd(sb,prefix);
      RosStrBufAddC(sb,'.');
    }
    RosStrBufAdd(sb,key);
    path=RosStrBufTake(sb);
    Free(key);
    while (RosIsSpace(s[*i]))
      *i=*i+1;
    if (s[*i]=='{') {
      if (!RosYamlFlowMap(s,i,path,out)) {
        Free(path);
        return FALSE;
      }
    } else {
      v0=*i;
      RosYamlSkipValue(s,i);
      val=RosStrSub(s,v0,*i-v0);
      RosYamlAdd(out,path,RosStrTrim(val));
      Free(val);
    }
    Free(path);
  }
  return FALSE;
}

//"{linear: {x: 0.5}, name: 'a, b', v: [1, 2]}" -> items
//("linear.x","0.5"), ("name","'a, b'"), ("v","[1, 2]"). Also takes
//the simpler "linear.x=0.5 name=bob" form. NULL if it can't parse.
CRosVec *RosYamlFlowFlatten(U8 *text)
{
  CRosVec *out=RosVecNew(8);
  U8 *s=RosStrTrim(text),*path,*val;
  I64 i=0,a,eq;
  Bool ok=TRUE;
  if (!s[0]) {
  } else if (s[0]=='{') {
    ok=RosYamlFlowMap(s,&i,NULL,out);
    while (ok && RosIsSpace(s[i]))
      i++;
    if (s[i])
      ok=FALSE;
  } else {
    while (ok && s[i]) {
      while (RosIsSpace(s[i]))
        i++;
      if (!s[i])
        break;
      a=i;
      while (s[i] && s[i]!='=' && !RosIsSpace(s[i]))
        i++;
      if (s[i]!='=') {
        ok=FALSE;
        break;
      }
      eq=i;
      i++;
      RosYamlSkipToken(s,&i);
      path=RosStrSub(s,a,eq-a);
      val=RosStrSub(s,eq+1,i-eq-1);
      RosYamlAdd(out,path,val);
      Free(path);
    }
  }
  Free(s);
  if (!ok) {
    StrCpy(ros_yaml_error,"can't read values");
    RosYamlFree(out);
    return NULL;
  }
  return out;
}
