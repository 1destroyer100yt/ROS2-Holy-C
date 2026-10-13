//Gen: turn interfaces/ (.msg/.srv files listed in interfaces/Index.txt)
//into HolyC: one interfaces/<pkg>/Types.HC per package with a class per
//message, constants, and a loader; plus interfaces/Load.HC and
//tests/TestLayout.HC (checks the classes match the CRosType layout).
//
//In TempleOS:  Cd("~/ROS2-Holy-C"); RosIfaceGen(".");
//On Linux:     tools/gen_interfaces.sh

//"geometry_msgs/msg/Twist" -> "geometry_msgs_Twist" (srv: "pkg_Name_Request")
U0 RosGenIdent(CRosStrBuf *sb,CRosType *t)
{
  RosStrBufAdd(sb,t->pkg);
  RosStrBufAddC(sb,'_');
  RosStrBufAdd(sb,t->base);
}

//Class name: CMsg_pkg_Name, CSrv_pkg_Name_Request for services,
//CAct_pkg_Name_Goal (and _SendGoal_Request...) for actions.
U0 RosGenClassName(CRosStrBuf *sb,CRosType *t)
{
  if (RosStrFind(t->name,"/srv/")>=0)
    RosStrBufAdd(sb,"CSrv_");
  else if (RosStrFind(t->name,"/action/")>=0)
    RosStrBufAdd(sb,"CAct_");
  else
    RosStrBufAdd(sb,"CMsg_");
  RosGenIdent(sb,t);
}

//Add text as the inside of a HolyC string literal.
U0 RosGenStrLit(CRosStrBuf *sb,U8 *s)
{
  while (*s) {
    if (*s=='\n')
      RosStrBufAdd(sb,"\\n");
    else if (*s=='\t')
      RosStrBufAdd(sb,"\\t");
    else if (*s=='\\')
      RosStrBufAdd(sb,"\\\\");
    else if (*s=='"')
      RosStrBufAdd(sb,"\\\"");
    else if (*s=='$$')
      RosStrBufAdd(sb,"$$$$");	//DolDoc: two in a file mean one
    else if (*s!='\r')
      RosStrBufAddC(sb,*s);
    s++;
  }
}

U0 RosGenClass(CRosStrBuf *sb,CRosType *t)
{
  I64 i,pad;
  CRosField *f;
  CRosConst *c;
  RosStrBufAdd(sb,"class ");
  RosGenClassName(sb,t);
  RosStrBufAdd(sb,"\n{\n");
  for (i=0;i<t->fields->len;i++) {
    f=RosVecGetI64(t->fields,i);
    RosStrBufAdd(sb,"  ");
    if (f->arr==ROS_ARR_SEQ)
      RosStrBufAdd(sb,"CRosSeq ");
    else if (f->kind==ROS_T_MSG) {
      RosGenClassName(sb,f->sub);
      RosStrBufAddC(sb,' ');
    } else if (f->kind==ROS_T_STRING)
      RosStrBufAdd(sb,"U8 *");
    else if (RosKindIsFloat(f->kind))
      RosStrBufAdd(sb,"F64 ");
    else if (f->stride==1)
      RosStrBufAdd(sb,"U8 ");
    else if (f->kind==ROS_T_UINT64)
      RosStrBufAdd(sb,"U64 ");
    else
      RosStrBufAdd(sb,"I64 ");
    RosStrBufAdd(sb,f->name);
    if (f->arr==ROS_ARR_FIXED) {
      RosStrBufAddC(sb,'[');
      RosStrBufAddI64(sb,f->arr_len);
      RosStrBufAddC(sb,']');
    }
    RosStrBufAdd(sb,";\t//");
    if (f->kind==ROS_T_MSG)
      RosStrBufAdd(sb,f->sub->name);
    else
      RosStrBufAdd(sb,RosKindName(f->kind));
    if (f->arr==ROS_ARR_FIXED)
      RosStrBufAdd(sb," fixed array");
    else if (f->arr==ROS_ARR_SEQ)
      RosStrBufAdd(sb," sequence");
    RosStrBufAddC(sb,'\n');
    if (f->arr==ROS_ARR_FIXED && f->stride==1) {
      pad=f->size-f->arr_len;
      if (pad) {
        RosStrBufAdd(sb,"  U8 _pad_");
        RosStrBufAdd(sb,f->name);
        RosStrBufAddC(sb,'[');
        RosStrBufAddI64(sb,pad);
        RosStrBufAdd(sb,"];\n");
      }
    }
  }
  RosStrBufAdd(sb,"};\n");
  for (i=0;i<t->consts->len;i++) {
    c=RosVecGetI64(t->consts,i);
    RosStrBufAdd(sb,"#define ROS_");
    RosGenIdent(sb,t);
    RosStrBufAddC(sb,'_');
    RosStrBufAdd(sb,c->name);
    RosStrBufAdd(sb,"\t(");
    if (c->kind==ROS_T_STRING) {
      RosStrBufAddC(sb,'"');
      RosGenStrLit(sb,c->s);
      RosStrBufAddC(sb,'"');
    } else if (RosKindIsFloat(c->kind))
      RosStrBufAddF64(sb,c->f);
    else
      RosStrBufAddI64(sb,c->i);
    RosStrBufAdd(sb,")\n");
  }
  RosStrBufAdd(sb,"CRosType *ros_t_");
  RosGenIdent(sb,t);
  RosStrBufAdd(sb,"=NULL;\n\n");
}

//Layout checks for one type, appended to tests/TestLayout.HC.
U0 RosGenCheck(CRosStrBuf *sb,CRosType *t)
{
  I64 i;
  CRosField *f;
  RosStrBufAdd(sb,"  t=ros_t_");
  RosGenIdent(sb,t);
  RosStrBufAdd(sb,";\n  RosTestI64(\"");
  RosStrBufAdd(sb,t->name);
  RosStrBufAdd(sb," size\",sizeof(");
  RosGenClassName(sb,t);
  RosStrBufAdd(sb,"),t->size);\n");
  for (i=0;i<t->fields->len;i++) {
    f=RosVecGetI64(t->fields,i);
    RosStrBufAdd(sb,"  RosTestI64(\"");
    RosStrBufAdd(sb,t->name);
    RosStrBufAddC(sb,'.');
    RosStrBufAdd(sb,f->name);
    RosStrBufAdd(sb,"\",offset(");
    RosGenClassName(sb,t);
    RosStrBufAddC(sb,'.');
    RosStrBufAdd(sb,f->name);
    RosStrBufAdd(sb,"),RosTypeField(t,\"");
    RosStrBufAdd(sb,f->name);
    RosStrBufAdd(sb,"\")->offset);\n");
  }
}

class CRosGenPkg
{
  U8 *name;
  CRosVec *items;	//U8 * "pkg/msg/Name" / "pkg/srv/Name", in load order
  CRosVec *deps;	//U8 * package names this one needs
  Bool done;
};

CRosGenPkg *RosGenPkgGet(CRosVec *pkgs,U8 *name)
{
  I64 i;
  CRosGenPkg *p;
  for (i=0;i<pkgs->len;i++) {
    p=RosVecGetI64(pkgs,i);
    if (!StrCmp(p->name,name))
      return p;
  }
  p=CAlloc(sizeof(CRosGenPkg));
  p->name=StrNew(name);
  p->items=RosVecNew(8);
  p->deps=RosVecNew(8);
  RosVecPushI64(pkgs,p);
  return p;
}

U0 RosGenAddDeps(CRosGenPkg *p,CRosType *t)
{
  I64 i,j;
  CRosField *f;
  Bool have;
  for (i=0;i<t->fields->len;i++) {
    f=RosVecGetI64(t->fields,i);
    if (f->kind!=ROS_T_MSG || !StrCmp(f->sub->pkg,p->name))
      continue;
    have=FALSE;
    for (j=0;j<p->deps->len;j++)
      if (!StrCmp(RosVecGetI64(p->deps,j),f->sub->pkg))
        have=TRUE;
    if (!have)
      RosVecPushI64(p->deps,StrNew(f->sub->pkg));
  }
}

U8 *RosGenPath(U8 *root,U8 *a,U8 *b,U8 *c)
{
  CRosStrBuf *sb=RosStrBufNew();
  RosStrBufAdd(sb,root);
  RosStrBufAddC(sb,'/');
  RosStrBufAdd(sb,a);
  if (b) {
    RosStrBufAddC(sb,'/');
    RosStrBufAdd(sb,b);
  }
  if (c)
    RosStrBufAdd(sb,c);
  return RosStrBufTake(sb);
}

//The 7 message types of an action, in the order they're generated.
CRosVec *RosGenActTypes(CRosActionType *at)
{
  CRosVec *v=RosVecNew(8);
  RosVecPushI64(v,at->goal);
  RosVecPushI64(v,at->result);
  RosVecPushI64(v,at->feedback);
  RosVecPushI64(v,at->send_goal->req);
  RosVecPushI64(v,at->send_goal->resp);
  RosVecPushI64(v,at->get_result->req);
  RosVecPushI64(v,at->get_result->resp);
  RosVecPushI64(v,at->feedback_msg);
  return v;
}

//"ros_act_pkg_Name" for an action.
U0 RosGenActVar(CRosStrBuf *sb,CRosActionType *at)
{
  RosStrBufAdd(sb,"ros_act_");
  RosStrBufAdd(sb,at->goal->pkg);
  RosStrBufAddC(sb,'_');
  RosStrBufAdd(sb,at->name+StrLen(at->goal->pkg)+8);	//skip "pkg/action/"
}

Bool RosGenWrite(U8 *path,CRosStrBuf *sb)
{
  if (!RosFileWrite(path,sb->buf,sb->len)) {
    Print("RosIfaceGen: can't write %s\n",path);
    return FALSE;
  }
  return TRUE;
}

//Generate everything under root (the repo folder). TRUE on success.
Bool RosIfaceGen(U8 *root)
{
  U8 *path,*text,*item,*rel,*kind,*nm;
  CRosVec *lines,*parts,*pkgs=RosVecNew(8),*order=RosVecNew(8);
  CRosGenPkg *p,*q;
  CRosStrBuf *sb,*chk,*ld;
  CRosType *t;
  CRosSrvType *st;
  CRosActionType *at;
  CRosVec *acts;
  I64 i,j,k,progress,ntypes=0;
  Bool ready,ok=TRUE;

  path=RosGenPath(root,"interfaces/Index.txt",NULL,NULL);
  text=RosFileRead(path,NULL);
  if (!text) {
    Print("RosIfaceGen: can't read %s\n",path);
    return FALSE;
  }
  Free(path);
  lines=RosStrSplit(text,'\n');
  Free(text);

  //1. define every interface, in Index.txt order
  for (i=0;ok && i<lines->len;i++) {
    item=RosStrTrim(RosVecGetI64(lines,i));
    if (!StrLen(item)) {
      Free(item);
      continue;
    }
    parts=RosStrSplit(item,'/');
    if (parts->len!=3) {
      Print("RosIfaceGen: bad line in Index.txt: %s\n",item);
      ok=FALSE;
    } else {
      kind=RosVecGetI64(parts,1);
      nm=RosVecGetI64(parts,2);
      rel=RosGenPath(RosVecGetI64(parts,0),kind,nm,NULL);
      if (!StrCmp(kind,"msg"))
        path=RosGenPath(root,"interfaces",rel,".msg");
      else if (!StrCmp(kind,"action"))
        path=RosGenPath(root,"interfaces",rel,".action");
      else
        path=RosGenPath(root,"interfaces",rel,".srv");
      text=RosFileRead(path,NULL);
      if (!text) {
        Print("RosIfaceGen: can't read %s\n",path);
        ok=FALSE;
      } else {
        p=RosGenPkgGet(pkgs,RosVecGetI64(parts,0));
        if (!StrCmp(kind,"msg")) {
          t=RosMsgDefine(item,text);
          if (t)
            RosGenAddDeps(p,t);
        } else if (!StrCmp(kind,"action")) {
          at=RosActionDefine(item,text);
          t=NULL;
          if (at) {
            t=at->goal;
            acts=RosGenActTypes(at);
            for (k=0;k<acts->len;k++)
              RosGenAddDeps(p,RosVecGetI64(acts,k));
            RosVecDel(acts);
          }
        } else {
          st=RosSrvDefine(item,text);
          t=NULL;
          if (st) {
            t=st->req;
            RosGenAddDeps(p,st->req);
            RosGenAddDeps(p,st->resp);
          }
        }
        if (!t) {
          Print("RosIfaceGen: %s\n",ros_msg_error);
          ok=FALSE;
        } else {
          RosVecPushI64(p->items,StrNew(item));
          ntypes++;
        }
        Free(text);
      }
      Free(path);
      Free(rel);
    }
    RosStrVecDel(parts);
    Free(item);
  }
  RosStrVecDel(lines);
  if (!ok)
    return FALSE;

  //2. order packages so each comes after the ones it uses
  progress=TRUE;
  while (progress) {
    progress=FALSE;
    for (i=0;i<pkgs->len;i++) {
      p=RosVecGetI64(pkgs,i);
      if (p->done)
        continue;
      ready=TRUE;
      for (j=0;j<p->deps->len;j++) {
        q=RosGenPkgGet(pkgs,RosVecGetI64(p->deps,j));
        if (!q->done)
          ready=FALSE;
      }
      if (ready) {
        p->done=TRUE;
        RosVecPushI64(order,p);
        progress=TRUE;
      }
    }
  }
  if (order->len!=pkgs->len) {
    Print("RosIfaceGen: packages depend on each other in a loop\n");
    return FALSE;
  }

  //3. write the files
  chk=RosStrBufNew();
  RosStrBufAdd(chk,"//Generated by RosIfaceGen (src/Msg/Gen.HC). Don't edit.\n");
  RosStrBufAdd(chk,"//Checks every generated class has the layout CRosType expects.\n\n");
  RosStrBufAdd(chk,"U0 TestLayout()\n{\n  CRosType *t;\n  RosTestGroup(\"Layout\");\n");
  ld=RosStrBufNew();
  RosStrBufAdd(ld,"//Generated by RosIfaceGen (src/Msg/Gen.HC). Don't edit.\n");
  RosStrBufAdd(ld,"//Loads every interface package. Needs src/Load.HC first.\n\n");
  RosStrBufAdd(ld,"Cd(__DIR__);;\n");
  for (i=0;i<order->len;i++) {
    p=RosVecGetI64(order,i);
    RosStrBufAdd(ld,"#include \"");
    RosStrBufAdd(ld,p->name);
    RosStrBufAdd(ld,"/Types.HC\"\n");
  }
  RosStrBufAdd(ld,"\nU0 RosIfaceLoadAll()\n{\n");
  for (i=0;i<order->len;i++) {
    p=RosVecGetI64(order,i);
    sb=RosStrBufNew();
    RosStrBufAdd(sb,"//Generated by RosIfaceGen (src/Msg/Gen.HC) from interfaces/");
    RosStrBufAdd(sb,p->name);
    RosStrBufAdd(sb,". Don't edit.\n\n");
    for (j=0;j<p->items->len;j++) {
      item=RosVecGetI64(p->items,j);
      if (RosStrFind(item,"/action/")>=0) {
        at=RosActionFind(item);
        acts=RosGenActTypes(at);
        for (k=0;k<acts->len;k++) {
          RosGenClass(sb,RosVecGetI64(acts,k));
          RosGenCheck(chk,RosVecGetI64(acts,k));
        }
        RosVecDel(acts);
        RosStrBufAdd(sb,"CRosActionType *");
        RosGenActVar(sb,at);
        RosStrBufAdd(sb,"=NULL;\n\n");
      } else if (RosStrFind(item,"/srv/")>=0) {
        st=RosSrvFind(item);
        RosGenClass(sb,st->req);
        RosGenClass(sb,st->resp);
        RosGenCheck(chk,st->req);
        RosGenCheck(chk,st->resp);
        RosStrBufAdd(sb,"CRosSrvType *ros_srv_");
        RosGenIdent(sb,st->req);
        sb->len-=8;	//drop "_Request"
        sb->buf[sb->len]=0;
        RosStrBufAdd(sb,"=NULL;\n\n");
      } else {
        t=RosTypeFind(item);
        RosGenClass(sb,t);
        RosGenCheck(chk,t);
      }
    }
    RosStrBufAdd(sb,"U0 RosIfaceLoad_");
    RosStrBufAdd(sb,p->name);
    RosStrBufAdd(sb,"()\n{\n");
    for (j=0;j<p->items->len;j++) {
      item=RosVecGetI64(p->items,j);
      if (RosStrFind(item,"/action/")>=0) {
        at=RosActionFind(item);
        RosStrBufAdd(sb,"  ");
        RosGenActVar(sb,at);
        RosStrBufAdd(sb,"=RosActionDefine(\"");
        RosStrBufAdd(sb,item);
        RosStrBufAdd(sb,"\",\"");
        rel=RosGenPath(root,"interfaces",item,".action");
        text=RosFileRead(rel,NULL);
        RosGenStrLit(sb,text);
        Free(text);
        Free(rel);
        RosStrBufAdd(sb,"\");\n");
        acts=RosGenActTypes(at);
        for (k=0;k<acts->len;k++) {
          t=RosVecGetI64(acts,k);
          RosStrBufAdd(sb,"  ros_t_");
          RosGenIdent(sb,t);
          RosStrBufAdd(sb,"=RosTypeFind(\"");
          RosStrBufAdd(sb,t->name);
          RosStrBufAdd(sb,"\");\n");
        }
        RosVecDel(acts);
      } else if (RosStrFind(item,"/srv/")>=0) {
        st=RosSrvFind(item);
        RosStrBufAdd(sb,"  ros_srv_");
        RosGenIdent(sb,st->req);
        sb->len-=8;
        sb->buf[sb->len]=0;
        RosStrBufAdd(sb,"=RosSrvDefine(\"");
        RosStrBufAdd(sb,item);
        RosStrBufAdd(sb,"\",\"");
        rel=RosGenPath(root,"interfaces",item,".srv");
        text=RosFileRead(rel,NULL);
        RosGenStrLit(sb,text);
        Free(text);
        Free(rel);
        RosStrBufAdd(sb,"\");\n  ros_t_");
        RosGenIdent(sb,st->req);
        RosStrBufAdd(sb,"=ros_srv_");
        RosGenIdent(sb,st->req);
        sb->len-=8;
        sb->buf[sb->len]=0;
        RosStrBufAdd(sb,"->req;\n  ros_t_");
        RosGenIdent(sb,st->resp);
        RosStrBufAdd(sb,"=ros_srv_");
        RosGenIdent(sb,st->req);
        sb->len-=8;
        sb->buf[sb->len]=0;
        RosStrBufAdd(sb,"->resp;\n");
      } else {
        t=RosTypeFind(item);
        RosStrBufAdd(sb,"  ros_t_");
        RosGenIdent(sb,t);
        RosStrBufAdd(sb,"=RosMsgDefine(\"");
        RosStrBufAdd(sb,item);
        RosStrBufAdd(sb,"\",\"");
        RosGenStrLit(sb,t->def);
        RosStrBufAdd(sb,"\");\n");
      }
    }
    RosStrBufAdd(sb,"}\n");
    path=RosGenPath(root,"interfaces",p->name,"/Types.HC");
    ok=ok && RosGenWrite(path,sb);
    Free(path);
    RosStrBufDel(sb);
    RosStrBufAdd(ld,"  RosIfaceLoad_");
    RosStrBufAdd(ld,p->name);
    RosStrBufAdd(ld,"();\n");
  }
  RosStrBufAdd(ld,"}\n\n//@host-skip-begin\nRosIfaceLoadAll;\n//@host-skip-end\n");
  RosStrBufAdd(chk,"}\n");
  path=RosGenPath(root,"interfaces/Load.HC",NULL,NULL);
  ok=ok && RosGenWrite(path,ld);
  Free(path);
  path=RosGenPath(root,"tests/TestLayout.HC",NULL,NULL);
  ok=ok && RosGenWrite(path,chk);
  Free(path);
  RosStrBufDel(ld);
  RosStrBufDel(chk);
  if (ok)
    Print("RosIfaceGen: %d interfaces in %d packages\n",ntypes,order->len);
  return ok;
}
