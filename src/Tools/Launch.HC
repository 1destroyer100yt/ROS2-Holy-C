//Launch: start several nodes with names, namespaces, remaps and
//parameters, like a ROS 2 launch file. A "launch file" is just a
//HolyC file that builds a CRosLaunch and starts it:
//
//  CRosLaunch *l=RosLaunchNew();
//  CRosLaunchNode *t=RosLaunchAdd(l,"talker","/demo",&TalkerSetup,NULL);
//  RosLaunchRemap(t,"chatter","news");
//  RosLaunchParam(t,"rate","2.0");
//  RosLaunchStart(l);          //TempleOS: one task per node
//  ...
//  RosLaunchStop(l);
//
//setup(node,arg) creates the node's publishers/timers/etc. It runs in
//the node's own task, which then spins until RosLaunchStop.

class CRosLaunchNode
{
  U8 *name,*ns;
  U0 (*setup)(CRosNode *n,U8 *arg);
  U8 *arg;
  CRosVec *remaps;	//from,to
  CRosVec *params;	//name,value
  CRosNode *node;
  Bool done;
};

class CRosLaunch
{
  CRosVec *nodes;	//CRosLaunchNode *
  CRosExec *exec;	//when started in this task
};

CRosLaunch *RosLaunchNew()
{
  CRosLaunch *l=CAlloc(sizeof(CRosLaunch));
  l->nodes=RosVecNew(8);
  return l;
}

CRosLaunchNode *RosLaunchAdd(CRosLaunch *l,U8 *name,U8 *ns,
	U0 (*setup)(CRosNode *n,U8 *arg),U8 *arg)
{
  CRosLaunchNode *ln=CAlloc(sizeof(CRosLaunchNode));
  ln->name=StrNew(name);
  if (ns)
    ln->ns=StrNew(ns);
  ln->setup=setup;
  ln->arg=arg;
  ln->remaps=RosVecNew(8);
  ln->params=RosVecNew(8);
  RosVecPushI64(l->nodes,ln);
  return ln;
}

U0 RosLaunchRemap(CRosLaunchNode *ln,U8 *from,U8 *to)
{
  RosVecPushI64(ln->remaps,StrNew(from));
  RosVecPushI64(ln->remaps,StrNew(to));
}

U0 RosLaunchParam(CRosLaunchNode *ln,U8 *name,U8 *value)
{
  RosVecPushI64(ln->params,StrNew(name));
  RosVecPushI64(ln->params,StrNew(value));
}

//Make the node and run its setup (in the calling task).
CRosNode *RosLaunchMake(CRosLaunchNode *ln)
{
  CRosNode *n=RosNodeNew(ln->name,ln->ns);
  I64 i;
  if (!n) {
    Print("launch: can't make node %s: %s\n",ln->name,ros_graph_error);
    return NULL;
  }
  for (i=0;i+1<ln->remaps->len;i+=2)
    RosNodeRemap(n,RosVecGetI64(ln->remaps,i),RosVecGetI64(ln->remaps,i+1));
  for (i=0;i+1<ln->params->len;i+=2)
    RosNodeParamOverride(n,RosVecGetI64(ln->params,i),RosVecGetI64(ln->params,i+1));
  if (ln->setup)
    (*ln->setup)(n,ln->arg);
  ln->node=n;
  return n;
}

U0 RosLaunchTask(CRosLaunchNode *ln)
{
  CRosNode *n=RosLaunchMake(ln);
  if (n) {
    RosSpin(n);
    RosExecDel(n->exec);
    RosNodeDel(n);
  }
  ln->node=NULL;
  ln->done=TRUE;
}

//Start every node in its own task (TempleOS).
U0 RosLaunchStart(CRosLaunch *l)
{
  I64 i;
  CRosLaunchNode *ln;
  ros_ok=TRUE;
  for (i=0;i<l->nodes->len;i++) {
    ln=RosVecGetI64(l->nodes,i);
    ln->done=FALSE;
//@host-skip-begin
    Spawn(&RosLaunchTask,ln,ln->name);
//@host-skip-end
  }
}

//Start every node in the calling task, sharing one executor (for
//the host, or if you'd rather keep it all in one task). Spin it with
//RosExecSpin(l->exec) or RosLaunchSpinFor.
U0 RosLaunchStartHere(CRosLaunch *l)
{
  I64 i;
  CRosNode *n;
  l->exec=RosExecNew();
  for (i=0;i<l->nodes->len;i++) {
    n=RosLaunchMake(RosVecGetI64(l->nodes,i));
    if (n)
      RosExecAdd(l->exec,n);
  }
}

U0 RosLaunchSpinFor(CRosLaunch *l,I64 ns)
{
  if (l->exec)
    RosExecSpinFor(l->exec,ns);
}

//Find a running node by its name in the launch.
CRosNode *RosLaunchNode(CRosLaunch *l,U8 *name)
{
  I64 i;
  CRosLaunchNode *ln;
  for (i=0;i<l->nodes->len;i++) {
    ln=RosVecGetI64(l->nodes,i);
    if (!StrCmp(ln->name,name))
      return ln->node;
  }
  return NULL;
}

//Stop everything and free the launch description.
U0 RosLaunchStop(CRosLaunch *l)
{
  I64 i;
  CRosLaunchNode *ln;
  if (l->exec) {	//started here
    for (i=0;i<l->nodes->len;i++) {
      ln=RosVecGetI64(l->nodes,i);
      RosNodeDel(ln->node);
      ln->node=NULL;
    }
    RosExecDel(l->exec);
  } else {		//tasks: ask them to stop and wait
    RosShutdown();
    for (i=0;i<l->nodes->len;i++) {
      ln=RosVecGetI64(l->nodes,i);
      while (!ln->done)
        Yield();
    }
    ros_ok=TRUE;
  }
  for (i=0;i<l->nodes->len;i++) {
    ln=RosVecGetI64(l->nodes,i);
    Free(ln->name);
    Free(ln->ns);
    RosStrVecDel(ln->remaps);
    RosStrVecDel(ln->params);
    Free(ln);
  }
  RosVecDel(l->nodes);
  Free(l);
}

//@host-skip-begin
//Run a launch file (a HolyC file that sets up and starts a launch).
U0 RosLaunchFile(U8 *path)
{
  ExeFile(path);
}
//@host-skip-end
