//Turtlesim: the ROS 2 turtlesim, in HolyC, drawn on the TempleOS
//screen. Same topics and services as the real one:
//  /<turtle>/cmd_vel (geometry_msgs/Twist)  in
//  /<turtle>/pose (turtlesim/Pose)          out, 62.5 Hz
//  /spawn /kill /clear /reset
//  /<turtle>/set_pen /<turtle>/teleport_absolute /<turtle>/teleport_relative
//
//In TempleOS:  TurtleSim;   (arrow keys drive turtle1, Esc quits)

#define TURTLE_WORLD	11.088889	//meters across (500 px / 45 px per m)
#define TURTLE_START	5.544445
#define TURTLE_DT_NS	16000000	//62.5 Hz like turtlesim
#define TURTLE_CMD_TIMEOUT 1000000000	//stop if no cmd_vel for 1 s

class CTurtle
{
  U8 *name;
  F64 x,y,theta,lin,ang;
  I64 last_cmd_ns;
  I64 r,g,b,width;
  Bool pen_off;
  CRosSub *cmd;
  CRosPub *pose;
  CRosService *set_pen,*tele_abs,*tele_rel;
  U8 *sim;	//CTurtleSim *
};

class CTurtleLine
{
  F64 x0,y0,x1,y1;
  I64 r,g,b,width;
};

class CTurtleSim
{
  CRosNode *node;
  CRosVec *turtles;	//CTurtle *
  CRosVec *trail;	//CTurtleLine *
  I64 spawned;
  I64 last_ns;
};

CTurtleSim *turtle_sim=NULL;	//the one being drawn

F64 TurtleNormAngle(F64 a)
{
  while (a>pi)
    a-=2.0*pi;
  while (a< -pi)
    a+=2.0*pi;
  return a;
}

CTurtle *TurtleFind(CTurtleSim *ts,U8 *name)
{
  I64 i;
  CTurtle *t;
  for (i=0;i<ts->turtles->len;i++) {
    t=RosVecGetI64(ts->turtles,i);
    if (!StrCmp(t->name,name))
      return t;
  }
  return NULL;
}

U0 TurtleOnCmd(CMsg_geometry_msgs_Twist *m,CTurtle *t)
{
  t->lin=m->linear.x;
  t->ang=m->angular.z;
  t->last_cmd_ns=RosTimeSteadyNs();
}

U0 TurtleAddLine(CTurtleSim *ts,CTurtle *t,F64 x0,F64 y0)
{
  CTurtleLine *l;
  if (t->pen_off || (x0==t->x && y0==t->y))
    return;
  l=CAlloc(sizeof(CTurtleLine));
  l->x0=x0;
  l->y0=y0;
  l->x1=t->x;
  l->y1=t->y;
  l->r=t->r;
  l->g=t->g;
  l->b=t->b;
  l->width=t->width;
  RosVecPushI64(ts->trail,l);
}

U0 TurtleSetPen(CSrv_turtlesim_SetPen_Request *req,U8 *resp,CTurtle *t)
{
  t->r=req->r;
  t->g=req->g;
  t->b=req->b;
  t->width=req->width;
  t->pen_off=req->off!=0;
}

U0 TurtleTeleAbs(CSrv_turtlesim_TeleportAbsolute_Request *req,U8 *resp,CTurtle *t)
{
  F64 x0=t->x,y0=t->y;
  t->x=req->x;
  t->y=req->y;
  t->theta=TurtleNormAngle(req->theta);
  TurtleAddLine(t->sim,t,x0,y0);
}

U0 TurtleTeleRel(CSrv_turtlesim_TeleportRelative_Request *req,U8 *resp,CTurtle *t)
{
  F64 x0=t->x,y0=t->y;
  t->theta=TurtleNormAngle(t->theta+req->angular);
  t->x+=Cos(t->theta)*req->linear;
  t->y+=Sin(t->theta)*req->linear;
  TurtleAddLine(t->sim,t,x0,y0);
}

U8 *TurtleName(CTurtle *t,U8 *what)
{
  return MStrPrint("%s/%s",t->name,what);
}

CTurtle *TurtleAdd(CTurtleSim *ts,U8 *name,F64 x,F64 y,F64 theta)
{
  CTurtle *t=CAlloc(sizeof(CTurtle));
  U8 *s;
  t->name=StrNew(name);
  t->x=x;
  t->y=y;
  t->theta=TurtleNormAngle(theta);
  t->r=179;	//turtlesim's default pen
  t->g=184;
  t->b=255;
  t->width=3;
  t->sim=ts;
  s=TurtleName(t,"cmd_vel");
  t->cmd=RosSubNew(ts->node,s,ros_t_geometry_msgs_Twist,NULL,&TurtleOnCmd,t);
  Free(s);
  s=TurtleName(t,"pose");
  t->pose=RosPubNew(ts->node,s,ros_t_turtlesim_Pose,NULL);
  Free(s);
  s=TurtleName(t,"set_pen");
  t->set_pen=RosServiceNew(ts->node,s,ros_srv_turtlesim_SetPen,&TurtleSetPen,t);
  Free(s);
  s=TurtleName(t,"teleport_absolute");
  t->tele_abs=RosServiceNew(ts->node,s,ros_srv_turtlesim_TeleportAbsolute,&TurtleTeleAbs,t);
  Free(s);
  s=TurtleName(t,"teleport_relative");
  t->tele_rel=RosServiceNew(ts->node,s,ros_srv_turtlesim_TeleportRelative,&TurtleTeleRel,t);
  Free(s);
  RosVecPushI64(ts->turtles,t);
  ts->spawned++;
  return t;
}

U0 TurtleRemove(CTurtleSim *ts,CTurtle *t)
{
  RosVecRemovePtr(ts->turtles,t);
  RosSubDel(t->cmd);
  RosPubDel(t->pose);
  RosServiceDel(t->set_pen);
  RosServiceDel(t->tele_abs);
  RosServiceDel(t->tele_rel);
  Free(t->name);
  Free(t);
}

U0 TurtleClear(CTurtleSim *ts)
{
  I64 i;
  for (i=0;i<ts->trail->len;i++)
    Free(RosVecGetI64(ts->trail,i));
  ts->trail->len=0;
}

U0 TurtleSpawn(CSrv_turtlesim_Spawn_Request *req,CSrv_turtlesim_Spawn_Response *resp,
	CTurtleSim *ts)
{
  U8 *name;
  if (req->name && req->name[0])
    name=StrNew(req->name);
  else
    name=MStrPrint("turtle%d",ts->spawned+1);
  if (TurtleFind(ts,name) || !RosNodeNameOk(name)) {
    RosNodeLog(ts->node,ROS_LOG_ERROR,"A turtle with that name already exists (or bad name)");
    resp->name=StrNew("");
  } else {
    TurtleAdd(ts,name,req->x,req->y,req->theta);
    resp->name=StrNew(name);
  }
  Free(name);
}

U0 TurtleKill(CSrv_turtlesim_Kill_Request *req,U8 *resp,CTurtleSim *ts)
{
  CTurtle *t=TurtleFind(ts,req->name);
  if (t)
    TurtleRemove(ts,t);
  else
    RosNodeLog(ts->node,ROS_LOG_ERROR,"Tried to kill a turtle that doesn't exist");
}

U0 TurtleOnClear(U8 *req,U8 *resp,CTurtleSim *ts)
{
  TurtleClear(ts);
}

U0 TurtleOnReset(U8 *req,U8 *resp,CTurtleSim *ts)
{
  while (ts->turtles->len)
    TurtleRemove(ts,RosVecGetI64(ts->turtles,0));
  TurtleClear(ts);
  ts->spawned=0;
  TurtleAdd(ts,"turtle1",TURTLE_START,TURTLE_START,0.0);
}

//Move every turtle by dt and publish poses.
U0 TurtleStep(CTurtleSim *ts,F64 dt)
{
  I64 i,now=RosTimeSteadyNs();
  CTurtle *t;
  CMsg_turtlesim_Pose *p;
  F64 x0,y0;
  for (i=0;i<ts->turtles->len;i++) {
    t=RosVecGetI64(ts->turtles,i);
    if (now-t->last_cmd_ns>TURTLE_CMD_TIMEOUT) {
      t->lin=0.0;
      t->ang=0.0;
    }
    x0=t->x;
    y0=t->y;
    t->theta=TurtleNormAngle(t->theta+t->ang*dt);
    t->x+=Cos(t->theta)*t->lin*dt;
    t->y+=Sin(t->theta)*t->lin*dt;
    if (t->x<0.0 || t->x>TURTLE_WORLD || t->y<0.0 || t->y>TURTLE_WORLD) {
      t->x=CppMin(CppMax(t->x,0.0),TURTLE_WORLD);
      t->y=CppMin(CppMax(t->y,0.0),TURTLE_WORLD);
    }
    TurtleAddLine(ts,t,x0,y0);
    p=RosPubLoan(t->pose);
    p->x=t->x;
    p->y=t->y;
    p->theta=t->theta;
    p->linear_velocity=t->lin;
    p->angular_velocity=t->ang;
    RosPublishLoaned(t->pose,p);
  }
}

U0 TurtleTick(CTurtleSim *ts)
{
  I64 now=RosTimeSteadyNs();
  F64 dt=RosTimeToSec(now-ts->last_ns);
  ts->last_ns=now;
  if (dt>0.1)
    dt=0.1;
  TurtleStep(ts,dt);
}

extern U0 TurtleSimDel(CTurtleSim *ts);

U0 TurtleSimOnNodeDel(CRosNode *n,CTurtleSim *ts)
{
  TurtleSimDel(ts);
}

//Make the sim on node n (a "turtlesim" node, usually). It goes away
//with the node, or earlier with TurtleSimDel.
CTurtleSim *TurtleSimNew(CRosNode *n)
{
  CTurtleSim *ts=CAlloc(sizeof(CTurtleSim));
  ts->node=n;
  ts->turtles=RosVecNew(8);
  ts->trail=RosVecNew(8);
  ts->last_ns=RosTimeSteadyNs();
  RosServiceNew(n,"/spawn",ros_srv_turtlesim_Spawn,&TurtleSpawn,ts);
  RosServiceNew(n,"/kill",ros_srv_turtlesim_Kill,&TurtleKill,ts);
  RosServiceNew(n,"/clear",ros_srv_std_srvs_Empty,&TurtleOnClear,ts);
  RosServiceNew(n,"/reset",ros_srv_std_srvs_Empty,&TurtleOnReset,ts);
  RosParamDeclare(n,"background_r","69");
  RosParamDeclare(n,"background_g","86");
  RosParamDeclare(n,"background_b","255");
  TurtleAdd(ts,"turtle1",TURTLE_START,TURTLE_START,0.0);
  RosTimerNew(n,TURTLE_DT_NS,&TurtleTick,ts);
  RosNodeOnDel(n,&TurtleSimOnNodeDel,ts);
  RosNodeLog(n,ROS_LOG_INFO,"Spawning turtle [turtle1] at x=[5.544445], y=[5.544445], theta=[0.000000]");
  return ts;
}

U0 TurtleSimDel(CTurtleSim *ts)
{
  ts->node->on_del=NULL;
  while (ts->turtles->len)
    TurtleRemove(ts,RosVecGetI64(ts->turtles,0));
  TurtleClear(ts);
  RosVecDel(ts->turtles);
  RosVecDel(ts->trail);
  if (turtle_sim==ts)
    turtle_sim=NULL;
  Free(ts);
}

//Setup function for RosLaunchAdd.
U0 TurtleSimSetup(CRosNode *n,U8 *arg)
{
  turtle_sim=TurtleSimNew(n);
}

//@host-skip-begin
//---- drawing (TempleOS). VERIFY: DC/Gr names on a real install.

#define TURTLE_PX	400	//world drawn this many pixels across

I64 TurtleColor(I64 r,I64 g,I64 b)	//nearest of the 16 colors, roughly
{
  I64 c=0;
  if (r>127) c|=4;
  if (g>127) c|=2;
  if (b>127) c|=1;
  if (r>200 || g>200 || b>200) c|=8;
  return c;
}

I64 TurtleSx(F64 x) {return ToI64(x/TURTLE_WORLD*TURTLE_PX)+8;}
I64 TurtleSy(F64 y) {return TURTLE_PX-ToI64(y/TURTLE_WORLD*TURTLE_PX)+8;}

U0 TurtleDrawIt(CTask *task,CDC *dc)
{
  CTurtleSim *ts=turtle_sim;
  CTurtleLine *l;
  CTurtle *t;
  I64 i;
  if (!ts) return;
  dc->color=BLUE;
  GrRect(dc,8,8,TURTLE_PX,TURTLE_PX);
  for (i=0;i<ts->trail->len;i++) {
    l=RosVecGetI64(ts->trail,i);
    dc->color=TurtleColor(l->r,l->g,l->b);
    GrLine(dc,TurtleSx(l->x0),TurtleSy(l->y0),TurtleSx(l->x1),TurtleSy(l->y1));
  }
  for (i=0;i<ts->turtles->len;i++) {
    t=RosVecGetI64(ts->turtles,i);
    dc->color=YELLOW;
    GrCircle(dc,TurtleSx(t->x),TurtleSy(t->y),6);
    dc->color=GREEN;
    GrLine(dc,TurtleSx(t->x),TurtleSy(t->y),
	TurtleSx(t->x+Cos(t->theta)*0.4),TurtleSy(t->y+Sin(t->theta)*0.4));
  }
}

//Run turtlesim in this task's window. Arrow keys drive turtle1.
U0 TurtleSim()
{
  CRosNode *n=RosNodeNew("turtlesim",NULL);
  CRosPub *teleop=RosPubNew(n,"/turtle1/cmd_vel",ros_t_geometry_msgs_Twist,NULL);
  CMsg_geometry_msgs_Twist cmd;
  I64 ch,sc;
  ros_ok=TRUE;
  turtle_sim=TurtleSimNew(n);
  DocClear;
  Fs->draw_it=&TurtleDrawIt;
  while (ros_ok) {
    if (ScanKey(&ch,&sc)) {
      MemSet(&cmd,0,sizeof(CMsg_geometry_msgs_Twist));
      switch (sc.u8[0]) {
        case SC_CURSOR_UP:    cmd.linear.x=2.0;  break;
        case SC_CURSOR_DOWN:  cmd.linear.x=-2.0; break;
        case SC_CURSOR_LEFT:  cmd.angular.z=2.0; break;
        case SC_CURSOR_RIGHT: cmd.angular.z=-2.0; break;
      }
      if (ch==CH_ESC || ch==CH_SHIFT_ESC)
        break;
      RosPublish(teleop,&cmd);
    }
    RosExecSpinOnce(RosNodeExec(n),10*ROS_NS_PER_MS);
  }
  Fs->draw_it=NULL;
  TurtleSimDel(turtle_sim);
  RosExecDel(n->exec);
  RosNodeDel(n);
}
//@host-skip-end
