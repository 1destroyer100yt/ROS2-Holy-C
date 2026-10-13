//Launch file: turtlesim + a node that drives turtle1 in a square.
//In TempleOS (watch it draw):
//  #include "~/ROS2-Holy-C/examples/TurtleSquare.HC"
//  TurtleSquare;          //Esc stops
//On Linux (no drawing, 3 seconds):
//  python3 tools/hctest/hc2c.py run examples/TurtleSquare.HC --main TurtleSquareHost

Cd(__DIR__);;
#include "../Load.HC"

class CSquare
{
  CRosNode *node;
  CRosPub *cmd;
  I64 phase;	//even: drive straight, odd: turn
  I64 phase_end;
  F64 speed;
};

U0 SquareTick(CSquare *sq)
{
  CMsg_geometry_msgs_Twist tw;
  I64 now=RosTimeSteadyNs();
  F64 turn_s=(pi/2.0)/1.0;	//90 degrees at 1 rad/s
  MemSet(&tw,0,sizeof(CMsg_geometry_msgs_Twist));
  if (now>=sq->phase_end) {
    sq->phase++;
    if (sq->phase&1)
      sq->phase_end=now+RosTimeFromSec(turn_s);
    else
      sq->phase_end=now+RosTimeFromSec(2.0/sq->speed);
  }
  if (sq->phase&1)
    tw.angular.z=1.0;
  else
    tw.linear.x=sq->speed;
  RosPublish(sq->cmd,&tw);
}

U0 SquareOnDel(CRosNode *n,CSquare *sq)
{
  Free(sq);
}

U0 SquareSetup(CRosNode *n,U8 *arg)
{
  CSquare *sq=CAlloc(sizeof(CSquare));
  sq->node=n;
  RosParamDeclare(n,"speed","1.0");
  sq->speed=RosParamF64(n,"speed");
  sq->cmd=RosPubNew(n,"/turtle1/cmd_vel",ros_t_geometry_msgs_Twist,NULL);
  sq->phase_end=RosTimeSteadyNs()+RosTimeFromSec(2.0/sq->speed);
  RosTimerNew(n,100*ROS_NS_PER_MS,&SquareTick,sq);
  RosNodeOnDel(n,&SquareOnDel,sq);
}

CRosLaunch *TurtleSquareLaunch()
{
  CRosLaunch *l=RosLaunchNew();
  CRosLaunchNode *sq;
  RosLaunchAdd(l,"turtlesim",NULL,&TurtleSimSetup,NULL);
  sq=RosLaunchAdd(l,"draw_square",NULL,&SquareSetup,NULL);
  RosLaunchParam(sq,"speed","2.0");
  return l;
}

//@host-skip-begin
U0 TurtleSquare()
{
  CRosLaunch *l=TurtleSquareLaunch();
  RosLaunchStartHere(l);
  DocClear;
  Fs->draw_it=&TurtleDrawIt;
  while (ros_ok && !RosCliKey())
    RosExecSpinOnce(l->exec,10*ROS_NS_PER_MS);
  Fs->draw_it=NULL;
  RosLaunchStop(l);
}
//@host-skip-end

I64 TurtleSquareHost()
{
  CRosLaunch *l;
  CTurtle *t;
  F64 x,y;
  I64 lines;
  RosIfaceLoadAll();
  l=TurtleSquareLaunch();
  RosLaunchStartHere(l);
  RosLaunchSpinFor(l,3*ROS_NS_PER_SEC);
  t=TurtleFind(turtle_sim,"turtle1");
  x=t->x;
  y=t->y;
  lines=turtle_sim->trail->len;
  Print("after 3 s: turtle1 at (%.2f, %.2f), %d trail segments\n",x,y,lines);
  RosLaunchStop(l);
  //1 s straight at 2 m/s, then turning: it must have left the start
  if (x>TURTLE_START+1.5 && lines>50)
    return 0;
  return 1;
}
