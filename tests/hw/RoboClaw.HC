//HolyC side of tools/hwtest/run.sh: DiffDrive on a RoboClaw
//(simulated by tools/hwtest/roboclaw_sim.py, which the official
//Basicmicro library talks to). Over a pty, or over the 16550 UART
//(COM2) of a QEMU guest.

Cd(__DIR__);;
#include "../../Load.HC"

class CHwOdom
{
  I64 n;
  F64 x,y,yaw;
};

U0 HwOdomCb(CMsg_nav_msgs_Odometry *m,CHwOdom *o)
{
  CRosTf t;
  F64 r,p;
  o->n++;
  o->x=m->pose.pose.position.x;
  o->y=m->pose.pose.position.y;
  RosTfSetXyzQ(&t,0.0,0.0,0.0,m->pose.pose.orientation.x,m->pose.pose.orientation.y,
	m->pose.pose.orientation.z,m->pose.pose.orientation.w);
  RosTfGetRpy(&t,&r,&p,&o->yaw);
}

U0 HwDrive(CRosExec *e,CRosPub *p,F64 v,F64 w,F64 secs)
{
  CMsg_geometry_msgs_Twist t;
  I64 end=RosTimeSteadyNs()+ToI64(secs*ROS_NS_PER_SEC),next=0;
  MemSet(&t,0,sizeof(t));
  t.linear.x=v;
  t.angular.z=w;
  while (RosTimeSteadyNs()<end) {
    if (RosTimeSteadyNs()>=next) {
      RosPublish(p,&t);
      next=RosTimeSteadyNs()+50*ROS_NS_PER_MS;
    }
    RosExecSpinOnce(e,5*ROS_NS_PER_MS);
  }
}

I64 HwRoboClawBody(CRosSerial *ser)
{
  CRosRoboClaw *rc=RoboClawOpen(ser,0x80);
  CRosNode *base,*teleop;
  CRosExec *e=RosExecNew();
  CRosDiffDrive *dd;
  CRosPub *cmd;
  CHwOdom o;
  U8 *ver;
  Bool ok;
  RosIfaceLoadAll();
  MemSet(&o,0,sizeof(o));
  ver=RoboClawVersion(rc);
  if (!ver)
    ver=StrNew("(none)");
  if (StrLen(ver) && ver[StrLen(ver)-1]=='\n')
    ver[StrLen(ver)-1]=0;
  base=RosNodeNew("base",NULL);
  teleop=RosNodeNew("teleop",NULL);
  RosExecAdd(e,base);
  RosExecAdd(e,teleop);
  dd=RosDiffDriveNew(base,rc);
  cmd=RosPubNew(teleop,"cmd_vel",ros_t_geometry_msgs_Twist,NULL);
  RosSubNew(teleop,"odom",ros_t_nav_msgs_Odometry,NULL,&HwOdomCb,&o);
  HwDrive(e,cmd,0.5,0.0,1.0);		//forward 0.5 m
  HwDrive(e,cmd,0.0,pi/2.0,1.0);	//turn left 90 degrees
  HwDrive(e,cmd,0.0,0.0,0.3);
  ok=RosStrStartsWith(ver,"USB Roboclaw") && Abs(o.x-0.5)<0.08 && Abs(o.yaw-pi/2.0)<0.15 &&
	o.n>30 && !dd->hw_errors;
  Print("RESULT holyc: version='%s' odom x=%.3f y=%.3f yaw=%.3f msgs=%d hw_errors=%d crc_retries=%d -> %d\n",
	ver,o.x,o.y,o.yaw,o.n,dd->hw_errors,rc->errors,ok);
//@host: fflush(stdout);
  Free(ver);
  RosDiffDriveDel(dd);
  RosNodeDel(base);
  RosNodeDel(teleop);
  RosExecDel(e);
  RoboClawClose(rc);
  if (ok)
    return 0;
  return 1;
}

//Over a pty: its path is in the ROBOCLAW_TTY environment variable.
I64 HwRoboClawPty()
{
  I64 fd=-1;
//@host: struct termios t; fd=open(getenv("ROBOCLAW_TTY"),O_RDWR|O_NONBLOCK|O_NOCTTY);
//@host: if (fd>=0) { tcgetattr(fd,&t); cfmakeraw(&t); tcsetattr(fd,TCSANOW,&t); }
  if (fd<0) {
    Print("RESULT holyc: can't open the pty\n");
    return 1;
  }
  return HwRoboClawBody(RosSerialOpenFd(fd));
}

//Over COM2's UART in a QEMU guest (as /init, like tools/e1000test).
I64 HwRoboClawCom()
{
  I64 r;
//@host: iopl(3);
  r=HwRoboClawBody(RosSerialOpenCom(0x2F8,38400));
//@host: fflush(stdout); sync(); reboot(RB_POWER_OFF);
  return r;
}
