//Tests for src/Hw (RoboClaw packets, DiffDrive). The serial side is
//tested against a RoboClaw simulator in tools/hwtest.

U0 TestRoboClawCrc()
{
  U8 a[10],b[2];
  RosTestGroup("RoboClaw CRC");
  //expected values from the official Basicmicro Python library
  RosTestI64("CRC-16/XMODEM check",RoboClawCrcOf("123456789",9),0x31C3);
  a[0]=0x80; a[1]=37; a[2]=0; a[3]=0; a[4]=0x06; a[5]=0x37;
  a[6]=0xFF; a[7]=0xFF; a[8]=0xF9; a[9]=0xC9;	//M1 1591, M2 -1591
  RosTestI64("speed packet",RoboClawCrcOf(a,10),0xB723);
  b[0]=0x80;
  b[1]=78;
  RosTestI64("read encoders",RoboClawCrcOf(b,2),0xB292);
  RosTestI64("signed 32",RoboClawGet32(a+6),-1591);
}

class CTestOdom
{
  I64 n;
  F64 x,y,yaw,vx,wz;
};

U0 TestOdomCb(CMsg_nav_msgs_Odometry *m,CTestOdom *o)
{
  CRosTf t;
  F64 r,p;
  o->n++;
  o->x=m->pose.pose.position.x;
  o->y=m->pose.pose.position.y;
  RosTfSetXyzQ(&t,0.0,0.0,0.0,m->pose.pose.orientation.x,m->pose.pose.orientation.y,
	m->pose.pose.orientation.z,m->pose.pose.orientation.w);
  RosTfGetRpy(&t,&r,&p,&o->yaw);
  o->vx=m->twist.twist.linear.x;
  o->wz=m->twist.twist.angular.z;
}

//Drive for secs, publishing cmd_vel at 20 Hz like a teleop would.
U0 TestDriveFor(CRosExec *e,CRosPub *p,F64 v,F64 w,F64 secs)
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

U0 TestDiffDriveSim()
{
  CRosNode *base=RosNodeNew("base",NULL),*teleop=RosNodeNew("teleop",NULL);
  CRosExec *e=RosExecNew();
  CRosDiffDrive *dd;
  CRosPub *cmd;
  CTestOdom o;
  CRosTfBuffer *tb=RosTfBufferNew();
  CRosTfListener *tl;
  CRosTf t;
  I64 ql,qr;
  RosTestGroup("DiffDrive (simulated wheels)");
  MemSet(&o,0,sizeof(o));
  RosExecAdd(e,base);
  RosExecAdd(e,teleop);
  dd=RosDiffDriveNew(base,NULL);
  RosDiffDriveWheels(dd,0.5,0.0,&ql,&qr);
  RosTestI64("0.5 m/s: left counts/s",ql,1592);	//0.5/(2 pi 0.05)*1000
  RosTestI64("0.5 m/s: right counts/s",qr,1592);
  RosDiffDriveWheels(dd,0.0,1.0,&ql,&qr);
  RosTestTrue("turning: wheels opposite",ql==-qr && qr==477);
  cmd=RosPubNew(teleop,"cmd_vel",ros_t_geometry_msgs_Twist,NULL);
  RosSubNew(teleop,"odom",ros_t_nav_msgs_Odometry,NULL,&TestOdomCb,&o);
  tl=RosTfListenerNew(teleop,tb);
  TestDriveFor(e,cmd,0.5,0.0,1.0);
  RosTestTrue("odom published",o.n>=15);
  RosTestTrue("drove about 0.5 m",Abs(o.x-0.5)<0.06 && Abs(o.y)<0.01);
  RosTestTrue("speed in odom",Abs(o.vx-0.5)<0.05);
  TestDriveFor(e,cmd,0.0,pi/2.0,1.0);
  RosTestTrue("turned about 90 degrees",Abs(o.yaw-pi/2.0)<0.12);
  TestDriveFor(e,cmd,0.0,0.0,0.3);
  RosTestTrue("tf odom->base_link",RosTfLookup(tb,"odom","base_link",0,&t) &&
	Abs(t.x-o.x)<0.03);
  dd->timeout=0.1;	//cmd_vel stops: the robot stops
  RosExecSpinFor(e,400*ROS_NS_PER_MS);
  RosTestTrue("stops without cmd_vel",dd->v==0.0 && Abs(o.vx)<0.001);
  RosTfListenerDel(tl);
  RosTfBufferDel(tb);
  RosDiffDriveDel(dd);
  RosNodeDel(base);
  RosNodeDel(teleop);
  RosExecDel(e);
}

U0 TestHw()
{
  TestRoboClawCrc();
  TestDiffDriveSim();
}
