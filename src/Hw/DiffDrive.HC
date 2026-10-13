//DiffDrive: a two-wheeled robot base, like ros2_control's
//diff_drive_controller: geometry_msgs/Twist on cmd_vel in, wheel
//speeds out to a RoboClaw (M1 left, M2 right), encoders back in,
//nav_msgs/Odometry on odom and odom -> base_link on /tf.
//
//  CRosDiffDrive *dd=RosDiffDriveNew(node,RoboClawOpen(RosSerialOpenCom(0x2F8,38400),0x80));
//  CRosDiffDrive *sim=RosDiffDriveNew(node,NULL);   //no hardware: simulated wheels
//
//Parameters (on node): wheel_radius (m), wheel_separation (m),
//ticks_per_rev, cmd_timeout (s, stop if cmd_vel goes quiet), rate (Hz),
//odom_frame, base_frame, publish_tf.

class CRosDiffDrive
{
  CRosNode *node;
  CRosRoboClaw *rc;	//NULL: simulated wheels
  CRosSub *cmd;
  CRosPub *odom;
  CRosTfBroadcaster *tf;
  CRosTimer *timer;
  F64 radius,sep,ticks,timeout;
  F64 v,w;		//commanded
  I64 last_cmd_ns,last_ns;
  F64 x,y,th;		//pose in odom
  I64 enc_l,enc_r;
  Bool have_enc;
  F64 sim_l,sim_r;	//simulated encoder counts
  I64 sim_ql,sim_qr;	//simulated speeds (counts/s)
  I64 hw_errors;
  U8 *odom_frame,*base_frame;
  Bool publish_tf;
};

U0 RosDiffDriveOnCmd(CMsg_geometry_msgs_Twist *m,CRosDiffDrive *dd)
{
  dd->v=m->linear.x;
  dd->w=m->angular.z;
  dd->last_cmd_ns=RosTimeSteadyNs();
}

//Wheel speeds in encoder counts per second for (v,w).
U0 RosDiffDriveWheels(CRosDiffDrive *dd,F64 v,F64 w,I64 *ql,I64 *qr)
{
  F64 k=dd->ticks/(2.0*pi*dd->radius);	//counts per meter of wheel travel
  *ql=ToI64(Floor((v-w*dd->sep/2.0)*k+0.5));
  *qr=ToI64(Floor((v+w*dd->sep/2.0)*k+0.5));
}

U0 RosDiffDriveTick(CRosDiffDrive *dd)
{
  I64 now=RosTimeSteadyNs(),ql,qr,l,r,stamp;
  F64 dt,dl,dr,d,dth,per_tick;
  CMsg_nav_msgs_Odometry *o;
  CRosTf t;
  dt=RosTimeToSec(now-dd->last_ns);
  dd->last_ns=now;
  if (now-dd->last_cmd_ns>ToI64(dd->timeout*ROS_NS_PER_SEC)) {
    dd->v=0.0;
    dd->w=0.0;
  }
  RosDiffDriveWheels(dd,dd->v,dd->w,&ql,&qr);
  if (dd->rc) {
    if (!RoboClawSpeedM1M2(dd->rc,ql,qr) || !RoboClawEncoders(dd->rc,&l,&r)) {
      dd->hw_errors++;
      return;
    }
  } else {	//simulated: the wheels do what they're told
    dd->sim_l+=dd->sim_ql*dt;
    dd->sim_r+=dd->sim_qr*dt;
    dd->sim_ql=ql;
    dd->sim_qr=qr;
    l=ToI64(Floor(dd->sim_l));
    r=ToI64(Floor(dd->sim_r));
  }
  if (!dd->have_enc) {
    dd->enc_l=l;
    dd->enc_r=r;
    dd->have_enc=TRUE;
    return;
  }
  per_tick=2.0*pi*dd->radius/dd->ticks;
  dl=(l-dd->enc_l)*per_tick;
  dr=(r-dd->enc_r)*per_tick;
  dd->enc_l=l;
  dd->enc_r=r;
  d=(dl+dr)/2.0;
  dth=(dr-dl)/dd->sep;
  dd->x+=d*Cos(dd->th+dth/2.0);
  dd->y+=d*Sin(dd->th+dth/2.0);
  dd->th=Arg(Cos(dd->th+dth),Sin(dd->th+dth));	//keep it in -pi..pi
  stamp=RosClockNow(ROS_CLOCK_ROS);
  RosTfIdentity(&t);
  t.x=dd->x;
  t.y=dd->y;
  RosTfSetRpy(&t,0.0,0.0,dd->th);
  o=RosPubLoan(dd->odom);
  o->header.stamp.sec=RosTimeSec(stamp);
  o->header.stamp.nanosec=RosTimeNsec(stamp);
  o->header.frame_id=StrNew(dd->odom_frame);
  o->child_frame_id=StrNew(dd->base_frame);
  o->pose.pose.position.x=t.x;
  o->pose.pose.position.y=t.y;
  o->pose.pose.orientation.z=t.qz;
  o->pose.pose.orientation.w=t.qw;
  if (dt>0.0) {
    o->twist.twist.linear.x=d/dt;
    o->twist.twist.angular.z=dth/dt;
  }
  RosPublishLoaned(dd->odom,o);
  if (dd->publish_tf)
    RosTfSend(dd->tf,dd->odom_frame,dd->base_frame,stamp,&t);
}

//rc: the motor controller, or NULL to simulate the wheels.
CRosDiffDrive *RosDiffDriveNew(CRosNode *n,CRosRoboClaw *rc)
{
  CRosDiffDrive *dd=CAlloc(sizeof(CRosDiffDrive));
  F64 rate;
  dd->node=n;
  dd->rc=rc;
  RosParamDeclare(n,"wheel_radius","0.05");
  RosParamDeclare(n,"wheel_separation","0.3");
  RosParamDeclare(n,"ticks_per_rev","1000.0");
  RosParamDeclare(n,"cmd_timeout","0.5");
  RosParamDeclare(n,"rate","20.0");
  RosParamDeclare(n,"odom_frame","odom");
  RosParamDeclare(n,"base_frame","base_link");
  RosParamDeclare(n,"publish_tf","true");
  dd->radius=RosParamF64(n,"wheel_radius");
  dd->sep=RosParamF64(n,"wheel_separation");
  dd->ticks=RosParamF64(n,"ticks_per_rev");
  dd->timeout=RosParamF64(n,"cmd_timeout");
  rate=RosParamF64(n,"rate");
  if (rate<=0.0)
    rate=20.0;
  dd->odom_frame=StrNew(RosParamStr(n,"odom_frame"));
  dd->base_frame=StrNew(RosParamStr(n,"base_frame"));
  dd->publish_tf=RosParamBool(n,"publish_tf");
  dd->cmd=RosSubNew(n,"cmd_vel",ros_t_geometry_msgs_Twist,NULL,&RosDiffDriveOnCmd,dd);
  dd->odom=RosPubNew(n,"odom",ros_t_nav_msgs_Odometry,NULL);
  dd->tf=RosTfBroadcasterNew(n);
  dd->last_ns=RosTimeSteadyNs();
  dd->timer=RosTimerNew(n,ToI64(ROS_NS_PER_SEC/rate),&RosDiffDriveTick,dd);
  if (rc)
    RoboClawResetEncoders(rc);
  return dd;
}

//Stops the motors. The controller (rc) stays yours.
U0 RosDiffDriveDel(CRosDiffDrive *dd)
{
  if (!dd)
    return;
  if (dd->rc)
    RoboClawSpeedM1M2(dd->rc,0,0);
  RosTimerDel(dd->timer);
  RosSubDel(dd->cmd);
  RosPubDel(dd->odom);
  RosTfBroadcasterDel(dd->tf);
  Free(dd->odom_frame);
  Free(dd->base_frame);
  Free(dd);
}
