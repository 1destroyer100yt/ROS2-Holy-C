//Messages in ROS2-Holy-C. In TempleOS:
//  #include "~/ROS2-Holy-C/examples/MsgDemo.HC"
//  MsgDemo;
//On Linux:
//  python3 tools/hctest/hc2c.py run examples/MsgDemo.HC --main MsgDemo

Cd(__DIR__);;
#include "../Load.HC"

I64 MsgDemo()
{
  CMsg_geometry_msgs_Twist *cmd;
  CMsg_sensor_msgs_LaserScan *scan,*copy;
  CRosType *t;
  CRosBuf *b;
  F64 *ranges;
  I64 i;

  RosIfaceLoadAll();	//already done by Load.HC in TempleOS; harmless twice

  //1. Typed access: the generated classes are plain HolyC classes.
  cmd=RosMsgNew(ros_t_geometry_msgs_Twist);
  cmd->linear.x=0.5;
  cmd->angular.z=-1.25;
  Print("A Twist:\n");
  RosMsgPrint(ros_t_geometry_msgs_Twist,cmd);

  //2. Sequences: resize, then fill the elements in place.
  t=ros_t_sensor_msgs_LaserScan;
  scan=RosMsgNew(t);
  RosMsgSet(t,scan,"header.frame_id","laser");
  scan->angle_min=-1.5;
  scan->range_max=10.0;
  RosSeqResize(RosTypeField(t,"ranges"),scan,4);
  ranges=scan->ranges.data;
  for (i=0;i<4;i++)
    ranges[i]=1.0+i*0.25;

  //3. Set anything by path from text (what RosTopicPub will use).
  RosMsgSet(t,scan,"header.stamp.sec","1700000000");
  RosMsgSet(t,scan,"intensities","[100.0, 50.0]");

  //4. Serialize exactly like ROS 2 does on the wire, and back.
  b=RosBufNew();
  RosCdrSer(t,scan,b);
  Print("LaserScan is %d bytes on the wire\n",b->len);
  copy=RosMsgNew(t);
  if (!RosCdrDeser(t,b->data,b->len,copy))
    Print("decode failed: %s\n",ros_msg_error);
  Print("Decoded again:\n");
  RosMsgPrint(t,copy);

  //5. Constants from .msg files are #defines.
  Print("NavSatStatus.STATUS_FIX = %d\n",ROS_sensor_msgs_NavSatStatus_STATUS_FIX);

  RosBufDel(b);
  RosMsgDel(t,copy);
  RosMsgDel(t,scan);
  RosMsgDel(ros_t_geometry_msgs_Twist,cmd);
  return 0;
}
