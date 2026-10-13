//Parameters: a YAML file, declaring, a guard callback, and changing a
//parameter through the set_parameters service like `ros2 param set`.
//  #include "~/ROS2-Holy-C/examples/ParamDemo.HC"
//  ParamDemo;
//On Linux:
//  python3 tools/hctest/hc2c.py run examples/ParamDemo.HC --main ParamDemoHost

Cd(__DIR__);;
#include "../Load.HC"

//Refuse speeds above 2 m/s.
Bool ParamDemoGuard(CRosNode *n,U8 *name,CMsg_rcl_interfaces_ParameterValue *v,U8 *user)
{
  if (!StrCmp(name,"max_speed") && v->double_value>2.0) {
    RosNodeLog(n,ROS_LOG_WARN,"max_speed above 2.0 refused");
    return FALSE;
  }
  return TRUE;
}

U0 ParamDemoEvent(CMsg_rcl_interfaces_ParameterEvent *ev,U8 *user)
{
  CMsg_rcl_interfaces_Parameter *ps=ev->changed_parameters.data;
  U8 *v;
  if (!ev->changed_parameters.len)
    return;
  v=RosPvToText(&ps[0].value);
  Print("event: %s changed %s to %s\n",ev->node,ps[0].name,v);
  Free(v);
}

I64 ParamDemo()
{
  CRosNode *robot,*tool;
  CRosExec *e=RosExecNew();
  CRosClient *c;
  CRosType *t=ros_t_rcl_interfaces_SetParameters_Request;
  U8 *req,*resp,*s;
  I64 ok=0;
  RosParamLoadYaml("/demo/robot:\n  ros__parameters:\n    max_speed: 1.5\n    name: rover\n");
  robot=RosNodeNew("robot","demo");
  tool=RosNodeNew("param_tool",NULL);
  RosExecAdd(e,robot);
  RosExecAdd(e,tool);
  RosSubNew(tool,"/parameter_events",ros_t_rcl_interfaces_ParameterEvent,NULL,
	&ParamDemoEvent,NULL);
  RosParamDeclare(robot,"max_speed","0.5");	//the YAML value wins
  RosParamDeclare(robot,"name","'unnamed'");
  RosParamDeclare(robot,"wheel_count","4");
  RosParamOnSet(robot,&ParamDemoGuard,NULL);
  Print("max_speed=%f name=%s wheels=%d\n",RosParamF64(robot,"max_speed"),
	RosParamStr(robot,"name"),RosParamI64(robot,"wheel_count"));

  //what `ros2 param set /demo/robot max_speed 1.8` does
  c=RosClientNew(tool,"/demo/robot/set_parameters",ros_srv_rcl_interfaces_SetParameters);
  req=RosMsgNew(t);
  RosMsgSet(t,req,"parameters[0].name","max_speed");
  RosMsgSet(t,req,"parameters[0].value.type","3");
  RosMsgSet(t,req,"parameters[0].value.double_value","1.8");
  resp=RosClientCallWait(c,e,req,ROS_NS_PER_SEC);
  if (resp) {
    s=RosMsgToYaml(ros_t_rcl_interfaces_SetParameters_Response,resp);
    Print("set_parameters answered:\n%s",s);
    Free(s);
    RosMsgDel(ros_t_rcl_interfaces_SetParameters_Response,resp);
  }
  RosMsgDel(t,req);
  if (!RosParamSet(robot,"max_speed","9.0"))
    Print("direct set refused: %s\n",ros_param_error);
  RosExecSpinSome(e);
  if (RosParamF64(robot,"max_speed")==1.8)
    ok=1;
  RosNodeDel(robot);
  RosNodeDel(tool);
  RosExecDel(e);
  if (ok)
    return 0;
  return 1;
}

I64 ParamDemoHost()
{
  RosIfaceLoadAll();
  return ParamDemo();
}
