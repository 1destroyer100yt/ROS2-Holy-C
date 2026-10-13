//Plugins inside a node, the way nav2 picks its planners: a parameter
//names the plugin class, and changing the parameter swaps the
//implementation while the node runs. The plugins are the C++ pluginlib
//tutorial polygons, converted by tools/cpp2hc.
//
//In TempleOS:
//  #include "~/ROS2-Holy-C/examples/PluginNode.HC"
//  PluginNode;     //publishes /area; try RosParamSetCli("/polygon_node","plugin","'polygon_plugins::Triangle'")
//On Linux:
//  python3 tools/hctest/hc2c.py run examples/PluginNode.HC --main PluginNodeHost

Cd(__DIR__);;
#include "../Load.HC"
#include "../tools/cpp2hc/examples/polygon/Polygons.HC"

#define POLYGON_BASE "polygon_base::RegularPolygon"

class CPolygonNode
{
  CRosNode *node;
  CRosPub *pub;
  CPluginObj *poly;
};

//Make the plugin named by the "plugin" parameter. FALSE if unknown.
Bool PolygonUse(CPolygonNode *pn,U8 *cls)
{
  CPluginObj *o=PluginCreateAs(cls,POLYGON_BASE);
  U0 (*init)(U8 *self,F64 side);
  if (!o)
    return FALSE;
  init=PluginMethod(o,"initialize");
  (*init)(o->self,RosParamF64(pn->node,"side"));
  PluginDestroy(pn->poly);
  pn->poly=o;
  return TRUE;
}

Bool PolygonParamCheck(CRosNode *n,U8 *name,CMsg_rcl_interfaces_ParameterValue *v,
	CPolygonNode *pn)
{
  if (StrCmp(name,"plugin"))
    return TRUE;
  return PolygonUse(pn,v->string_value);	//refuse classes that don't exist
}

U0 PolygonTick(CPolygonNode *pn)
{
  CMsg_std_msgs_Float64 m;
  F64 (*area)(U8 *self);
  if (!pn->poly)
    return;
  area=PluginMethod(pn->poly,"area");
  m.data=(*area)(pn->poly->self);
  RosPublish(pn->pub,&m);
}

U0 PolygonOnDel(CRosNode *n,CPolygonNode *pn)
{
  PluginDestroy(pn->poly);
  Free(pn);
}

CPolygonNode *PolygonNodeNew()
{
  CPolygonNode *pn=CAlloc(sizeof(CPolygonNode));
  CRosVec *v;
  I64 i;
  pn->node=RosNodeNew("polygon_node",NULL);
  RosParamDeclare(pn->node,"side","2.0");
  RosParamDeclare(pn->node,"plugin","'polygon_plugins::Square'");
  pn->pub=RosPubNew(pn->node,"area",ros_t_std_msgs_Float64,NULL);
  v=PluginClassesFor(POLYGON_BASE);
  for (i=0;i<v->len;i++)
    Print("available: %s\n",RosVecGetI64(v,i));
  RosStrVecDel(v);
  PolygonUse(pn,RosParamStr(pn->node,"plugin"));
  RosParamOnSet(pn->node,&PolygonParamCheck,pn);
  RosTimerNew(pn->node,100*ROS_NS_PER_MS,&PolygonTick,pn);
  RosNodeOnDel(pn->node,&PolygonOnDel,pn);
  return pn;
}

//@host-skip-begin
U0 PluginNode()
{
  CPolygonNode *pn=PolygonNodeNew();
  RosSpin(pn->node);
}
//@host-skip-end

U0 PluginNodeArea(CMsg_std_msgs_Float64 *m,F64 *out)
{
  *out=m->data;
}

I64 PluginNodeHost()
{
  CPolygonNode *pn;
  CRosNode *me;
  CRosExec *e=RosExecNew();
  F64 area=0.0,sq,tri;
  Bool refused;
  RosIfaceLoadAll();
  PluginInit_polygons();	//TempleOS runs this when the file loads
  pn=PolygonNodeNew();
  me=RosNodeNew("watcher",NULL);
  RosExecAdd(e,pn->node);
  RosExecAdd(e,me);
  RosSubNew(me,"/area",ros_t_std_msgs_Float64,NULL,&PluginNodeArea,&area);
  RosExecSpinFor(e,250*ROS_NS_PER_MS);
  sq=area;
  Print("Square, side 2: area %f\n",sq);
  RosParamSet(pn->node,"plugin","'polygon_plugins::Triangle'");
  RosExecSpinFor(e,250*ROS_NS_PER_MS);
  tri=area;
  Print("Triangle, side 2: area %f\n",tri);
  refused=!RosParamSet(pn->node,"plugin","'polygon_plugins::Hexagon'");
  Print("unknown plugin refused: %d\n",refused);
  RosNodeDel(pn->node);
  RosNodeDel(me);
  RosExecDel(e);
  if (sq==4.0 && Abs(tri-Sqrt(3.0))<1e-9 && refused)
    return 0;
  return 1;
}
