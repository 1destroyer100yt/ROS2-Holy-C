//Tests for src/Plugin/PluginLib.HC with the cpp2hc-converted
//pluginlib tutorial plugins (tools/cpp2hc/examples/polygon).

//"Hot reload": a new version of Square's area, registered while a
//Square object exists.
F64 Square_area_v2(Square *self)
{
  return 2.0*self->side_length_*self->side_length_;
}

U0 TestPlugin()
{
  CPluginObj *sq,*tri,*bad;
  U0 (*init)(U8 *self,F64 side);
  F64 (*area)(U8 *self);
  CRosVec *v;
  RosTestGroup("Plugins");
  PluginDescribe("polygon_plugins::Square","polygon_base::RegularPolygon","A square.");
  RosTestTrue("declared, not loaded",!PluginIsLoaded("polygon_plugins::Square"));
  PluginInit_polygons();
  RosTestTrue("loaded",PluginIsLoaded("polygon_plugins::Square") &&
	PluginIsLoaded("polygon_plugins::Triangle"));
  v=PluginClassesFor("polygon_base::RegularPolygon");
  RosTestI64("classes for base",v->len,2);
  RosTestStr("sorted",RosVecGetI64(v,0),"polygon_plugins::Square");
  RosStrVecDel(v);
  sq=PluginCreateAs("polygon_plugins::Square","polygon_base::RegularPolygon");
  tri=PluginCreate("polygon_plugins::Triangle");
  RosTestTrue("created",sq && tri);
  bad=PluginCreateAs("polygon_plugins::Square","some_other::Base");
  RosTestTrue("wrong base refused",bad==NULL);
  RosTestTrue("unknown refused",!PluginCreate("nope::Nope"));
  init=PluginMethod(sq,"initialize");
  (*init)(sq->self,3.0);
  area=PluginMethod(sq,"area");
  RosTestF64("square area",(*area)(sq->self),9.0,0.0);
  init=PluginMethod(tri,"initialize");
  (*init)(tri->self,2.0);
  area=PluginMethod(tri,"area");
  RosTestF64("triangle area",(*area)(tri->self),Sqrt(3.0),1e-12);
  RosTestTrue("missing method",PluginMethod(sq,"perimeter")==NULL);
  RosTestI64("live objects",PluginClassFind("polygon_plugins::Square")->live_objs,1);
  //reload: same object, new code
  PluginMethodAdd("polygon_plugins::Square","area",&Square_area_v2);
  area=PluginMethod(sq,"area");
  RosTestF64("reloaded method, old object",(*area)(sq->self),18.0,0.0);
  PluginMethodAdd("polygon_plugins::Square","area",&Square_area);
  PluginDestroy(sq);
  PluginDestroy(tri);
  RosTestI64("live objects after destroy",PluginClassFind("polygon_plugins::Square")->live_objs,0);
}
