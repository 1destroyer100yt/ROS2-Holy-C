//Converted by cpp2hc from: planner.cpp
//Needs ROS2-Holy-C loaded first (src/Load.HC).
//Search for TODO(cpp2hc) and fix every one before trusting this file.

class Point
{
  F64 x_;
  F64 y_;
};

class StraightLine
{
  U8 *name_;
  rclcpp::Node::SharedPtr node_; //TODO(cpp2hc): unknown type 'rclcpp::Node::SharedPtr'
  F64 step_;
  U64 count_;
  Point *last_;
};

extern Point *Point_New(F64 x,F64 y);
extern F64 Point_dist(Point *self,Point *o);
extern F64 Point_heading(Point *self,Point *o);
extern StraightLine *StraightLine_New();
extern U0 StraightLine_Del(StraightLine *self);
extern U0 StraightLine_configure(StraightLine *self,U8 *name,rclcpp::Node::SharedPtr node);
extern I64 StraightLine_plan(StraightLine *self,Point *start,Point *goal,std::vector<Point> *out);
extern U0 Point_Del(Point *self);

Point *Point_New(F64 x,F64 y)
{
  Point *self=CAlloc(sizeof(Point));
  self->x_=x;
  self->y_=y;
  return self;
}

F64 Point_dist(Point *self,Point *o)
{
  return Sqrt((o->x_ - self->x_)*(o->x_ - self->x_)+(o->y_ - self->y_)*(o->y_ - self->y_));
}

F64 Point_heading(Point *self,Point *o)
{
  return Arg(o->x_ - self->x_,o->y_ - self->y_);
}

U0 Point_Del(Point *self)
{
  Free(self);
}

StraightLine *StraightLine_New()
{
  StraightLine *self=CAlloc(sizeof(StraightLine));
  self->last_=NULL;
  self->step_=0.1;
  self->count_=0;
  return self;
}

U0 StraightLine_Del(StraightLine *self)
{
  Point_Del(self->last_);
  Free(self);
}

U0 StraightLine_configure(StraightLine *self,U8 *name,rclcpp::Node::SharedPtr node) //TODO(cpp2hc): param node: unknown type 'rclcpp::Node::SharedPtr'
{
  self->name_ = name;
  self->node_ = node;
  RosLogInfo("configured %s step %f", name, self->step_);
}

I64 StraightLine_plan(StraightLine *self,Point *start,Point *goal,std::vector<Point> *out) //TODO(cpp2hc): param out: non-const reference became a pointer - callers must pass &x
{
  I64 i;
  auto d = Point_dist(start,goal); //TODO(cpp2hc): auto - write the real type
  I64 n = ToI64(d / self->step_);
  for (i = 0; i <= n; ++i) {
    out.push_back(Point(0, 0)); //TODO(cpp2hc): method call on an object of unknown type
  }
  self->last_ = Point_New(goal->x_, goal->y_);
  Print("planned %d points for %s\n",n,self->name_);
  self->count_ += 1;
  return n;
}

//pluginlib registration (was PLUGINLIB_EXPORT_CLASS).
//Runs when the file is loaded (PluginLoad); call it yourself if you #include.
U0 PluginInit_planner()
{
  PluginClassAdd("my_planners::StraightLine","nav_base::GlobalPlanner",&StraightLine_New,&StraightLine_Del);
  PluginMethodAdd("my_planners::StraightLine","configure",&StraightLine_configure);
  PluginMethodAdd("my_planners::StraightLine","plan",&StraightLine_plan);
}
//@host-skip-begin
PluginInit_planner;
//@host-skip-end
