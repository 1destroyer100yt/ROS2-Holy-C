//Converted by cpp2hc from: regular_polygon.hpp, polygons.cpp
//Needs ROS2-Holy-C loaded first (src/Load.HC).
//Search for TODO(cpp2hc) and fix every one before trusting this file.

class RegularPolygon
{
  I64 cpp2hc_empty;
};

class Square:RegularPolygon
{
  F64 side_length_;
  I64 calls_;
};

class Triangle:RegularPolygon
{
  F64 side_length_;
};

extern U0 RegularPolygon_Del(RegularPolygon *self);
extern RegularPolygon *RegularPolygon_New();
extern U0 Square_initialize(Square *self,F64 side_length);
extern F64 Square_area(Square *self);
extern U0 Square_describe(Square *self);
extern U0 Triangle_initialize(Triangle *self,F64 side_length);
extern F64 Triangle_area(Triangle *self);
extern F64 Triangle_getHeight(Triangle *self);
extern Square *Square_New();
extern Triangle *Triangle_New();
extern U0 Square_Del(Square *self);
extern U0 Triangle_Del(Triangle *self);

//RegularPolygon::initialize is pure virtual: implement it in a subclass.
//RegularPolygon::area is pure virtual: implement it in a subclass.
U0 RegularPolygon_Del(RegularPolygon *self)
{
  Free(self);
}

RegularPolygon *RegularPolygon_New()
{
  RegularPolygon *self=CAlloc(sizeof(RegularPolygon));
  return self;
}

U0 Square_initialize(Square *self,F64 side_length)
{
  self->side_length_ = side_length;
  self->calls_ = 0;
}

F64 Square_area(Square *self)
{
  self->calls_++;
  return self->side_length_ * self->side_length_;
}

U0 Square_describe(Square *self)
{
  Print("Square with side %f area %f\n",self->side_length_,Square_area(self));
}

Square *Square_New()
{
  Square *self=CAlloc(sizeof(Square));
  self->side_length_=1.0;
  return self;
}

U0 Square_Del(Square *self)
{
  Free(self);
}

U0 Triangle_initialize(Triangle *self,F64 side_length)
{
  self->side_length_ = side_length;
}

F64 Triangle_area(Triangle *self)
{
  return 0.5 * self->side_length_ * Triangle_getHeight(self);
}

F64 Triangle_getHeight(Triangle *self)
{
  I64 i;
  F64 sum = 0.0;
  for (i = 0; i < 3; i++) {
    sum += 1.0;
  }
  Print("height for side %f (%d sides)\n", self->side_length_, ToI64(3));
  return Sqrt((self->side_length_ * self->side_length_) - ((self->side_length_ / 2) * (self->side_length_ / 2)));
}

Triangle *Triangle_New()
{
  Triangle *self=CAlloc(sizeof(Triangle));
  return self;
}

U0 Triangle_Del(Triangle *self)
{
  Free(self);
}

//pluginlib registration (was PLUGINLIB_EXPORT_CLASS)
PluginClassAdd("polygon_plugins::Square","polygon_base::RegularPolygon",&Square_New,&Square_Del);
PluginMethodAdd("polygon_plugins::Square","initialize",&Square_initialize);
PluginMethodAdd("polygon_plugins::Square","area",&Square_area);
PluginMethodAdd("polygon_plugins::Square","describe",&Square_describe);
PluginClassAdd("polygon_plugins::Triangle","polygon_base::RegularPolygon",&Triangle_New,&Triangle_Del);
PluginMethodAdd("polygon_plugins::Triangle","initialize",&Triangle_initialize);
PluginMethodAdd("polygon_plugins::Triangle","area",&Triangle_area);
PluginMethodAdd("polygon_plugins::Triangle","getHeight",&Triangle_getHeight);
