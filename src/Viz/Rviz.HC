//Rviz: RvizLite, a top-down 2D rviz. It subscribes to the usual
//topics, transforms everything into one fixed frame with tf, and
//draws into a CRosCanvas (shown in a TempleOS window by RvizLite;).
//
//  CRosRviz *rv=RosRvizNew(node,"map",640,480);
//  RosRvizAdd(rv,ROS_RVIZ_MAP,"/map");
//  RosRvizAdd(rv,ROS_RVIZ_SCAN,"/scan");
//  RosRvizAdd(rv,ROS_RVIZ_PATH,"/plan");
//  RosRvizRender(rv);              //rv->cv holds the picture
//
//Displays: grid, tf frames, LaserScan, OccupancyGrid, Path,
//PoseStamped, Odometry, Marker and MarkerArray (arrows, cubes,
//spheres, cylinders, lines, points; text and meshes are skipped).

#define ROS_RVIZ_SCAN		1	//sensor_msgs/LaserScan
#define ROS_RVIZ_MAP		2	//nav_msgs/OccupancyGrid
#define ROS_RVIZ_PATH		3	//nav_msgs/Path
#define ROS_RVIZ_POSE		4	//geometry_msgs/PoseStamped
#define ROS_RVIZ_ODOM		5	//nav_msgs/Odometry
#define ROS_RVIZ_MARKER		6	//visualization_msgs/Marker
#define ROS_RVIZ_MARKERS	7	//visualization_msgs/MarkerArray

class CRosRvizDisplay
{
  I64 kind,color;
  U8 *topic;
  CRosType *t;
  CRosSub *sub;
  U8 *last;		//newest message (owned), NULL before the first
  CRosVec *markers;	//kept CMsg_visualization_msgs_Marker * by ns/id
  I64 lock;
  U8 *rv;		//CRosRviz *
};

class CRosRviz
{
  CRosNode *node;
  CRosCanvas *cv;
  CRosTfBuffer *tf;
  CRosTfListener *tl;
  U8 *fixed;		//fixed frame
  F64 cx,cy;		//view center (fixed frame, m)
  F64 scale;		//pixels per meter
  F64 grid_step;	//m, 0 = no grid
  Bool show_tf;
  CRosVec *displays;	//CRosRvizDisplay *
  I64 drawn;		//things drawn in the last render (for tests)
};

CRosRviz *RosRvizNew(CRosNode *n,U8 *fixed_frame,I64 w,I64 h)
{
  CRosRviz *rv=CAlloc(sizeof(CRosRviz));
  rv->node=n;
  rv->cv=RosCanvasNew(w,h);
  rv->tf=RosTfBufferNew();
  rv->tl=RosTfListenerNew(n,rv->tf);
  rv->fixed=StrNew(fixed_frame);
  rv->scale=40.0;
  rv->grid_step=1.0;
  rv->show_tf=TRUE;
  rv->displays=RosVecNew(8);
  return rv;
}

U0 RosRvizMarkersFree(CRosRvizDisplay *d)
{
  I64 i;
  if (!d->markers)
    return;
  for (i=0;i<d->markers->len;i++)
    RosMsgDel(ros_t_visualization_msgs_Marker,RosVecGetI64(d->markers,i));
  RosVecClear(d->markers);
}

U0 RosRvizDel(CRosRviz *rv)
{
  I64 i;
  CRosRvizDisplay *d;
  if (!rv)
    return;
  for (i=0;i<rv->displays->len;i++) {
    d=RosVecGetI64(rv->displays,i);
    RosSubDel(d->sub);
    RosMsgDel(d->t,d->last);
    if (d->markers) {
      RosRvizMarkersFree(d);
      RosVecDel(d->markers);
    }
    Free(d->topic);
    Free(d);
  }
  RosVecDel(rv->displays);
  RosTfListenerDel(rv->tl);
  RosTfBufferDel(rv->tf);
  RosCanvasDel(rv->cv);
  Free(rv->fixed);
  Free(rv);
}

//Keep (or replace, or delete) one marker by ns + id.
U0 RosRvizMarkerKeep(CRosRvizDisplay *d,CMsg_visualization_msgs_Marker *m)
{
  I64 i;
  CMsg_visualization_msgs_Marker *k;
  if (m->action==ROS_visualization_msgs_Marker_DELETEALL) {
    RosRvizMarkersFree(d);
    return;
  }
  for (i=0;i<d->markers->len;i++) {
    k=RosVecGetI64(d->markers,i);
    if (k->id==m->id && RosStrSame(k->ns,m->ns)) {
      RosMsgDel(ros_t_visualization_msgs_Marker,k);
      RosVecRemove(d->markers,i);
      break;
    }
  }
  if (m->action!=ROS_visualization_msgs_Marker_DELETE)
    RosVecPushI64(d->markers,RosMsgDup(ros_t_visualization_msgs_Marker,m));
}

U0 RosRvizOnMsg(U8 *m,CRosRvizDisplay *d)
{
  CMsg_visualization_msgs_MarkerArray *ma;
  CMsg_visualization_msgs_Marker *mk;
  I64 i;
  RosLockGet(&d->lock);
  if (d->kind==ROS_RVIZ_MARKER)
    RosRvizMarkerKeep(d,m);
  else if (d->kind==ROS_RVIZ_MARKERS) {
    ma=m;
    mk=ma->markers.data;
    for (i=0;i<ma->markers.len;i++)
      RosRvizMarkerKeep(d,&mk[i]);
  } else {
    RosMsgDel(d->t,d->last);
    d->last=RosMsgDup(d->t,m);
  }
  RosLockPut(&d->lock);
}

//Show topic as kind. Returns the display (its color can be changed).
CRosRvizDisplay *RosRvizAdd(CRosRviz *rv,I64 kind,U8 *topic)
{
  CRosRvizDisplay *d=CAlloc(sizeof(CRosRvizDisplay));
  CRosQoS q;
  d->kind=kind;
  d->topic=StrNew(topic);
  d->rv=rv;
  RosQoSDefault(&q);
  switch (kind) {
    case ROS_RVIZ_SCAN:
      d->t=ros_t_sensor_msgs_LaserScan;
      d->color=ROS_LTRED;
      RosQoSSensorData(&q);
      break;
    case ROS_RVIZ_MAP:
      d->t=ros_t_nav_msgs_OccupancyGrid;
      RosQoSTransientLocal(&q,1);	//map servers latch the map
      break;
    case ROS_RVIZ_PATH:
      d->t=ros_t_nav_msgs_Path;
      d->color=ROS_LTGREEN;
      break;
    case ROS_RVIZ_POSE:
      d->t=ros_t_geometry_msgs_PoseStamped;
      d->color=ROS_LTPURPLE;
      break;
    case ROS_RVIZ_ODOM:
      d->t=ros_t_nav_msgs_Odometry;
      d->color=ROS_LTCYAN;
      break;
    case ROS_RVIZ_MARKER:
      d->t=ros_t_visualization_msgs_Marker;
      d->markers=RosVecNew(8);
      q.depth=100;
      break;
    case ROS_RVIZ_MARKERS:
      d->t=ros_t_visualization_msgs_MarkerArray;
      d->markers=RosVecNew(8);
      break;
    default:
      Free(d->topic);
      Free(d);
      return NULL;
  }
  d->sub=RosSubNew(rv->node,topic,d->t,&q,&RosRvizOnMsg,d);
  RosVecPushI64(rv->displays,d);
  return d;
}

//---- drawing helpers

I64 RosRvizSx(CRosRviz *rv,F64 x)
{
  return ToI64(Floor(rv->cv->w/2+(x-rv->cx)*rv->scale+0.5));
}

I64 RosRvizSy(CRosRviz *rv,F64 y)
{
  return ToI64(Floor(rv->cv->h/2-(y-rv->cy)*rv->scale+0.5));
}

//T(fixed <- frame) at stamp, or the latest if that time isn't there.
Bool RosRvizFrame(CRosRviz *rv,U8 *frame,I64 stamp,CRosTf *t)
{
  if (!frame || !frame[0] || !StrCmp(RosTfName(frame),RosTfName(rv->fixed))) {
    RosTfIdentity(t);
    return TRUE;
  }
  if (stamp && RosTfLookup(rv->tf,rv->fixed,frame,stamp,t))
    return TRUE;
  return RosTfLookup(rv->tf,rv->fixed,frame,0,t);
}

Bool RosRvizHeaderTf(CRosRviz *rv,CMsg_std_msgs_Header *h,CRosTf *t)
{
  return RosRvizFrame(rv,h->frame_id,RosTimeMake(h->stamp.sec,h->stamp.nanosec),t);
}

//A line between two points given in a frame (t = T(fixed <- frame)).
U0 RosRvizLine(CRosRviz *rv,CRosTf *t,F64 x0,F64 y0,F64 x1,F64 y1,I64 color)
{
  F64 z=0.0;
  RosTfApply(t,&x0,&y0,&z);
  z=0.0;
  RosTfApply(t,&x1,&y1,&z);
  RosCanvasLine(rv->cv,RosRvizSx(rv,x0),RosRvizSy(rv,y0),RosRvizSx(rv,x1),RosRvizSy(rv,y1),color);
}

U0 RosRvizDot(CRosRviz *rv,CRosTf *t,F64 x,F64 y,I64 r,I64 color)
{
  F64 z=0.0;
  RosTfApply(t,&x,&y,&z);
  RosCanvasCircle(rv->cv,RosRvizSx(rv,x),RosRvizSy(rv,y),r,color,TRUE);
}

//An arrow along x of pose p (in a frame t), len meters.
U0 RosRvizArrow(CRosRviz *rv,CRosTf *t,CMsg_geometry_msgs_Pose *p,F64 len,I64 color)
{
  CRosTf pt,w;
  RosTfSetXyzQ(&pt,p->position.x,p->position.y,p->position.z,p->orientation.x,
	p->orientation.y,p->orientation.z,p->orientation.w);
  RosTfCompose(&w,t,&pt);
  RosRvizLine(rv,&w,0.0,0.0,len,0.0,color);
  RosRvizLine(rv,&w,len,0.0,len*0.7,len*0.2,color);
  RosRvizLine(rv,&w,len,0.0,len*0.7,-len*0.2,color);
}

I64 RosRvizRgba(CMsg_std_msgs_ColorRGBA *c)
{
  return RosColor16(ToI64(c->r*255.0),ToI64(c->g*255.0),ToI64(c->b*255.0));
}

//---- displays

U0 RosRvizDrawGrid(CRosRviz *rv)
{
  F64 x,y,x0,x1,y0,y1;
  I64 n=0;
  if (rv->grid_step<=0.0 || rv->grid_step*rv->scale<4.0)
    return;
  x0=rv->cx-rv->cv->w/2/rv->scale;
  x1=rv->cx+rv->cv->w/2/rv->scale;
  y0=rv->cy-rv->cv->h/2/rv->scale;
  y1=rv->cy+rv->cv->h/2/rv->scale;
  x=Floor(x0/rv->grid_step)*rv->grid_step;
  while (x<=x1 && n<1000) {
    RosCanvasLine(rv->cv,RosRvizSx(rv,x),0,RosRvizSx(rv,x),rv->cv->h-1,ROS_DKGRAY);
    x+=rv->grid_step;
    n++;
  }
  y=Floor(y0/rv->grid_step)*rv->grid_step;
  while (y<=y1 && n<2000) {
    RosCanvasLine(rv->cv,0,RosRvizSy(rv,y),rv->cv->w-1,RosRvizSy(rv,y),ROS_DKGRAY);
    y+=rv->grid_step;
    n++;
  }
}

//The map: every pixel looks up its cell (works for any zoom and rotation).
U0 RosRvizDrawMap(CRosRviz *rv,CMsg_nav_msgs_OccupancyGrid *m)
{
  CRosTf t,o,full,inv;
  CMsg_geometry_msgs_Pose *p=&m->info.origin;
  I64 px,py,cx,cy,v,w=m->info.width,h=m->info.height;
  F64 x,y,z,res=m->info.resolution;
  U8 *cells=m->data.data;
  if (res<=0.0 || m->data.len<w*h || !RosRvizHeaderTf(rv,&m->header,&t))
    return;
  RosTfSetXyzQ(&o,p->position.x,p->position.y,p->position.z,p->orientation.x,p->orientation.y,
	p->orientation.z,p->orientation.w);
  RosTfCompose(&full,&t,&o);	//T(fixed <- grid origin)
  RosTfInverse(&inv,&full);
  for (py=0;py<rv->cv->h;py++)
    for (px=0;px<rv->cv->w;px++) {
      x=rv->cx+(px-rv->cv->w/2)/rv->scale;
      y=rv->cy-(py-rv->cv->h/2)/rv->scale;
      z=0.0;
      RosTfApply(&inv,&x,&y,&z);
      if (x<0.0 || y<0.0)
        goto next_rviz_1;
      cx=ToI64(x/res);
      cy=ToI64(y/res);
      if (cx>=w || cy>=h)
        goto next_rviz_1;
      v=cells[cy*w+cx];
      if (v>127)
        v-=256;	//int8
      if (v<0)
        rv->cv->px[py*rv->cv->w+px]=ROS_DKGRAY;	//unknown
      else if (v>=65)
        rv->cv->px[py*rv->cv->w+px]=ROS_BLACK;	//occupied
      else if (v<=25)
        rv->cv->px[py*rv->cv->w+px]=ROS_LTGRAY;	//free
      else
        rv->cv->px[py*rv->cv->w+px]=ROS_BROWN;	//in between
      next_rviz_1:;
    }
  rv->drawn++;
}

U0 RosRvizDrawScan(CRosRviz *rv,CMsg_sensor_msgs_LaserScan *s,I64 color)
{
  CRosTf t;
  F64 *r=s->ranges.data,a;
  I64 i;
  if (!RosRvizHeaderTf(rv,&s->header,&t))
    return;
  for (i=0;i<s->ranges.len;i++) {
    if (r[i]!=r[i] || r[i]<s->range_min || r[i]>s->range_max)
      goto next_rviz_2;	//NaN, too near, too far (inf)
    a=s->angle_min+i*s->angle_increment;
    RosRvizDot(rv,&t,r[i]*Cos(a),r[i]*Sin(a),1,color);
    rv->drawn++;
    next_rviz_2:;
  }
}

U0 RosRvizDrawPath(CRosRviz *rv,CMsg_nav_msgs_Path *p,I64 color)
{
  CRosTf t;
  CMsg_geometry_msgs_PoseStamped *ps=p->poses.data;
  I64 i;
  if (!RosRvizHeaderTf(rv,&p->header,&t))
    return;
  for (i=1;i<p->poses.len;i++) {
    RosRvizLine(rv,&t,ps[i-1].pose.position.x,ps[i-1].pose.position.y,ps[i].pose.position.x,
	ps[i].pose.position.y,color);
    rv->drawn++;
  }
}

U0 RosRvizDrawMarker(CRosRviz *rv,CMsg_visualization_msgs_Marker *m)
{
  CRosTf t,pt,w;
  CMsg_geometry_msgs_Point *pts=m->points.data;
  CMsg_std_msgs_ColorRGBA *cols=m->colors.data;
  I64 i,color=RosRvizRgba(&m->color),c,r;
  F64 sx=m->scale.x/2.0,sy=m->scale.y/2.0;
  if (!RosRvizHeaderTf(rv,&m->header,&t))
    return;
  RosTfSetXyzQ(&pt,m->pose.position.x,m->pose.position.y,m->pose.position.z,
	m->pose.orientation.x,m->pose.orientation.y,m->pose.orientation.z,m->pose.orientation.w);
  RosTfCompose(&w,&t,&pt);	//T(fixed <- marker)
  switch (m->type) {
    case ROS_visualization_msgs_Marker_ARROW:
      if (m->points.len>=2)
        RosRvizLine(rv,&w,pts[0].x,pts[0].y,pts[1].x,pts[1].y,color);
      else {
        RosRvizLine(rv,&w,0.0,0.0,m->scale.x,0.0,color);
        RosRvizLine(rv,&w,m->scale.x,0.0,m->scale.x*0.7,m->scale.x*0.2,color);
        RosRvizLine(rv,&w,m->scale.x,0.0,m->scale.x*0.7,-m->scale.x*0.2,color);
      }
      break;
    case ROS_visualization_msgs_Marker_CUBE:
      RosRvizLine(rv,&w,-sx,-sy,sx,-sy,color);
      RosRvizLine(rv,&w,sx,-sy,sx,sy,color);
      RosRvizLine(rv,&w,sx,sy,-sx,sy,color);
      RosRvizLine(rv,&w,-sx,sy,-sx,-sy,color);
      break;
    case ROS_visualization_msgs_Marker_SPHERE:
    case ROS_visualization_msgs_Marker_CYLINDER:
      r=ToI64(sx*rv->scale);
      RosRvizDot(rv,&w,0.0,0.0,r,color);
      break;
    case ROS_visualization_msgs_Marker_LINE_STRIP:
      for (i=1;i<m->points.len;i++)
        RosRvizLine(rv,&w,pts[i-1].x,pts[i-1].y,pts[i].x,pts[i].y,color);
      break;
    case ROS_visualization_msgs_Marker_LINE_LIST:
      for (i=1;i<m->points.len;i+=2)
        RosRvizLine(rv,&w,pts[i-1].x,pts[i-1].y,pts[i].x,pts[i].y,color);
      break;
    case ROS_visualization_msgs_Marker_POINTS:
    case ROS_visualization_msgs_Marker_CUBE_LIST:
    case ROS_visualization_msgs_Marker_SPHERE_LIST:
      r=ToI64(sx*rv->scale);
      for (i=0;i<m->points.len;i++) {
        c=color;
        if (i<m->colors.len)
          c=RosRvizRgba(&cols[i]);
        RosRvizDot(rv,&w,pts[i].x,pts[i].y,r,c);
      }
      break;
    default:
      return;	//text, meshes: not in 2D
  }
  rv->drawn++;
}

//Every tf frame: its x (red) and y (green) axes and a line to its parent.
U0 RosRvizDrawTf(CRosRviz *rv)
{
  I64 it=0;
  U8 *k;
  CRosTfFrame *f;
  CRosTf t,tp;
  CRosVec *names=RosVecNew(8),*parents=RosVecNew(8);
  F64 len=0.3;
  I64 i;
  RosLockGet(&rv->tf->lock);
  while (RosMapNext(rv->tf->frames,&it,&k,&f)) {
    RosVecPushI64(names,StrNew(f->name));
    RosVecPushI64(parents,StrNew(f->parent));
  }
  RosLockPut(&rv->tf->lock);
  for (i=0;i<names->len;i++) {
    if (!RosRvizFrame(rv,RosVecGetI64(names,i),0,&t))
      goto next_rviz_3;
    if (RosRvizFrame(rv,RosVecGetI64(parents,i),0,&tp))
      RosCanvasLine(rv->cv,RosRvizSx(rv,t.x),RosRvizSy(rv,t.y),RosRvizSx(rv,tp.x),
	RosRvizSy(rv,tp.y),ROS_YELLOW);
    RosRvizLine(rv,&t,0.0,0.0,len,0.0,ROS_RED);
    RosRvizLine(rv,&t,0.0,0.0,0.0,len,ROS_GREEN);
    rv->drawn++;
    next_rviz_3:;
  }
  RosStrVecDel(names);
  RosStrVecDel(parents);
}

U0 RosRvizDrawDisplay(CRosRviz *rv,CRosRvizDisplay *d)
{
  CMsg_geometry_msgs_PoseStamped *ps;
  CMsg_nav_msgs_Odometry *od;
  CRosTf t;
  I64 j;
  RosLockGet(&d->lock);
  if (d->markers)
    for (j=0;j<d->markers->len;j++)
      RosRvizDrawMarker(rv,RosVecGetI64(d->markers,j));
  else if (d->last)
    switch (d->kind) {
      case ROS_RVIZ_MAP:
        RosRvizDrawMap(rv,d->last);
        break;
      case ROS_RVIZ_SCAN:
        RosRvizDrawScan(rv,d->last,d->color);
        break;
      case ROS_RVIZ_PATH:
        RosRvizDrawPath(rv,d->last,d->color);
        break;
      case ROS_RVIZ_POSE:
        ps=d->last;
        if (RosRvizHeaderTf(rv,&ps->header,&t)) {
          RosRvizArrow(rv,&t,&ps->pose,0.5,d->color);
          rv->drawn++;
        }
        break;
      case ROS_RVIZ_ODOM:
        od=d->last;
        if (RosRvizHeaderTf(rv,&od->header,&t)) {
          RosRvizArrow(rv,&t,&od->pose.pose,0.5,d->color);
          rv->drawn++;
        }
        break;
    }
  RosLockPut(&d->lock);
}

//Draw everything into rv->cv: maps, the grid, the rest, tf frames.
U0 RosRvizRender(CRosRviz *rv)
{
  I64 i;
  CRosRvizDisplay *d;
  rv->drawn=0;
  RosCanvasClear(rv->cv,ROS_BLACK);
  for (i=0;i<rv->displays->len;i++) {
    d=RosVecGetI64(rv->displays,i);
    if (d->kind==ROS_RVIZ_MAP)
      RosRvizDrawDisplay(rv,d);
  }
  RosRvizDrawGrid(rv);
  for (i=0;i<rv->displays->len;i++) {
    d=RosVecGetI64(rv->displays,i);
    if (d->kind!=ROS_RVIZ_MAP)
      RosRvizDrawDisplay(rv,d);
  }
  if (rv->show_tf)
    RosRvizDrawTf(rv);
}

//@host-skip-begin
CDC *ros_rviz_img=NULL;	//the last frame, as a TempleOS bitmap

U0 RosRvizDrawIt(CTask *task,CDC *dc)
{
  if (ros_rviz_img)
    GrBlot(dc,0,0,ros_rviz_img);	//one blit (GrPlot per pixel ran at 1 FPS)
}

//Copy the canvas into the bitmap the window draws: both are 16-color numbers.
U0 RosRvizToImg(CRosRviz *rv,CDC *img)
{
  I64 y;
  for (y=0;y<rv->cv->h;y++)
    MemCpy(img->body+y*img->width_internal,rv->cv->px+y*rv->cv->w,rv->cv->w);
}

//RvizLite in this task's window, showing the usual topics in frame
//fixed: /map /scan /plan /odom /goal_pose /visualization_marker(_array)
//and tf. Arrow keys pan, + and - zoom, g toggles the grid, Esc quits.
U0 RvizLite(U8 *fixed)
{
  CRosNode *n=RosNodeNew("rviz_lite",NULL);
  CRosRviz *rv=RosRvizNew(n,fixed,TEXT_COLS*8,(TEXT_ROWS-2)*8);
  I64 ch,sc,next=0;
  F64 step;
  CDC *img;
  RosRvizAdd(rv,ROS_RVIZ_MAP,"/map");
  RosRvizAdd(rv,ROS_RVIZ_SCAN,"/scan");
  RosRvizAdd(rv,ROS_RVIZ_PATH,"/plan");
  RosRvizAdd(rv,ROS_RVIZ_ODOM,"/odom");
  RosRvizAdd(rv,ROS_RVIZ_POSE,"/goal_pose");
  RosRvizAdd(rv,ROS_RVIZ_MARKER,"/visualization_marker");
  RosRvizAdd(rv,ROS_RVIZ_MARKERS,"/visualization_marker_array");
  img=DCNew(rv->cv->w,rv->cv->h);
  img->flags|=DCF_NO_TRANSPARENTS;
  ros_ok=TRUE;
  ros_rviz_img=img;
  DocClear;
  Fs->draw_it=&RosRvizDrawIt;
  while (ros_ok) {
    if (ScanKey(&ch,&sc)) {
      step=100.0/rv->scale;
      switch (sc.u8[0]) {
        case SC_CURSOR_UP:    rv->cy+=step; break;
        case SC_CURSOR_DOWN:  rv->cy-=step; break;
        case SC_CURSOR_LEFT:  rv->cx-=step; break;
        case SC_CURSOR_RIGHT: rv->cx+=step; break;
      }
      if (ch=='+' || ch=='=')
        rv->scale*=1.25;
      if (ch=='-')
        rv->scale/=1.25;
      if (ch=='g')
        rv->grid_step=1.0-rv->grid_step;
      if (ch==CH_ESC || ch==CH_SHIFT_ESC)
        break;
    }
    RosExecSpinOnce(RosNodeExec(n),10*ROS_NS_PER_MS);
    if (RosTimeSteadyNs()>=next) {	//10 frames a second
      RosRvizRender(rv);
      RosRvizToImg(rv,img);
      next=RosTimeSteadyNs()+100*ROS_NS_PER_MS;
    }
  }
  Fs->draw_it=NULL;
  ros_rviz_img=NULL;
  DCDel(img);
  RosRvizDel(rv);
  RosExecDel(n->exec);
  RosNodeDel(n);
}
//@host-skip-end
