//Tests for src/Viz (canvas + RvizLite).

U0 TestCanvas()
{
  CRosCanvas *cv=RosCanvasNew(32,16);
  U8 *ppm;
  I64 len;
  RosTestGroup("Canvas");
  RosCanvasClear(cv,ROS_BLUE);
  RosTestI64("clear",RosCanvasGet(cv,5,5),ROS_BLUE);
  RosCanvasLine(cv,0,0,31,15,ROS_YELLOW);
  RosTestI64("line start",RosCanvasGet(cv,0,0),ROS_YELLOW);
  RosTestI64("line end",RosCanvasGet(cv,31,15),ROS_YELLOW);
  RosTestI64("line middle",RosCanvasGet(cv,16,8),ROS_YELLOW);
  RosCanvasLine(cv,-1000,5,1000,5,ROS_RED);	//clipped
  RosTestI64("clipped line",RosCanvasGet(cv,0,5),ROS_RED);
  RosCanvasLine(cv,-5000000,0,5000000,0,ROS_RED);	//absurd: ignored, no hang
  RosCanvasRect(cv,30,14,100,100,ROS_GREEN);
  RosTestI64("rect clipped",RosCanvasGet(cv,31,15),ROS_GREEN);
  RosCanvasCircle(cv,10,10,3,ROS_WHITE,TRUE);
  RosTestI64("circle",RosCanvasGet(cv,10,12),ROS_WHITE);
  RosTestI64("outside",RosCanvasGet(cv,40,1),-1);
  RosTestI64("nearest red",RosColor16(255,0,0),ROS_RED);
  RosTestI64("nearest white",RosColor16(250,250,250),ROS_WHITE);
  RosTestI64("nearest yellow",RosColor16(255,255,0),ROS_YELLOW);
  ppm=RosCanvasPpm(cv,&len);
  RosTestTrue("ppm header",!MemCmp(ppm,"P6\n32 16\n255\n",13));
  RosTestI64("ppm size",len,13+32*16*3);
  RosTestTrue("ppm pixel (blue)",ppm[13+(2*32+20)*3]==0 && ppm[13+(2*32+20)*3+2]==0xAA);
  Free(ppm);
  RosCanvasDel(cv);
}

U0 TestRvizScene()
{
  CRosNode *viz=RosNodeNew("rviz_lite",NULL),*src=RosNodeNew("robot",NULL);
  CRosExec *e=RosExecNew();
  CRosRviz *rv;
  CRosTf t;
  CRosQoS q;
  CRosPub *pmap,*pscan,*ppath,*pmark;
  CMsg_nav_msgs_OccupancyGrid *map=RosMsgNew(ros_t_nav_msgs_OccupancyGrid);
  CMsg_sensor_msgs_LaserScan *scan=RosMsgNew(ros_t_sensor_msgs_LaserScan);
  CMsg_nav_msgs_Path *path=RosMsgNew(ros_t_nav_msgs_Path);
  CMsg_visualization_msgs_Marker *mk=RosMsgNew(ros_t_visualization_msgs_Marker);
  CMsg_geometry_msgs_PoseStamped *ps;
  U8 *cells;
  F64 *r;
  I64 i;
  RosTestGroup("RvizLite");
  RosExecAdd(e,viz);
  RosExecAdd(e,src);
  rv=RosRvizNew(viz,"map",200,200);
  RosTestI64("default scale",ToI64(rv->scale),40);
  RosRvizAdd(rv,ROS_RVIZ_MAP,"/map");
  RosRvizAdd(rv,ROS_RVIZ_SCAN,"/scan");
  RosRvizAdd(rv,ROS_RVIZ_PATH,"/plan");
  RosRvizAdd(rv,ROS_RVIZ_MARKER,"/visualization_marker");
  RosTestTrue("bad kind",RosRvizAdd(rv,99,"/x")==NULL);
  //tf: map -> odom -> base_link (1,0) facing +y -> laser 0.2 ahead
  RosTfIdentity(&t);
  RosTfSet(rv->tf,"map","odom",0,&t,TRUE);
  RosTfSetRpy(&t,0.0,0.0,pi/2.0);
  t.x=1.0;
  RosTfSet(rv->tf,"odom","base_link",0,&t,TRUE);
  RosTfIdentity(&t);
  t.x=0.2;
  RosTfSet(rv->tf,"base_link","laser",0,&t,TRUE);
  //a 1 m map of 10 cm cells centered on the origin: free, the top-right
  //cell occupied, the bottom row unknown
  RosQoSTransientLocal(&q,1);
  pmap=RosPubNew(src,"/map",ros_t_nav_msgs_OccupancyGrid,&q);
  Free(map->header.frame_id);
  map->header.frame_id=StrNew("map");
  map->info.resolution=0.1;
  map->info.width=10;
  map->info.height=10;
  map->info.origin.position.x=-0.5;
  map->info.origin.position.y=-0.5;
  map->info.origin.orientation.w=1.0;
  RosSeqResize(RosTypeField(ros_t_nav_msgs_OccupancyGrid,"data"),map,100);
  cells=map->data.data;
  for (i=0;i<100;i++)
    cells[i]=0;
  for (i=0;i<10;i++)
    cells[i]=255;	//-1: unknown
  cells[99]=100;
  RosPublish(pmap,map);
  //one ray straight ahead of the laser, 1 m; one at infinity
  RosQoSSensorData(&q);
  pscan=RosPubNew(src,"/scan",ros_t_sensor_msgs_LaserScan,&q);
  Free(scan->header.frame_id);
  scan->header.frame_id=StrNew("laser");
  scan->angle_min=0.0;
  scan->angle_increment=0.1;
  scan->range_min=0.1;
  scan->range_max=10.0;
  RosSeqResize(RosTypeField(ros_t_sensor_msgs_LaserScan,"ranges"),scan,2);
  r=scan->ranges.data;
  r[0]=1.0;
  r[1]=100.0;
  RosPublish(pscan,scan);
  //a path straight down from the origin
  ppath=RosPubNew(src,"/plan",ros_t_nav_msgs_Path,NULL);
  Free(path->header.frame_id);
  path->header.frame_id=StrNew("map");
  RosSeqResize(RosTypeField(ros_t_nav_msgs_Path,"poses"),path,2);
  ps=path->poses.data;
  ps[1].pose.position.y=-2.0;
  RosPublish(ppath,path);
  //a red sphere at (-1,-1), 0.5 m across
  pmark=RosPubNew(src,"/visualization_marker",ros_t_visualization_msgs_Marker,NULL);
  Free(mk->header.frame_id);
  mk->header.frame_id=StrNew("map");
  mk->type=ROS_visualization_msgs_Marker_SPHERE;
  mk->pose.position.x=-1.0;
  mk->pose.position.y=-1.0;
  mk->pose.orientation.w=1.0;
  mk->scale.x=0.5;
  mk->color.r=1.0;
  mk->color.a=1.0;
  RosPublish(pmark,mk);
  for (i=0;i<5;i++)
    RosExecSpinSome(e);
  RosRvizRender(rv);
  //canvas center (100,100) is map (0,0); 40 px per meter, y up
  RosTestI64("map: free cell",RosCanvasGet(rv->cv,104,104),ROS_LTGRAY);
  RosTestI64("map: occupied cell",RosCanvasGet(rv->cv,118,82),ROS_BLACK);
  RosTestI64("map: unknown row",RosCanvasGet(rv->cv,104,118),ROS_DKGRAY);
  RosTestI64("outside the map: background",RosCanvasGet(rv->cv,5,5),ROS_BLACK);
  RosTestI64("grid line at x=2 m",RosCanvasGet(rv->cv,180,30),ROS_DKGRAY);
  RosTestI64("scan point at (1,1.2)",RosCanvasGet(rv->cv,140,52),ROS_LTRED);
  RosTestI64("path",RosCanvasGet(rv->cv,100,150),ROS_LTGREEN);
  RosTestI64("marker sphere",RosCanvasGet(rv->cv,60,140),ROS_RED);
  RosTestI64("marker sphere edge",RosCanvasGet(rv->cv,68,140),ROS_RED);
  RosTestI64("base_link y axis points -x",RosCanvasGet(rv->cv,134,100),ROS_GREEN);
  RosTestTrue("drawn count",rv->drawn>=1+1+1+1+3);
  //pan and zoom
  rv->cx=1.0;
  rv->scale=80.0;
  RosRvizRender(rv);
  RosTestI64("zoomed scan point",RosCanvasGet(rv->cv,100,4),ROS_LTRED);
  //delete the marker
  rv->cx=0.0;
  rv->scale=40.0;
  mk->action=ROS_visualization_msgs_Marker_DELETE;
  RosPublish(pmark,mk);
  RosExecSpinSome(e);
  RosRvizRender(rv);
  RosTestTrue("marker deleted",RosCanvasGet(rv->cv,60,140)!=ROS_RED);
//@host: if (getenv("RVIZ_PPM")) RosCanvasSavePpm(rv->cv,(U8 *)getenv("RVIZ_PPM"));
  RosRvizDel(rv);
  RosMsgDel(ros_t_nav_msgs_OccupancyGrid,map);
  RosMsgDel(ros_t_sensor_msgs_LaserScan,scan);
  RosMsgDel(ros_t_nav_msgs_Path,path);
  RosMsgDel(ros_t_visualization_msgs_Marker,mk);
  RosNodeDel(viz);
  RosNodeDel(src);
  RosExecDel(e);
}

U0 TestRviz()
{
  TestCanvas();
  TestRvizScene();
}
