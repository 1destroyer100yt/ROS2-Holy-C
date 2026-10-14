//On-screen checks for real TempleOS (tools/templeos/hw.py): TurtleSim and
//RvizLite draw in the shell's window; hw.py drives them with keys over
//QEMU and reads the pixels of screenshots.

#include "../../Load.HC"

//@host-skip-begin
//A latched 4 m x 4 m map (a wall around it, a block in the middle) and a
//static transform map -> base_link at (1,0), then RvizLite until Esc.
U0 ScreenRviz()
{
  CRosNode *src=RosNodeNew("screen_map",NULL);
  CRosQoS q;
  CRosPub *pmap;
  CRosTfBroadcaster *br;
  CRosTf t;
  CMsg_nav_msgs_OccupancyGrid *map=RosMsgNew(ros_t_nav_msgs_OccupancyGrid);
  U8 *cells;
  I64 x,y,c;
  RosQoSTransientLocal(&q,1);
  pmap=RosPubNew(src,"/map",ros_t_nav_msgs_OccupancyGrid,&q);
  Free(map->header.frame_id);
  map->header.frame_id=StrNew("map");
  map->info.resolution=0.1;
  map->info.width=40;
  map->info.height=40;
  map->info.origin.position.x=-2.0;
  map->info.origin.position.y=-2.0;
  map->info.origin.orientation.w=1.0;
  RosSeqResize(RosTypeField(ros_t_nav_msgs_OccupancyGrid,"data"),map,1600);
  cells=map->data.data;
  for (y=0;y<40;y++)
    for (x=0;x<40;x++) {
      c=0;
      if (x==0 || y==0 || x==39 || y==39 || (x>=16 && x<24 && y>=16 && y<24))
        c=100;
      cells[y*40+x]=c;
    }
  RosPublish(pmap,map);
  br=RosTfStaticBroadcasterNew(src);
  RosTfIdentity(&t);
  t.x=1.0;
  RosTfSend(br,"map","base_link",0,&t);
  RvizLite("map");
  RosTfBroadcasterDel(br);
  RosMsgDel(ros_t_nav_msgs_OccupancyGrid,map);
  RosNodeDel(src);
}
//@host-skip-end
