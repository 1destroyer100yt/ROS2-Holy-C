# RvizLite: a 2D rviz

`RvizLite("map");` opens a top-down view in the current TempleOS window. It shows the usual
topics in the fixed frame you give it, moved there with tf:

| Topic | Type | Drawn as |
|---|---|---|
| `/map` | nav_msgs/OccupancyGrid (transient local) | free light gray, occupied black, unknown dark gray |
| `/scan` | sensor_msgs/LaserScan | light red dots (NaN, inf and out-of-range rays skipped) |
| `/plan` | nav_msgs/Path | light green line |
| `/odom` | nav_msgs/Odometry | light cyan arrow |
| `/goal_pose` | geometry_msgs/PoseStamped | light purple arrow |
| `/visualization_marker(_array)` | visualization_msgs/Marker(Array) | arrows, cubes, spheres, cylinders, line strips/lists, points; text and meshes are skipped |
| tf | `/tf`, `/tf_static` | each frame's x axis red, y axis green, a yellow line to its parent |

Keys: arrows pan, `+`/`-` zoom, `g` toggles the 1 m grid, Esc quits.

## In code
```
CRosRviz *rv=RosRvizNew(node,"map",640,480);
RosRvizAdd(rv,ROS_RVIZ_SCAN,"/front/scan")->color=ROS_YELLOW;
rv->cx=2.0; rv->cy=1.0; rv->scale=80.0;   //center (m) and pixels per meter
RosRvizRender(rv);                         //rv->cv: 640x480, one color number per pixel
RosCanvasSavePpm(rv->cv,"/tmp/view.ppm");  //on Linux, to look at it
```
Everything draws into a `CRosCanvas` (`src/Viz/Canvas.HC`), a plain buffer in TempleOS's
16 colors, so the whole renderer runs in the Linux test build: `tests/TestRviz.HC` publishes
a map, a scan, a path, a marker and a tf tree and checks the pixels. Only the copy to the screen
is TempleOS-only (VERIFY on a real install; it plots pixel by pixel, writing the window's
`CDC` body directly would be faster).

Each message is transformed with the tf at its stamp, or the newest one if that time isn't
in the buffer, so a robot without a synced clock still shows up.
