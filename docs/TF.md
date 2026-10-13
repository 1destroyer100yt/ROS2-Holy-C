# tf: coordinate frames

`src/Tf/` does what tf2 and tf2_ros do: a tree of frames, each with a short (10 s) history of
where it sits relative to its parent, published on `/tf` and `/tf_static` with the same
messages, QoS and conventions as ROS 2.

```
CRosTfBuffer *tb=RosTfBufferNew();
RosTfListenerNew(node,tb);                      //fill it from /tf and /tf_static

CRosTf t;
if (RosTfLookup(tb,"map","base_link",0,&t))     //0 = latest common time; or a stamp in ns
  Print("robot at %f %f\n",t.x,t.y);
else
  Print("%s\n",ros_tf_error);                   //extrapolation, not connected, unknown frame

CRosTfBroadcaster *br=RosTfBroadcasterNew(node);
RosTfSend(br,"odom","base_link",RosTimeSystemNs(),&t);
CRosTfBroadcaster *sb=RosTfStaticBroadcasterNew(node);   //latched, all statics in one message
RosTfSend(sb,"base_link","laser",0,&t);

RosTfEcho("odom","base_link");                  //like: ros2 run tf2_ros tf2_echo odom base_link
```

`RosTfLookup(target,source)` returns T_target_source, the transform that maps points in `source`
into `target`, the same as tf2's `lookupTransform`. Between samples, translation is
interpolated linearly and rotation with slerp. Asking outside the history is an error, as in tf2.

Maths (`src/Tf/Math.HC`): `CRosTf` (translation + quaternion), `RosTfCompose`, `RosTfInverse`,
`RosTfApply` (point), `RosTfRotate`, `RosTfLerp`, `RosTfSetRpy` / `RosTfGetRpy`.
