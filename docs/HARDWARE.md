# Hardware: serial ports, motor controllers, a robot base

## Serial ports (`src/Core/Serial.HC`)
`RosSerialOpenCom(0x3F8,38400)` opens a 16550 UART (COM1; 0x2F8 is COM2) at 8N1, polled.
`RosSerialWrite`, `RosSerialRead(s,buf,cap,wait_ns)`, `RosSerialFlush`. On Linux (tests)
`RosSerialOpenFd(fd)` wraps an open file descriptor instead.

## RoboClaw (`src/Hw/RoboClaw.HC`)
Basicmicro RoboClaw controllers in packet serial mode (address 0x80 by default):
```
CRosRoboClaw *rc=RoboClawOpen(RosSerialOpenCom(0x2F8,38400),0x80);
RoboClawSpeedM1M2(rc,1000,-1000);   //encoder counts per second, signed
RoboClawDutyM1M2(rc,16000,16000);   //no encoders: -32767..32767
RoboClawEncoders(rc,&m1,&m2);
U8 *v=RoboClawVersion(rc);
```
Packets carry a CRC16 and are retried up to 3 times if the controller doesn't acknowledge.

## A differential drive base (`src/Hw/DiffDrive.HC`)
Like ros2_control's diff_drive_controller:
```
CRosNode *n=RosNodeNew("base",NULL);
CRosDiffDrive *dd=RosDiffDriveNew(n,rc);    //or NULL: simulated wheels, no hardware
RosSpin(n);
```
It subscribes `cmd_vel` (geometry_msgs/Twist), drives M1 (left) and M2 (right), reads the
encoders, and publishes `odom` (nav_msgs/Odometry) and `odom -> base_link` on `/tf`.
Parameters: `wheel_radius` (0.05 m), `wheel_separation` (0.3 m), `ticks_per_rev` (1000),
`cmd_timeout` (0.5 s: stops when cmd_vel goes quiet), `rate` (20 Hz), `odom_frame`,
`base_frame`, `publish_tf`. Put them in a params YAML or a launch file.
`RvizLite("odom")` shows where it thinks it is.

## How it's tested
`tools/hwtest/roboclaw_sim.py` simulates a RoboClaw. Before the test uses it, the official
Basicmicro Python library drives it (speed, encoders, version, reset), so the simulator
speaks the real protocol. Then the HolyC DiffDrive drives "0.5 m forward, then 90 degrees
left" and has to end up at x = 0.5 m, yaw = 90 degrees from the encoder counts:
- `tools/hwtest/run.sh`: over a pty
- `tools/hwtest/run.sh com`: over COM2 of a QEMU guest, where the HolyC drives the emulated
  16550's I/O ports itself, the same code TempleOS runs
