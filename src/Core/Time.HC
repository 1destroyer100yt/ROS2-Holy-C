//Time: everything is I64 nanoseconds. That covers +-292 years, so
//no struct juggling. Convert to sec/nsec only for messages.
//
//Clocks (like rclcpp):
//  ROS_CLOCK_STEADY  monotonic, from boot, never jumps
//  ROS_CLOCK_SYSTEM  wall clock: steady + an epoch offset you set
//  ROS_CLOCK_ROS     sim time if RosClockSetSim was used, else system

#define ROS_NS_PER_SEC	1000000000
#define ROS_NS_PER_MS	1000000

#define ROS_CLOCK_STEADY	0
#define ROS_CLOCK_SYSTEM	1
#define ROS_CLOCK_ROS		2

I64 ros_epoch_offset_ns=0;	//unix time (ns) at boot, 0 until set
Bool ros_sim_time_on=FALSE;
I64 ros_sim_time_ns=0;

//Monotonic nanoseconds since boot.
I64 RosTimeSteadyNs()
{
//@host-skip-begin
  //TSC ticks -> ns. F64 keeps full precision for ~1 month of uptime
  //at 3 GHz (TSC < 2^53); good enough until we find a better source.
  //VERIFY on a real install: GetTSC and cnts.time_stamp_freq.
  return ToI64(ToF64(GetTSC)*1000000000.0/cnts.time_stamp_freq);
//@host-skip-end
//@host: return HostNowNs();
}

//Tell the system clock what unix time it is now (from the network,
//a serial bridge, or by hand).
U0 RosTimeSetSystemNs(I64 unix_ns)
{
  ros_epoch_offset_ns=unix_ns-RosTimeSteadyNs();
}

I64 RosTimeSystemNs()
{
  return RosTimeSteadyNs()+ros_epoch_offset_ns;
}

//Simulation time (what /clock would drive).
U0 RosClockSetSim(I64 ns)
{
  ros_sim_time_on=TRUE;
  ros_sim_time_ns=ns;
}

U0 RosClockSimOff()
{
  ros_sim_time_on=FALSE;
}

I64 RosClockNow(I64 clock)
{
  if (clock==ROS_CLOCK_STEADY)
    return RosTimeSteadyNs();
  if (clock==ROS_CLOCK_ROS && ros_sim_time_on)
    return ros_sim_time_ns;
  return RosTimeSystemNs();
}

//Split for builtin_interfaces/Time: sec is floor(ns/1e9), nanosec in
//[0,1e9) even for negative times.
I64 RosTimeSec(I64 ns)
{
  I64 s=ns/ROS_NS_PER_SEC;
  if (ns%ROS_NS_PER_SEC<0)
    s--;
  return s;
}

I64 RosTimeNsec(I64 ns)
{
  I64 r=ns%ROS_NS_PER_SEC;
  if (r<0)
    r+=ROS_NS_PER_SEC;
  return r;
}

I64 RosTimeMake(I64 sec,I64 nsec)
{
  return sec*ROS_NS_PER_SEC+nsec;
}

F64 RosTimeToSec(I64 ns)
{
  return ToF64(ns)/1000000000.0;
}

I64 RosTimeFromSec(F64 s)
{
  return ToI64(s*1000000000.0);
}

//Sleep for ns. Gives the CPU away while waiting.
U0 RosSleepNs(I64 ns)
{
  I64 end=RosTimeSteadyNs()+ns;
  if (ns>=2*ROS_NS_PER_MS)
    Sleep(ns/ROS_NS_PER_MS-1);
  while (RosTimeSteadyNs()<end)
    Yield();
}

//Rate: keep a loop running at a fixed frequency.
//  CRosRate *r=RosRateNew(10.0);
//  while (running) { ...; RosRateSleep(r); }
class CRosRate
{
  I64 period_ns,next_ns;
  I64 missed;	//cycles that overran their slot
};

CRosRate *RosRateNew(F64 hz)
{
  CRosRate *r=CAlloc(sizeof(CRosRate));
  r->period_ns=ToI64(1000000000.0/hz);
  r->next_ns=RosTimeSteadyNs()+r->period_ns;
  return r;
}

U0 RosRateDel(CRosRate *r)
{
  Free(r);
}

//Sleep until the next tick. If we're late, don't try to catch up:
//count it and restart the schedule from now.
U0 RosRateSleep(CRosRate *r)
{
  I64 now=RosTimeSteadyNs();
  if (now<r->next_ns)
    RosSleepNs(r->next_ns-now);
  else {
    r->missed++;
    r->next_ns=now;
  }
  r->next_ns+=r->period_ns;
}
