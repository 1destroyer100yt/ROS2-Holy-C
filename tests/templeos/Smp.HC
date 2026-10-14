//Timing on each core of a real TempleOS (tools/templeos/run.py with
//CPUS=2): how long 100 RosSleepNs(2 ms) take on core 0 and on core 1.

Cd(__DIR__);;
#include "../../Load.HC"

I64 smp_took[8];
I64 smp_done=0;

U0 SmpSleeper(I64 cpu)
{
  I64 i,t0=RosTimeSteadyNs();
  for (i=0;i<100;i++)
    RosSleepNs(2*ROS_NS_PER_MS);
  smp_took[cpu]=RosTimeSteadyNs()-t0;
  LBts(&smp_done,cpu);
}

U0 SmpProbe()
{
  I64 cpu;
  Print("mp_cnt=%d\n",mp_cnt);
  for (cpu=0;cpu<mp_cnt;cpu++) {
    smp_done=0;
    Spawn(&SmpSleeper,cpu,"SmpSleeper",cpu);
    while (!Bt(&smp_done,cpu))
      Sleep(10);
    Print("core %d: 100 x 2 ms sleeps took %d ms\n",cpu,smp_took[cpu]/ROS_NS_PER_MS);
  }
}

Cd(__DIR__);;
#include "../../src/Core/Test.HC"
Cd(__DIR__);;
#include "../TestExecMT.HC"
