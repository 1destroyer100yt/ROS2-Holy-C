//Our IP stack on a Linux TAP device, against the real Linux kernel
//(tools/nettest/run.sh). We are 10.77.0.2, Linux is 10.77.0.1.
//A UDP echo server on port 7777; "quit" stops it.

Cd(__DIR__);;
#include "../../Load.HC"

I64 NetEcho(CRosNetIf *nif)
{
  CRosUdp *u;
  U8 *buf=MAlloc(70000);
  I64 n,ip,port,end,echoed=0;
  u=RosUdpOpen(7777,FALSE);
  Print("READY\n");
//@host: fflush(stdout);
  end=RosTimeSteadyNs()+30*ROS_NS_PER_SEC;
  while (RosTimeSteadyNs()<end) {
    n=RosUdpRecv(u,buf,70000,&ip,&port);
    if (n<0) {
      RosSleepNs(200000);
      continue;
    }
    if (n==4 && !MemCmp(buf,"quit",4))
      break;
    RosUdpSend(u,ip,port,buf,n);
    echoed++;
  }
  Print("RESULT holyc: echoed %d datagrams, rx %d frames (%d bad), tx %d\n",echoed,
	nif->rx_pkts,nif->rx_bad,nif->tx_pkts);
  RosUdpClose(u);
  Free(buf);
  return 0;
}

I64 NetTapEcho()
{
  CRosNetIf *nif=RosTapOpen("rhc0",RosIpParse("10.77.0.1"));
  if (!nif) {
    Print("RESULT holyc: can't open TAP (need root)\n");
    return 1;
  }
  RosNetUp(nif);
  RosNetConfig(RosIpParse("10.77.0.2"),RosIpParse("255.255.255.0"),0);
  return NetEcho(nif);
}

//DHCP from a server on the Linux side (tools/nettest/run.sh dhcp),
//then a datagram to it from the address we got.
I64 NetTapDhcp()
{
  CRosNetIf *nif=RosTapOpen("rhc2",RosIpParse("10.81.0.1"));
  CRosUdp *u;
  U8 *s;
  Bool ok;
  if (!nif) {
    Print("RESULT holyc: can't open TAP (need root)\n");
    return 1;
  }
  RosNetUp(nif);
  Print("READY\n");
//@host: fflush(stdout);
  ok=RosNetDhcp(20*ROS_NS_PER_SEC);
  Print("RESULT holyc: dhcp=%d ip=%s",ok,RosIpStr(ros_netif->ip));
  Print(" mask=%s",RosIpStr(ros_netif->mask));
  Print(" gw=%s lease=%d\n",RosIpStr(ros_netif->gw),ros_dhcp_lease_s);
  if (!ok)
    return 1;
  u=RosUdpOpen(0,FALSE);
  s=MStrPrint("hello from %s",RosIpStr(ros_netif->ip));
  RosUdpSend(u,RosIpParse("10.81.0.1"),9999,s,StrLen(s));
  Free(s);
  RosSleepNs(200*ROS_NS_PER_MS);
  RosNetPoll(64);
  RosUdpClose(u);
  return 0;
}

//The same over SLIP: the pty path is in the SLIP_TTY environment variable.
I64 NetSlipEcho()
{
  CRosNetIf *nif;
  I64 fd=-1;
//@host: struct termios t; fd=open(getenv("SLIP_TTY"),O_RDWR|O_NONBLOCK|O_NOCTTY);
//@host: if (fd>=0) { tcgetattr(fd,&t); cfmakeraw(&t); tcsetattr(fd,TCSANOW,&t); }
  if (fd<0) {
    Print("RESULT holyc: can't open the pty\n");
    return 1;
  }
  nif=SlipOpenFd(fd);
  RosNetUp(nif);
  RosNetConfig(RosIpParse("10.79.0.2"),RosIpParse("255.255.255.0"),0);
  return NetEcho(nif);
}

//The e1000 driver on QEMU's emulated card (tools/e1000test/run.sh).
//Runs as /init of a Linux guest: the driver talks to the card through
//I/O ports itself (iopl), Linux only provides memory. DHCP first, then
//the same echo server as over TAP.
I64 E1000Guest()
{
  CRosNetIf *nif;
  I64 r;
//@host: mkdir("/proc",0555); mount("proc","/proc","proc",0,NULL);
//@host: if (iopl(3)) { Print("RESULT holyc: iopl failed\n"); fflush(stdout); }
  nif=E1000Open();
  if (!nif) {
    Print("RESULT holyc: no e1000 found\n");
    r=1;
  } else {
    RosNetUp(nif);
    if (!RosNetDhcp(30*ROS_NS_PER_SEC)) {
      Print("RESULT holyc: no DHCP answer\n");
      r=1;
    } else {
      Print("DHCP %s\n",RosIpStr(ros_netif->ip));
      r=NetEcho(nif);
    }
  }
//@host: fflush(stdout); sync(); reboot(RB_POWER_OFF);
  return r;
}
