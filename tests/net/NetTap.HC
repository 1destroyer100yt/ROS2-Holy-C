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
