//Tap: a Linux TAP device as the link under our IP stack, so the stack
//can be tested against the real Linux kernel (host tests only; on
//TempleOS the link is E1000.HC or Slip.HC).

//@host: #include <linux/if_tun.h>
//@host: #include <net/if.h>
//@host: #include <sys/ioctl.h>

class CRosTap
{
  I64 fd;
};

Bool RosTapTx(CRosTap *t,U8 *frame,I64 len)
{
//@host: return write(t->fd,frame,len)==len;
  return FALSE;
}

I64 RosTapRx(CRosTap *t,U8 *buf,I64 cap)
{
//@host: ssize_t n=read(t->fd,buf,cap); if (n<=0) return -1; return n;
  return -1;
}

//Make TAP interface name; Linux gets host_ip/24 on it. Our stack
//gets a random MAC. Returns the link, NULL if it can't (not root?).
CRosNetIf *RosTapOpen(U8 *name,I64 host_ip)
{
  CRosNetIf *nif=CAlloc(sizeof(CRosNetIf));
  CRosTap *t=CAlloc(sizeof(CRosTap));
  I64 i,r=RosTimeSteadyNs();
//@host: struct ifreq ifr; int s; struct sockaddr_in *sa;
//@host: t->fd=open("/dev/net/tun",O_RDWR|O_NONBLOCK);
//@host: if (t->fd<0) { Free(t); Free(nif); return NULL; }
//@host: memset(&ifr,0,sizeof ifr); ifr.ifr_flags=IFF_TAP|IFF_NO_PI; strncpy(ifr.ifr_name,(char *)name,IFNAMSIZ-1);
//@host: if (ioctl(t->fd,TUNSETIFF,&ifr)<0) { close(t->fd); Free(t); Free(nif); return NULL; }
//@host: s=socket(AF_INET,SOCK_DGRAM,0); sa=(struct sockaddr_in *)&ifr.ifr_addr;
//@host: memset(sa,0,sizeof *sa); sa->sin_family=AF_INET; sa->sin_addr.s_addr=htonl(host_ip);
//@host: ioctl(s,SIOCSIFADDR,&ifr); sa->sin_addr.s_addr=htonl(0xFFFFFF00); ioctl(s,SIOCSIFNETMASK,&ifr);
//@host: ioctl(s,SIOCGIFFLAGS,&ifr); ifr.ifr_flags|=IFF_UP|IFF_RUNNING|IFF_MULTICAST; ioctl(s,SIOCSIFFLAGS,&ifr); close(s);
  nif->mac[0]=0x02;	//locally administered
  for (i=1;i<6;i++) {
    r=r*6364136223846793005+1442695040888963407;
    nif->mac[i]=(r>>33)&255;
  }
  nif->ethernet=TRUE;
  nif->mtu=1500;
  nif->tx=&RosTapTx;
  nif->rx=&RosTapRx;
  nif->drv=t;
  return nif;
}
