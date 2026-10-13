//Udp: datagram endpoints for RTPS. On TempleOS they sit on our own
//IP stack (src/Net); on Linux (host tests) on BSD sockets.
//Addresses are IPv4 in host order: 127.0.0.1 = 0x7F000001.

class CRosUdp
{
  I64 fd;	//host socket, or src/Net handle
  I64 port;
};

I64 ros_net_ip=0x7F000001;	//this machine's address (RosNetSetIp)

U0 RosNetSetIp(I64 a,I64 b,I64 c,I64 d)
{
  ros_net_ip=(a<<24)|(b<<16)|(c<<8)|d;
}

//"10.0.0.2" -> 0x0A000002, 0 if bad.
I64 RosIpParse(U8 *s)
{
  CRosVec *v=RosStrSplit(s,'.');
  I64 ip=0,i,x;
  Bool ok=v->len==4;
  for (i=0;ok && i<4;i++) {
    if (!RosStrToI64(RosVecGetI64(v,i),&x) || x<0 || x>255)
      ok=FALSE;
    ip=(ip<<8)|x;
  }
  RosStrVecDel(v);
  if (!ok)
    return 0;
  return ip;
}

U8 *RosIpStr(I64 ip)
{
  return MStrPrint("%d.%d.%d.%d",(ip>>24)&255,(ip>>16)&255,(ip>>8)&255,ip&255);
}

extern I64 RosNetUdpOpen(I64 port);
extern Bool RosNetUdpJoin(I64 h,I64 group);
extern I64 RosNetUdpSend(I64 h,I64 ip,I64 port,U8 *data,I64 len);
extern I64 RosNetUdpRecv(I64 h,U8 *buf,I64 cap,I64 *ip,I64 *port);
extern U0 RosNetUdpClose(I64 h);

//Bind a UDP port (0 = any). NULL if it's taken. Multicast-capable
//ports are opened shared (several participants on one machine).
CRosUdp *RosUdpOpen(I64 port,Bool shared)
{
  CRosUdp *u;
  I64 fd=-1;
//@host-skip-begin
  fd=RosNetUdpOpen(port);
//@host-skip-end
//@host: struct sockaddr_in sa; int one=1; fd=socket(AF_INET,SOCK_DGRAM,0);
//@host: if (fd<0) return NULL;
//@host: if (shared) setsockopt(fd,SOL_SOCKET,SO_REUSEADDR,&one,sizeof one);
//@host: memset(&sa,0,sizeof sa); sa.sin_family=AF_INET; sa.sin_port=htons(port); sa.sin_addr.s_addr=htonl(INADDR_ANY);
//@host: if (bind(fd,(struct sockaddr *)&sa,sizeof sa)<0) { close(fd); return NULL; }
//@host: fcntl(fd,F_SETFL,O_NONBLOCK);
//@host: if (!port) { socklen_t l=sizeof sa; getsockname(fd,(struct sockaddr *)&sa,&l); port=ntohs(sa.sin_port); }
  if (fd<0)
    return NULL;
  u=CAlloc(sizeof(CRosUdp));
  u->fd=fd;
  u->port=port;
  return u;
}

Bool RosUdpJoin(CRosUdp *u,I64 group)
{
//@host-skip-begin
  return RosNetUdpJoin(u->fd,group);
//@host-skip-end
//@host: struct ip_mreq m; m.imr_multiaddr.s_addr=htonl(group); m.imr_interface.s_addr=htonl(ros_net_ip);
//@host: unsigned char loop=1; setsockopt(u->fd,IPPROTO_IP,IP_MULTICAST_LOOP,&loop,1);
//@host: return setsockopt(u->fd,IPPROTO_IP,IP_ADD_MEMBERSHIP,&m,sizeof m)==0;
}

I64 RosUdpSend(CRosUdp *u,I64 ip,I64 port,U8 *data,I64 len)
{
//@host-skip-begin
  return RosNetUdpSend(u->fd,ip,port,data,len);
//@host-skip-end
//@host: struct sockaddr_in sa; memset(&sa,0,sizeof sa); sa.sin_family=AF_INET;
//@host: sa.sin_port=htons(port); sa.sin_addr.s_addr=htonl(ip);
//@host: return sendto(u->fd,data,len,0,(struct sockaddr *)&sa,sizeof sa);
}

//Next datagram, or -1 if none waiting (never blocks).
I64 RosUdpRecv(CRosUdp *u,U8 *buf,I64 cap,I64 *ip,I64 *port)
{
//@host-skip-begin
  return RosNetUdpRecv(u->fd,buf,cap,ip,port);
//@host-skip-end
//@host: struct sockaddr_in sa; socklen_t l=sizeof sa;
//@host: ssize_t n=recvfrom(u->fd,buf,cap,0,(struct sockaddr *)&sa,&l);
//@host: if (n<0) return -1;
//@host: *ip=ntohl(sa.sin_addr.s_addr); *port=ntohs(sa.sin_port); return n;
}

U0 RosUdpClose(CRosUdp *u)
{
  if (!u)
    return;
//@host-skip-begin
  RosNetUdpClose(u->fd);
//@host-skip-end
//@host: close(u->fd);
  Free(u);
}
