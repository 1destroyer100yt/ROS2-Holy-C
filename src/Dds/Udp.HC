//Udp: datagram endpoints for RTPS. On TempleOS they sit on our own
//IP stack (src/Net); on Linux (host tests) on BSD sockets.
//Addresses are IPv4 in host order: 127.0.0.1 = 0x7F000001.

class CRosUdp
{
  I64 fd;	//host socket, or src/Net handle
  I64 port;
};

//Bind a UDP port (0 = any). NULL if it's taken. Multicast-capable
//ports are opened shared (several participants on one machine).
//Uses our IP stack (src/Net) when it's up, which it always is on
//TempleOS; on Linux it can also use the host's sockets.
CRosUdp *RosUdpOpen(I64 port,Bool shared)
{
  CRosUdp *u;
  I64 fd=-1,i;
  if (ros_net_stack) {
    if (!shared)	//our stack shares ports freely: check by hand
      for (i=0;i<NET_SOCK_N;i++)
        if (ros_socks[i].used && ros_socks[i].port==port && port)
          return NULL;
    fd=RosNetUdpOpen(port);
    if (fd<0)
      return NULL;
    u=CAlloc(sizeof(CRosUdp));
    u->fd=fd;
    u->port=RosNetUdpPort(fd);
    return u;
  }
//@host: struct sockaddr_in sa; int one=1; fd=socket(AF_INET,SOCK_DGRAM,0);
//@host: if (fd<0) return NULL;
//@host: if (shared) setsockopt(fd,SOL_SOCKET,SO_REUSEADDR,&one,sizeof one);
//@host: memset(&sa,0,sizeof sa); sa.sin_family=AF_INET; sa.sin_port=htons(port); sa.sin_addr.s_addr=htonl(INADDR_ANY);
//@host: if (bind(fd,(struct sockaddr *)&sa,sizeof sa)<0) { close(fd); return NULL; }
//@host: fcntl(fd,F_SETFL,O_NONBLOCK);
//@host: if (!port) { socklen_t l=sizeof sa; getsockname(fd,(struct sockaddr *)&sa,&l); port=ntohs(sa.sin_port); }
  if (fd<0)		//TempleOS without RosNetUp: no network
    return NULL;
  u=CAlloc(sizeof(CRosUdp));
  u->fd=fd;
  u->port=port;
  return u;
}

Bool RosUdpJoin(CRosUdp *u,I64 group)
{
  if (ros_net_stack)
    return RosNetUdpJoin(u->fd,group);
//@host: struct ip_mreq m; m.imr_multiaddr.s_addr=htonl(group); m.imr_interface.s_addr=htonl(ros_net_ip);
//@host: unsigned char loop=1; setsockopt(u->fd,IPPROTO_IP,IP_MULTICAST_LOOP,&loop,1);
//@host: return setsockopt(u->fd,IPPROTO_IP,IP_ADD_MEMBERSHIP,&m,sizeof m)==0;
}

I64 RosUdpSend(CRosUdp *u,I64 ip,I64 port,U8 *data,I64 len)
{
  if (ros_net_stack)
    return RosNetUdpSend(u->fd,ip,port,data,len);
//@host: struct sockaddr_in sa; memset(&sa,0,sizeof sa); sa.sin_family=AF_INET;
//@host: sa.sin_port=htons(port); sa.sin_addr.s_addr=htonl(ip);
//@host: return sendto(u->fd,data,len,0,(struct sockaddr *)&sa,sizeof sa);
}

//Next datagram, or -1 if none waiting (never blocks).
I64 RosUdpRecv(CRosUdp *u,U8 *buf,I64 cap,I64 *ip,I64 *port)
{
  if (ros_net_stack)
    return RosNetUdpRecv(u->fd,buf,cap,ip,port);
//@host: struct sockaddr_in sa; socklen_t l=sizeof sa;
//@host: ssize_t n=recvfrom(u->fd,buf,cap,0,(struct sockaddr *)&sa,&l);
//@host: if (n<0) return -1;
//@host: *ip=ntohl(sa.sin_addr.s_addr); *port=ntohs(sa.sin_port); return n;
}

U0 RosUdpClose(CRosUdp *u)
{
  if (!u)
    return;
  if (ros_net_stack)
    RosNetUdpClose(u->fd);
//@host: else close(u->fd);
  Free(u);
}
