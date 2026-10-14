//Dhcp: get an address from the network's DHCP server (RFC 2131):
//DISCOVER -> OFFER -> REQUEST -> ACK, then RosNetConfig with the
//address, netmask and router it gave us.
//
//  RosNetUp(nif);
//  if (RosNetDhcp(10*ROS_NS_PER_SEC)) Print("got %s\n",RosIpStr(ros_net_ip));
//
//Ethernet links only (SLIP has no DHCP). The lease isn't renewed:
//ros_dhcp_lease_s says how long the address is good for.

#define DHCP_DISCOVER	1
#define DHCP_OFFER	2
#define DHCP_REQUEST	3
#define DHCP_ACK	5
#define DHCP_NAK	6

I64 ros_dhcp_server=0,ros_dhcp_lease_s=0,ros_dhcp_dns=0;

//A BOOTP request with our MAC, broadcast flag set (so the answer
//reaches us before we have an address). Options go after byte 240.
I64 DhcpPacket(U8 *p,I64 xid,I64 type,I64 want_ip,I64 server)
{
  I64 n=240;
  MemSet(p,0,548);
  p[0]=1;		//BOOTREQUEST
  p[1]=1;		//Ethernet
  p[2]=6;
  NetPut32(p+4,xid);
  NetPut16(p+10,0x8000);	//broadcast
  MemCpy(p+28,ros_netif->mac,6);
  p[236]=99;		//magic cookie
  p[237]=130;
  p[238]=83;
  p[239]=99;
  p[n++]=53;		//message type
  p[n++]=1;
  p[n++]=type;
  p[n++]=61;		//client id: hardware type + MAC
  p[n++]=7;
  p[n++]=1;
  MemCpy(p+n,ros_netif->mac,6);
  n+=6;
  if (want_ip) {
    p[n++]=50;		//requested address
    p[n++]=4;
    NetPut32(p+n,want_ip);
    n+=4;
  }
  if (server) {
    p[n++]=54;		//server id
    p[n++]=4;
    NetPut32(p+n,server);
    n+=4;
  }
  p[n++]=55;		//we want: netmask, router, DNS, lease, server id
  p[n++]=5;
  p[n++]=1;
  p[n++]=3;
  p[n++]=6;
  p[n++]=51;
  p[n++]=54;
  p[n++]=12;		//host name
  p[n++]=6;
  MemCpy(p+n,"holyc0",6);
  n+=6;
  p[n++]=255;
  if (n<300)
    n=300;		//old servers want the full BOOTP size
  return n;
}

//Find option code in a reply. Pointer to its data (length in *len) or NULL.
U8 *DhcpOpt(U8 *p,I64 n,I64 code,I64 *len)
{
  I64 i=240,c,l;
  while (i<n) {
    c=p[i];
    if (c==255)
      break;
    if (!c) {	//pad
      i++;
      goto next_dhcp_1;
    }
    if (i+1>=n)
      break;
    l=p[i+1];
    if (i+2+l>n)
      break;
    if (c==code) {
      *len=l;
      return p+i+2;
    }
    i+=2+l;
    next_dhcp_1:;
  }
  return NULL;
}

//Wait for a reply to xid. Returns its message type (0 on timeout);
//the packet is in buf, its length in *n.
I64 DhcpWait(I64 h,I64 xid,U8 *buf,I64 *n,I64 until)
{
  I64 ip,port,l,len;
  U8 *t;
  while (RosTimeSteadyNs()<until) {
    len=RosNetUdpRecv(h,buf,1500,&ip,&port);
    if (len<0) {
      RosSleepNs(ROS_NS_PER_MS);
      goto next_dhcp_2;
    }
    if (len<244 || buf[0]!=2 || NetGet32(buf+4)!=xid || MemCmp(buf+28,ros_netif->mac,6) ||
	buf[236]!=99 || buf[237]!=130 || buf[238]!=83 || buf[239]!=99)
      goto next_dhcp_2;
    t=DhcpOpt(buf,len,53,&l);
    if (!t || l!=1)
      goto next_dhcp_2;
    *n=len;
    return t[0];
    next_dhcp_2:;
  }
  return 0;
}

I64 DhcpOpt32(U8 *p,I64 n,I64 code)
{
  I64 l;
  U8 *o=DhcpOpt(p,n,code,&l);
  if (!o || l<4)
    return 0;
  return NetGet32(o);
}

//Ask for an address; tries for up to timeout_ns. TRUE when configured.
Bool RosNetDhcp(I64 timeout_ns)
{
  U8 *pkt,*buf;
  I64 h,xid,n,len,type,end=RosTimeSteadyNs()+timeout_ns,offer,server,step,mask,gw;
  Bool ok=FALSE;
  if (!ros_netif || !ros_netif->ethernet)
    return FALSE;
  h=RosNetUdpOpen(68);
  if (h<0)
    return FALSE;
  pkt=MAlloc(548);
  buf=MAlloc(1600);
  ros_netif->ip=0;	//no address while asking
  ros_netif->mask=0;
  while (!ok && RosTimeSteadyNs()<end) {
    xid=RandU64()&0xFFFFFFFF;
    n=DhcpPacket(pkt,xid,DHCP_DISCOVER,0,0);
    RosNetUdpSend(h,0xFFFFFFFF,67,pkt,n);
    step=RosTimeSteadyNs()+2*ROS_NS_PER_SEC;
    if (step>end)
      step=end;
    type=DhcpWait(h,xid,buf,&len,step);
    if (type!=DHCP_OFFER)
      goto next_dhcp_3;
    offer=NetGet32(buf+16);	//yiaddr
    server=DhcpOpt32(buf,len,54);
    n=DhcpPacket(pkt,xid,DHCP_REQUEST,offer,server);
    RosNetUdpSend(h,0xFFFFFFFF,67,pkt,n);
    step=RosTimeSteadyNs()+2*ROS_NS_PER_SEC;
    if (step>end)
      step=end;
    type=DhcpWait(h,xid,buf,&len,step);
    if (type!=DHCP_ACK)
      goto next_dhcp_3;	//NAK or nothing: start over
    mask=DhcpOpt32(buf,len,1);
    if (!mask)
      mask=0xFFFFFF00;
    gw=DhcpOpt32(buf,len,3);
    ros_dhcp_server=server;
    ros_dhcp_lease_s=DhcpOpt32(buf,len,51);
    ros_dhcp_dns=DhcpOpt32(buf,len,6);
    RosNetConfig(NetGet32(buf+16),mask,gw);
    ok=TRUE;
    next_dhcp_3:;
  }
  RosNetUdpClose(h);
  Free(pkt);
  Free(buf);
  return ok;
}
