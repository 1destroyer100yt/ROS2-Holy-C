//Net: a small IPv4 stack for TempleOS (which has none): Ethernet,
//ARP, IPv4 (with fragmentation and reassembly), UDP, ICMP echo and
//IGMPv2 for multicast. A link (src/Net/E1000.HC network card,
//src/Net/Slip.HC serial line, or a Linux TAP device in host tests)
//moves the frames.
//
//  RosNetUp(link);                       //from a link driver
//  RosNetConfig(RosIpParse("10.0.2.15"),RosIpParse("255.255.255.0"),RosIpParse("10.0.2.2"));
//
//Only UDP: that's all DDS (and so ROS 2) needs.

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


#define NET_ETH_IP	0x0800
#define NET_ETH_ARP	0x0806
#define NET_P_ICMP	1
#define NET_P_IGMP	2
#define NET_P_UDP	17
#define NET_ARP_N	64
#define NET_SOCK_N	64
#define NET_SOCK_QMAX	512	//datagrams queued per socket
#define NET_FRAG_N	8	//IP datagrams being reassembled at once
#define NET_PEND_MAX	64	//packets waiting for ARP

class CRosNetIf
{
  U8 mac[6],pad[2];
  I64 ip,mask,gw,mtu;
  Bool ethernet;	//FALSE: the link carries raw IP packets (SLIP)
  Bool (*tx)(U8 *drv,U8 *frame,I64 len);
  I64 (*rx)(U8 *drv,U8 *buf,I64 cap);	//-1 if nothing
  U8 *drv;
  I64 rx_pkts,tx_pkts,rx_bad;
};

class CRosArp
{
  I64 ip,stamp;
  U8 mac[6];
  Bool valid;
};

class CRosNetDgram
{
  I64 ip,port,len;
  U8 *data;
};

class CRosNetSock
{
  I64 port;
  Bool used;
  CRosVec *q;		//CRosNetDgram *
};

class CRosNetFrag
{
  I64 src,id,proto,total,got,stamp;
  U8 *buf,*have;	//have: one byte per 8-byte block
  Bool used;
};

class CRosNetPend
{
  I64 ip,len,stamp;
  U8 *pkt;		//IP packet
};

CRosNetIf *ros_netif=NULL;
CRosArp ros_arp[NET_ARP_N];
CRosNetSock ros_socks[NET_SOCK_N];
CRosNetFrag ros_frags[NET_FRAG_N];
CRosVec *ros_net_pend=NULL;	//CRosNetPend *
I64 ros_mcast[16];
I64 ros_mcast_n=0;
I64 ros_net_lock=0;
I64 ros_ip_id=1;
I64 ros_next_eph=49152;
U8 *ros_net_buf=NULL;		//one frame
Bool ros_net_stack=FALSE;	//UDP goes through this stack (always on TempleOS)

//---- bytes, big endian

I64 NetGet16(U8 *p)
{
  return (p[0]<<8)|p[1];
}

I64 NetGet32(U8 *p)
{
  return (p[0]<<24)|(p[1]<<16)|(p[2]<<8)|p[3];
}

U0 NetPut16(U8 *p,I64 v)
{
  p[0]=(v>>8)&255;
  p[1]=v&255;
}

U0 NetPut32(U8 *p,I64 v)
{
  p[0]=(v>>24)&255;
  p[1]=(v>>16)&255;
  p[2]=(v>>8)&255;
  p[3]=v&255;
}

//Internet checksum, adding to sum (start with 0).
U64 NetSum(U8 *p,I64 n,U64 sum)
{
  I64 i=0;
  while (i+1<n) {
    sum+=(p[i]<<8)|p[i+1];
    i+=2;
  }
  if (i<n)
    sum+=p[i]<<8;
  return sum;
}

I64 NetSumFold(U64 sum)
{
  while (sum>>16)
    sum=(sum&0xFFFF)+(sum>>16);
  return (~sum)&0xFFFF;
}

Bool NetIsMcast(I64 ip)
{
  return (ip>>28)==14;
}

Bool NetIsBcast(I64 ip)
{
  CRosNetIf *nif=ros_netif;
  return ip==0xFFFFFFFF || (nif && nif->mask && (ip|nif->mask)==0xFFFFFFFF &&
	(ip&nif->mask)==(nif->ip&nif->mask));
}

//---- link

Bool NetTx(U8 *frame,I64 len)
{
  CRosNetIf *nif=ros_netif;
  if (!nif)
    return FALSE;
  nif->tx_pkts++;
  return (*nif->tx)(nif->drv,frame,len);
}

//Send an ethernet frame with payload (IP or ARP).
U0 NetEthSend(U8 *dst_mac,I64 type,U8 *payload,I64 len)
{
  U8 *f;
  CRosNetIf *nif=ros_netif;
  if (!nif->ethernet) {	//raw IP link
    NetTx(payload,len);
    return;
  }
  f=MAlloc(len+14);
  MemCpy(f,dst_mac,6);
  MemCpy(f+6,nif->mac,6);
  NetPut16(f+12,type);
  MemCpy(f+14,payload,len);
  NetTx(f,len+14);
  Free(f);
}

//---- ARP

CRosArp *NetArpFind(I64 ip)
{
  I64 i;
  for (i=0;i<NET_ARP_N;i++)
    if (ros_arp[i].valid && ros_arp[i].ip==ip)
      return &ros_arp[i];
  return NULL;
}

U0 NetArpLearn(I64 ip,U8 *mac)
{
  CRosArp *a=NetArpFind(ip);
  I64 i,oldest=0;
  if (!a) {
    for (i=0;i<NET_ARP_N;i++) {
      if (!ros_arp[i].valid) {
        oldest=i;
        break;
      }
      if (ros_arp[i].stamp<ros_arp[oldest].stamp)
        oldest=i;
    }
    a=&ros_arp[oldest];
  }
  a->ip=ip;
  MemCpy(a->mac,mac,6);
  a->valid=TRUE;
  a->stamp=RosTimeSteadyNs();
}

U0 NetArpSend(I64 oper,U8 *dst_mac,U8 *tha,I64 tpa)
{
  U8 p[28];
  CRosNetIf *nif=ros_netif;
  NetPut16(p,1);
  NetPut16(p+2,NET_ETH_IP);
  p[4]=6;
  p[5]=4;
  NetPut16(p+6,oper);
  MemCpy(p+8,nif->mac,6);
  NetPut32(p+14,nif->ip);
  MemCpy(p+18,tha,6);
  NetPut32(p+24,tpa);
  NetEthSend(dst_mac,NET_ETH_ARP,p,28);
}

U0 NetArpRequest(I64 ip)
{
  U8 bc[6],zero[6];
  MemSet(bc,0xFF,6);
  MemSet(zero,0,6);
  NetArpSend(1,bc,zero,ip);
}

//Send IP packets that were waiting for ip's MAC.
U0 NetPendFlush(I64 ip,U8 *mac)
{
  I64 i=0;
  CRosNetPend *p;
  while (i<ros_net_pend->len) {
    p=RosVecGetI64(ros_net_pend,i);
    if (p->ip==ip) {
      NetEthSend(mac,NET_ETH_IP,p->pkt,p->len);
      RosVecRemove(ros_net_pend,i);
      Free(p->pkt);
      Free(p);
    } else
      i++;
  }
}

U0 NetOnArp(U8 *p,I64 len)
{
  I64 oper,spa,tpa;
  CRosNetIf *nif=ros_netif;
  if (len<28 || NetGet16(p)!=1 || NetGet16(p+2)!=NET_ETH_IP)
    return;
  oper=NetGet16(p+6);
  spa=NetGet32(p+14);
  tpa=NetGet32(p+24);
  if (spa)
    NetArpLearn(spa,p+8);
  if (oper==1 && tpa==nif->ip && nif->ip)
    NetArpSend(2,p+8,p+8,spa);
  if (spa)
    NetPendFlush(spa,p+8);
}

//---- IPv4 send

//Send an IP packet (header + payload already built) to next hop.
U0 NetIpOut(I64 dst,U8 *pkt,I64 len)
{
  CRosNetIf *nif=ros_netif;
  U8 mac[6];
  I64 hop=dst;
  CRosArp *a;
  CRosNetPend *pd;
  if (!nif->ethernet) {
    NetTx(pkt,len);
    return;
  }
  if (NetIsMcast(dst)) {
    mac[0]=0x01;
    mac[1]=0x00;
    mac[2]=0x5E;
    mac[3]=(dst>>16)&0x7F;
    mac[4]=(dst>>8)&255;
    mac[5]=dst&255;
    NetEthSend(mac,NET_ETH_IP,pkt,len);
    return;
  }
  if (NetIsBcast(dst)) {
    MemSet(mac,0xFF,6);
    NetEthSend(mac,NET_ETH_IP,pkt,len);
    return;
  }
  if (nif->mask && (dst&nif->mask)!=(nif->ip&nif->mask) && nif->gw)
    hop=nif->gw;
  a=NetArpFind(hop);
  if (a) {
    NetEthSend(a->mac,NET_ETH_IP,pkt,len);
    return;
  }
  if (ros_net_pend->len<NET_PEND_MAX) {	//wait for ARP
    pd=CAlloc(sizeof(CRosNetPend));
    pd->ip=hop;
    pd->pkt=MAlloc(len);
    MemCpy(pd->pkt,pkt,len);
    pd->len=len;
    pd->stamp=RosTimeSteadyNs();
    RosVecPushI64(ros_net_pend,pd);
  }
  NetArpRequest(hop);
}

//Send payload as IP protocol proto to dst, fragmenting if needed.
U0 NetIpSend(I64 dst,I64 proto,U8 *payload,I64 len)
{
  CRosNetIf *nif=ros_netif;
  I64 max=(nif->mtu-20)/8*8,off=0,n,id=ros_ip_id++,ttl=64;
  U8 *pkt=MAlloc(nif->mtu+8);
  if (NetIsMcast(dst))
    ttl=1;
  while (off<len || !len) {
    n=len-off;
    if (n>max)
      n=max;
    pkt[0]=0x45;
    pkt[1]=0;
    NetPut16(pkt+2,20+n);
    NetPut16(pkt+4,id);
    if (off+n<len)
      NetPut16(pkt+6,0x2000|(off/8));	//MF
    else
      NetPut16(pkt+6,off/8);
    pkt[8]=ttl;
    pkt[9]=proto;
    NetPut16(pkt+10,0);
    NetPut32(pkt+12,nif->ip);
    NetPut32(pkt+16,dst);
    NetPut16(pkt+10,NetSumFold(NetSum(pkt,20,0)));
    MemCpy(pkt+20,payload+off,n);
    NetIpOut(dst,pkt,20+n);
    off+=n;
    if (!len)
      break;
  }
  Free(pkt);
}

//---- UDP

U0 NetUdpSend(I64 sport,I64 dst,I64 dport,U8 *data,I64 len)
{
  CRosNetIf *nif=ros_netif;
  U8 *u=MAlloc(len+8),ph[12];
  U64 sum;
  I64 c;
  NetPut16(u,sport);
  NetPut16(u+2,dport);
  NetPut16(u+4,len+8);
  NetPut16(u+6,0);
  MemCpy(u+8,data,len);
  NetPut32(ph,nif->ip);
  NetPut32(ph+4,dst);
  ph[8]=0;
  ph[9]=NET_P_UDP;
  NetPut16(ph+10,len+8);
  sum=NetSum(ph,12,0);
  c=NetSumFold(NetSum(u,len+8,sum));
  if (!c)
    c=0xFFFF;
  NetPut16(u+6,c);
  NetIpSend(dst,NET_P_UDP,u,len+8);
  Free(u);
}

U0 NetOnUdp(I64 src,I64 dst,U8 *p,I64 len)
{
  I64 sport,dport,ulen,i;
  CRosNetSock *s;
  CRosNetDgram *d;
  if (len<8)
    return;
  sport=NetGet16(p);
  dport=NetGet16(p+2);
  ulen=NetGet16(p+4);
  if (ulen<8 || ulen>len)
    return;
  for (i=0;i<NET_SOCK_N;i++) {
    s=&ros_socks[i];
    if (!s->used || s->port!=dport)
      goto next_net_1;
    if (s->q->len>=NET_SOCK_QMAX)
      goto next_net_1;	//full: drop
    d=CAlloc(sizeof(CRosNetDgram));
    d->ip=src;
    d->port=sport;
    d->len=ulen-8;
    d->data=MAlloc(d->len+1);
    MemCpy(d->data,p+8,d->len);
    RosVecPushI64(s->q,d);
    next_net_1:;
  }
}

//---- ICMP / IGMP

U0 NetOnIcmp(I64 src,U8 *p,I64 len)
{
  U8 *r;
  if (len<8 || p[0]!=8)	//echo request only
    return;
  r=MAlloc(len);
  MemCpy(r,p,len);
  r[0]=0;		//echo reply
  NetPut16(r+2,0);
  NetPut16(r+2,NetSumFold(NetSum(r,len,0)));
  NetIpSend(src,NET_P_ICMP,r,len);
  Free(r);
}

U0 NetIgmpReport(I64 group)
{
  U8 m[8];
  m[0]=0x16;		//v2 membership report
  m[1]=0;
  NetPut16(m+2,0);
  NetPut32(m+4,group);
  NetPut16(m+2,NetSumFold(NetSum(m,8,0)));
  NetIpSend(group,NET_P_IGMP,m,8);
}

U0 NetOnIgmp(U8 *p,I64 len)
{
  I64 i;
  if (len>=8 && p[0]==0x11)	//query: report what we're in
    for (i=0;i<ros_mcast_n;i++)
      NetIgmpReport(ros_mcast[i]);
}

//---- IPv4 receive

U0 NetIpDeliver(I64 src,I64 dst,I64 proto,U8 *p,I64 len)
{
  switch (proto) {
    case NET_P_UDP: NetOnUdp(src,dst,p,len); break;
    case NET_P_ICMP: NetOnIcmp(src,p,len); break;
    case NET_P_IGMP: NetOnIgmp(p,len); break;
  }
}

Bool NetForUs(I64 dst)
{
  I64 i;
  CRosNetIf *nif=ros_netif;
  if (dst==nif->ip || NetIsBcast(dst))
    return TRUE;
  if (!nif->ip && !NetIsMcast(dst))
    return TRUE;	//no address yet (DHCP): servers may answer to the offered one
  if (NetIsMcast(dst))
    for (i=0;i<ros_mcast_n;i++)
      if (ros_mcast[i]==dst || dst==0xE0000001)
        return TRUE;
  return dst==0xE0000001;	//all hosts
}

//One fragment arrived. Delivers the datagram when it's complete.
U0 NetOnFrag(I64 src,I64 dst,I64 proto,I64 id,I64 off,Bool more,U8 *p,I64 len)
{
  CRosNetFrag *f=NULL;
  I64 i,b,oldest=0,now=RosTimeSteadyNs();
  for (i=0;i<NET_FRAG_N;i++) {
    if (ros_frags[i].used && ros_frags[i].src==src && ros_frags[i].id==id &&
	ros_frags[i].proto==proto)
      f=&ros_frags[i];
  }
  if (!f) {
    for (i=0;i<NET_FRAG_N;i++) {
      if (!ros_frags[i].used) {
        oldest=i;
        break;
      }
      if (ros_frags[i].stamp<ros_frags[oldest].stamp)
        oldest=i;
    }
    f=&ros_frags[oldest];
    Free(f->buf);
    Free(f->have);
    MemSet(f,0,sizeof(CRosNetFrag));
    f->used=TRUE;
    f->src=src;
    f->id=id;
    f->proto=proto;
    f->total=-1;
    f->buf=MAlloc(65536);
    f->have=CAlloc(8192);
  }
  f->stamp=now;
  if (off+len>65535 || (more && len%8)) {
    f->used=FALSE;
    return;
  }
  MemCpy(f->buf+off,p,len);
  for (b=off/8;b<(off+len+7)/8;b++)
    if (!f->have[b]) {
      f->have[b]=1;
      f->got+=8;
    }
  if (!more && f->total<0) {
    f->total=off+len;
    f->got-=(8-len%8)%8;	//the last block may be short
  }
  if (f->total>=0 && f->got>=f->total) {
    f->used=FALSE;
    NetIpDeliver(src,dst,proto,f->buf,f->total);
  }
}

U0 NetOnIp(U8 *p,I64 len)
{
  I64 ihl,tot,src,dst,proto,fl,off;
  if (len<20 || (p[0]>>4)!=4)
    return;
  ihl=(p[0]&15)*4;
  tot=NetGet16(p+2);
  if (ihl<20 || tot<ihl || tot>len || NetSumFold(NetSum(p,ihl,0))) {
    ros_netif->rx_bad++;
    return;
  }
  src=NetGet32(p+12);
  dst=NetGet32(p+16);
  if (!NetForUs(dst))
    return;
  proto=p[9];
  fl=NetGet16(p+6);
  off=(fl&0x1FFF)*8;
  if ((fl&0x2000) || off)
    NetOnFrag(src,dst,proto,NetGet16(p+4),off,(fl&0x2000)!=0,p+ihl,tot-ihl);
  else
    NetIpDeliver(src,dst,proto,p+ihl,tot-ihl);
}

//---- polling

//Handle up to max received frames, and housekeeping. Takes the lock.
I64 RosNetPoll(I64 max)
{
  CRosNetIf *nif=ros_netif;
  I64 n,done=0,i,now;
  CRosNetPend *pd;
  if (!nif)
    return 0;
  RosLockGet(&ros_net_lock);
  while (done<max) {
    n=(*nif->rx)(nif->drv,ros_net_buf,nif->mtu+64);
    if (n<0)
      break;
    done++;
    nif->rx_pkts++;
    if (!nif->ethernet)
      NetOnIp(ros_net_buf,n);
    else if (n>=14) {
      if (NetGet16(ros_net_buf+12)==NET_ETH_IP)
        NetOnIp(ros_net_buf+14,n-14);
      else if (NetGet16(ros_net_buf+12)==NET_ETH_ARP)
        NetOnArp(ros_net_buf+14,n-14);
    }
  }
  now=RosTimeSteadyNs();
  i=0;
  while (i<ros_net_pend->len) {	//ARP gave up after 3 s
    pd=RosVecGetI64(ros_net_pend,i);
    if (now-pd->stamp>3*ROS_NS_PER_SEC) {
      RosVecRemove(ros_net_pend,i);
      Free(pd->pkt);
      Free(pd);
    } else
      i++;
  }
  for (i=0;i<NET_FRAG_N;i++)	//half datagrams die after 5 s
    if (ros_frags[i].used && now-ros_frags[i].stamp>5*ROS_NS_PER_SEC)
      ros_frags[i].used=FALSE;
  RosLockPut(&ros_net_lock);
  return done;
}

//---- setup

U0 RosNetConfig(I64 ip,I64 mask,I64 gw)
{
  ros_netif->ip=ip;
  ros_netif->mask=mask;
  ros_netif->gw=gw;
  ros_net_ip=ip;
}

//Bring the stack up on a link.
U0 RosNetUp(CRosNetIf *nif)
{
  ros_netif=nif;
  if (!nif->mtu)
    nif->mtu=1500;
  if (!ros_net_pend)
    ros_net_pend=RosVecNew(8);
  if (!ros_net_buf)
    ros_net_buf=MAlloc(65536+64);
  ros_net_stack=TRUE;
}

//A datagram to our own address: straight into our sockets.
U0 NetUdpLoop(I64 sport,I64 dport,U8 *data,I64 len)
{
  U8 *u=MAlloc(len+8);
  NetPut16(u,sport);
  NetPut16(u+2,dport);
  NetPut16(u+4,len+8);
  NetPut16(u+6,0);
  MemCpy(u+8,data,len);
  NetOnUdp(ros_netif->ip,ros_netif->ip,u,len+8);
  Free(u);
}

//---- the API Udp.HC uses (handles are socket indexes)

I64 RosNetUdpOpen(I64 port)
{
  I64 i,h=-1;
  RosLockGet(&ros_net_lock);
  if (!port) {
    port=ros_next_eph++;
    if (ros_next_eph>65000)
      ros_next_eph=49152;
  }
  for (i=0;i<NET_SOCK_N;i++)	//shared ports are fine (multicast)
    if (!ros_socks[i].used) {
      ros_socks[i].used=TRUE;
      ros_socks[i].port=port;
      ros_socks[i].q=RosVecNew(8);
      h=i;
      break;
    }
  RosLockPut(&ros_net_lock);
  return h;
}

I64 RosNetUdpPort(I64 h)
{
  return ros_socks[h].port;
}

Bool RosNetUdpJoin(I64 h,I64 group)
{
  I64 i;
  RosLockGet(&ros_net_lock);
  for (i=0;i<ros_mcast_n;i++)
    if (ros_mcast[i]==group) {
      RosLockPut(&ros_net_lock);
      return TRUE;
    }
  if (ros_mcast_n<16) {
    ros_mcast[ros_mcast_n]=group;
    ros_mcast_n++;
    NetIgmpReport(group);
  }
  RosLockPut(&ros_net_lock);
  return TRUE;
}

I64 RosNetUdpSend(I64 h,I64 ip,I64 port,U8 *data,I64 len)
{
  if (!ros_netif || h<0)
    return -1;
  RosLockGet(&ros_net_lock);
  if (ip==ros_netif->ip)	//to ourselves: straight into the sockets
    NetUdpLoop(ros_socks[h].port,port,data,len);
  else
    NetUdpSend(ros_socks[h].port,ip,port,data,len);
  RosLockPut(&ros_net_lock);
  return len;
}

I64 RosNetUdpRecv(I64 h,U8 *buf,I64 cap,I64 *ip,I64 *port)
{
  CRosNetDgram *d;
  I64 n;
  if (h<0)
    return -1;
  RosNetPoll(64);
  RosLockGet(&ros_net_lock);
  if (!ros_socks[h].q->len) {
    RosLockPut(&ros_net_lock);
    return -1;
  }
  d=RosVecGetI64(ros_socks[h].q,0);
  RosVecRemove(ros_socks[h].q,0);
  RosLockPut(&ros_net_lock);
  n=d->len;
  if (n>cap)
    n=cap;
  MemCpy(buf,d->data,n);
  *ip=d->ip;
  *port=d->port;
  Free(d->data);
  Free(d);
  return n;
}

U0 RosNetUdpClose(I64 h)
{
  CRosNetDgram *d;
  I64 i;
  if (h<0)
    return;
  RosLockGet(&ros_net_lock);
  for (i=0;i<ros_socks[h].q->len;i++) {
    d=RosVecGetI64(ros_socks[h].q,i);
    Free(d->data);
    Free(d);
  }
  RosVecDel(ros_socks[h].q);
  ros_socks[h].used=FALSE;
  RosLockPut(&ros_net_lock);
}
