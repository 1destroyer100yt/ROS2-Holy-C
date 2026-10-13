//Start: one call to get a TempleOS machine onto a ROS 2 network.
//
//  RosNetwork("10.0.3.2","255.255.255.0","10.0.3.1",0);
//    finds the e1000, sets the address, starts DDS on domain 0 with
//    multicast discovery and 10.0.3.1 as a unicast peer too.
//  RosNetworkDhcp(0);
//    the same, with the address from the network's DHCP server.
//  RosNetworkSlip(0x3F8,"10.0.4.2","10.0.4.1",0);
//    the same over a serial line (COM1).
//See docs/NETWORK.md for the QEMU and Linux side.

//Bring up DDS on top of a configured stack.
CRtpsPart *RosNetDds(I64 peer,I64 domain,Bool multicast)
{
  CRtpsPart *pt=RosDdsStart(domain,multicast);
  if (pt && peer)
    RosDdsAddPeer(pt,peer);
  return pt;
}

//@host-skip-begin
Bool RosNetwork(U8 *ip,U8 *mask,U8 *peer,I64 domain)
{
  CRosNetIf *nif=E1000Open();
  if (!nif) {
    Print("RosNetwork: no supported network card (e1000) found\n");
    return FALSE;
  }
  RosNetUp(nif);
  RosNetConfig(RosIpParse(ip),RosIpParse(mask),RosIpParse(peer));
  return RosNetDds(RosIpParse(peer),domain,TRUE)!=NULL;
}

Bool RosNetworkDhcp(I64 domain)
{
  CRosNetIf *nif=E1000Open();
  if (!nif) {
    Print("RosNetwork: no supported network card (e1000) found\n");
    return FALSE;
  }
  RosNetUp(nif);
  if (!RosNetDhcp(15*ROS_NS_PER_SEC)) {
    Print("RosNetworkDhcp: no answer from a DHCP server\n");
    return FALSE;
  }
  Print("RosNetworkDhcp: %s, router %s\n",RosIpStr(ros_netif->ip),RosIpStr(ros_netif->gw));
  return RosNetDds(ros_netif->gw,domain,TRUE)!=NULL;
}

Bool RosNetworkSlip(I64 com,U8 *ip,U8 *peer,I64 domain)
{
  RosNetUp(SlipOpenCom(com));
  RosNetConfig(RosIpParse(ip),0xFFFFFFFC,0);	//point to point
  return RosNetDds(RosIpParse(peer),domain,FALSE)!=NULL;	//no multicast on SLIP
}
//@host-skip-end
