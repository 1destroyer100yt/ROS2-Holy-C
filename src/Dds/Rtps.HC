//Rtps: the DDSI-RTPS 2.x wire format (what every DDS, and so ROS 2,
//speaks on UDP). Building and parsing messages, submessages, GUIDs,
//locators and parameter lists. The protocol logic is in Dds.HC.

#define RTPS_PAD		0x01
#define RTPS_ACKNACK		0x06
#define RTPS_HEARTBEAT		0x07
#define RTPS_GAP		0x08
#define RTPS_INFO_TS		0x09
#define RTPS_INFO_SRC		0x0C
#define RTPS_INFO_DST		0x0E
#define RTPS_NACK_FRAG		0x12
#define RTPS_HEARTBEAT_FRAG	0x13
#define RTPS_DATA		0x15
#define RTPS_DATA_FRAG		0x16

//entity ids, as the 4 bytes read big-endian
#define RTPS_EID_UNKNOWN	0x00000000
#define RTPS_EID_PARTICIPANT	0x000001C1
#define RTPS_EID_SPDP_W		0x000100C2
#define RTPS_EID_SPDP_R		0x000100C7
#define RTPS_EID_SEDP_PUB_W	0x000003C2
#define RTPS_EID_SEDP_PUB_R	0x000003C7
#define RTPS_EID_SEDP_SUB_W	0x000004C2
#define RTPS_EID_SEDP_SUB_R	0x000004C7
#define RTPS_KIND_WRITER_NOKEY	0x03
#define RTPS_KIND_READER_NOKEY	0x04

//parameter ids
#define PID_SENTINEL		0x0001
#define PID_PARTICIPANT_LEASE	0x0002
#define PID_TOPIC_NAME		0x0005
#define PID_TYPE_NAME		0x0007
#define PID_DOMAIN_ID		0x000F
#define PID_PROTOCOL_VERSION	0x0015
#define PID_VENDORID		0x0016
#define PID_RELIABILITY		0x001A
#define PID_DURABILITY		0x001D
#define PID_USER_DATA		0x002C
#define PID_UNICAST_LOCATOR	0x002F
#define PID_DEFAULT_UC_LOC	0x0031
#define PID_META_UC_LOC		0x0032
#define PID_META_MC_LOC		0x0033
#define PID_HISTORY		0x0040
#define PID_DEFAULT_MC_LOC	0x0048
#define PID_PARTICIPANT_GUID	0x0050
#define PID_BUILTIN_ENDPOINTS	0x0058
#define PID_ENDPOINT_GUID	0x005A
#define PID_ENTITY_NAME		0x0062
#define PID_KEY_HASH		0x0070
#define PID_STATUS_INFO		0x0071
#define PID_RELATED_SAMPLE_IDENTITY	0x0083	//DDS-RPC; Fast DDS services use it
#define PID_CUSTOM_RELATED_SAMPLE_IDENTITY 0x800F	//Fast DDS's older one
#define RTPS_VENDOR_EPROSIMA	0x010F	//Fast DDS

//builtin endpoint set bits: participant + publications + subscriptions
#define RTPS_BUILTIN_SET	0x3F

#define RTPS_VENDOR_HI	0x01
#define RTPS_VENDOR_LO	0xFE	//not an assigned vendor id

//ports for domain d and participant index p (spec defaults)
I64 RtpsPortSpdpMc(I64 d)		{return 7400+250*d;}
I64 RtpsPortUserMc(I64 d)		{return 7401+250*d;}
I64 RtpsPortMetaUc(I64 d,I64 p)	{return 7410+250*d+2*p;}
I64 RtpsPortUserUc(I64 d,I64 p)	{return 7411+250*d+2*p;}
#define RTPS_MC_ADDR	0xEFFF0001	//239.255.0.1

class CRtpsGuid
{
  U8 prefix[12];
  U8 pad[4];
  I64 eid;
};

class CRtpsLoc
{
  I64 ip,port;
};

Bool RtpsGuidEq(CRtpsGuid *a,CRtpsGuid *b)
{
  return a->eid==b->eid && !MemCmp(a->prefix,b->prefix,12);
}

Bool RtpsPrefixEq(U8 *a,U8 *b)
{
  return !MemCmp(a,b,12);
}

//---- writing (little endian, our choice; E flag = 1)

U0 RtpsW8(CRosBuf *b,I64 v)
{
  RosBufReserve(b,1);
  b->data[b->len]=v&0xFF;
  b->len++;
}

U0 RtpsW16(CRosBuf *b,I64 v)
{
  RtpsW8(b,v);
  RtpsW8(b,v>>8);
}

U0 RtpsW32(CRosBuf *b,I64 v)
{
  RtpsW16(b,v);
  RtpsW16(b,v>>16);
}

U0 RtpsW32BE(CRosBuf *b,I64 v)
{
  RtpsW8(b,v>>24);
  RtpsW8(b,v>>16);
  RtpsW8(b,v>>8);
  RtpsW8(b,v);
}

U0 RtpsWBytes(CRosBuf *b,U8 *p,I64 n)
{
  RosBufPutBytes(b,p,n);
}

//Sequence number: high int32, then low uint32.
U0 RtpsWSn(CRosBuf *b,I64 sn)
{
  RtpsW32(b,sn>>32);
  RtpsW32(b,sn&0xFFFFFFFF);
}

U0 RtpsWPatch16(CRosBuf *b,I64 at,I64 v)
{
  b->data[at]=v&0xFF;
  b->data[at+1]=(v>>8)&0xFF;
}

//"RTPS" 2.3, vendor, our guid prefix.
U0 RtpsWHeader(CRosBuf *b,U8 *prefix)
{
  RtpsW8(b,'R');
  RtpsW8(b,'T');
  RtpsW8(b,'P');
  RtpsW8(b,'S');
  RtpsW8(b,2);
  RtpsW8(b,3);
  RtpsW8(b,RTPS_VENDOR_HI);
  RtpsW8(b,RTPS_VENDOR_LO);
  RtpsWBytes(b,prefix,12);
}

//Start a submessage; returns where its length goes (RtpsWSubEnd).
I64 RtpsWSub(CRosBuf *b,I64 id,I64 flags)
{
  I64 at;
  RtpsW8(b,id);
  RtpsW8(b,flags|1);
  at=b->len;
  RtpsW16(b,0);
  return at;
}

U0 RtpsWSubEnd(CRosBuf *b,I64 at)
{
  while ((b->len-at+2)%4)	//keep the next submessage 4-aligned (start = at-2)
    RtpsW8(b,0);
  RtpsWPatch16(b,at,b->len-at-2);
}

U0 RtpsWInfoDst(CRosBuf *b,U8 *prefix)
{
  I64 at=RtpsWSub(b,RTPS_INFO_DST,0);
  RtpsWBytes(b,prefix,12);
  RtpsWSubEnd(b,at);
}

U0 RtpsWInfoTs(CRosBuf *b,I64 unix_ns)
{
  I64 at=RtpsWSub(b,RTPS_INFO_TS,0),frac;
  frac=ToI64(ToF64(RosTimeNsec(unix_ns))*4.294967296);	//ns -> 2^-32 s
  RtpsW32(b,RosTimeSec(unix_ns));
  RtpsW32(b,frac);
  RtpsWSubEnd(b,at);
}

//---- parameter lists (PL_CDR_LE)

U0 RtpsPlBegin(CRosBuf *b)
{
  RtpsW8(b,0);
  RtpsW8(b,3);	//PL_CDR_LE
  RtpsW16(b,0);
}

I64 RtpsPlParam(CRosBuf *b,I64 pid)
{
  I64 at;
  RtpsW16(b,pid);
  at=b->len;
  RtpsW16(b,0);
  return at;
}

U0 RtpsPlEnd(CRosBuf *b,I64 at)
{
  while ((b->len-at-2)%4)
    RtpsW8(b,0);
  RtpsWPatch16(b,at,b->len-at-2);
}

U0 RtpsPlU32(CRosBuf *b,I64 pid,I64 v)
{
  I64 at=RtpsPlParam(b,pid);
  RtpsW32(b,v);
  RtpsPlEnd(b,at);
}

U0 RtpsPlStr(CRosBuf *b,I64 pid,U8 *s)
{
  I64 at=RtpsPlParam(b,pid),n=StrLen(s);
  RtpsW32(b,n+1);
  RtpsWBytes(b,s,n);
  RtpsW8(b,0);
  RtpsPlEnd(b,at);
}

//sequence<octet> holding s (no NUL), e.g. USER_DATA "key=value;" pairs.
U0 RtpsPlOctets(CRosBuf *b,I64 pid,U8 *s)
{
  I64 at=RtpsPlParam(b,pid),n=StrLen(s);
  RtpsW32(b,n);
  RtpsWBytes(b,s,n);
  RtpsPlEnd(b,at);
}

U0 RtpsPlGuid(CRosBuf *b,I64 pid,U8 *prefix,I64 eid)
{
  I64 at=RtpsPlParam(b,pid);
  RtpsWBytes(b,prefix,12);
  RtpsW32BE(b,eid);
  RtpsPlEnd(b,at);
}

U0 RtpsPlLoc(CRosBuf *b,I64 pid,CRtpsLoc *l)
{
  I64 at=RtpsPlParam(b,pid),i;
  RtpsW32(b,1);		//LOCATOR_KIND_UDPv4
  RtpsW32(b,l->port);
  for (i=0;i<12;i++)
    RtpsW8(b,0);
  RtpsW32BE(b,l->ip);
  RtpsPlEnd(b,at);
}

U0 RtpsPlEndList(CRosBuf *b)
{
  RtpsW16(b,PID_SENTINEL);
  RtpsW16(b,0);
}

//---- reading

class CRtpsIn		//a byte range being parsed
{
  U8 *p;
  I64 len,pos;
  Bool be;		//big endian
  Bool err;
};

U0 RtpsInInit(CRtpsIn *r,U8 *p,I64 len,Bool be)
{
  r->p=p;
  r->len=len;
  r->pos=0;
  r->be=be;
  r->err=FALSE;
}

U64 RtpsRN(CRtpsIn *r,I64 n)
{
  U64 v=0;
  I64 i;
  if (r->err || r->pos+n>r->len) {
    r->err=TRUE;
    return 0;
  }
  if (r->be)
    for (i=0;i<n;i++)
      v=(v<<8)|r->p[r->pos+i];
  else
    for (i=n-1;i>=0;i--)
      v=(v<<8)|r->p[r->pos+i];
  r->pos+=n;
  return v;
}

I64 RtpsR32BE(CRtpsIn *r)
{
  Bool be=r->be;
  I64 v;
  r->be=TRUE;
  v=RtpsRN(r,4);
  r->be=be;
  return v;
}

U8 *RtpsRBytes(CRtpsIn *r,I64 n)
{
  U8 *p;
  if (r->err || r->pos+n>r->len || n<0) {
    r->err=TRUE;
    return NULL;
  }
  p=r->p+r->pos;
  r->pos+=n;
  return p;
}

I64 RtpsRSn(CRtpsIn *r)
{
  I64 hi=RtpsRN(r,4),lo=RtpsRN(r,4);
  if (hi&0x80000000)
    hi-=0x100000000;
  return (hi<<32)|lo;
}

//Walk a parameter list. Call with *pos at the first parameter.
//Returns FALSE at the sentinel or on bad data.
Bool RtpsPlNext(CRtpsIn *r,I64 *pid,CRtpsIn *val)
{
  I64 len;
  *pid=RtpsRN(r,2);
  len=RtpsRN(r,2);
  if (r->err || *pid==PID_SENTINEL)
    return FALSE;
  RtpsInInit(val,RtpsRBytes(r,len),len,r->be);
  return !r->err;
}

//A CDR string parameter -> new string, NULL if bad.
U8 *RtpsPlReadStr(CRtpsIn *v)
{
  I64 n=RtpsRN(v,4);
  U8 *p=RtpsRBytes(v,n),*s;
  if (!p || !n || p[n-1])
    return NULL;
  s=MAlloc(n);
  MemCpy(s,p,n);
  return s;
}

//The value of key in a USER_DATA "k1=v1;k2=v2;" octet sequence (new), or NULL.
U8 *RtpsPlReadUserKey(CRtpsIn *v,U8 *key)
{
  I64 n=RtpsRN(v,4),i,k=StrLen(key),j;
  U8 *p=RtpsRBytes(v,n),*s;
  if (!p)
    return NULL;
  i=0;
  while (i+k<n) {
    if (!MemCmp(p+i,key,k) && p[i+k]=='=' && (!i || p[i-1]==';')) {
      j=i+k+1;
      while (j<n && p[j]!=';' && p[j])
        j++;
      s=MAlloc(j-i-k);
      MemCpy(s,p+i+k+1,j-i-k-1);
      s[j-i-k-1]=0;
      return s;
    }
    i++;
  }
  return NULL;
}

U0 RtpsPlReadLoc(CRtpsIn *v,CRtpsLoc *l)
{
  I64 kind=RtpsRN(v,4),port=RtpsRN(v,4);
  RtpsRBytes(v,12);
  if (kind==1 && !v->err) {
    l->port=port;
    l->ip=RtpsR32BE(v);
  }
}

U0 RtpsPlReadGuid(CRtpsIn *v,CRtpsGuid *g)
{
  U8 *p=RtpsRBytes(v,12);
  if (p) {
    MemCpy(g->prefix,p,12);
    g->eid=RtpsR32BE(v);
  }
}
