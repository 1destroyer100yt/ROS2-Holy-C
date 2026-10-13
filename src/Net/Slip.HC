//Slip: IP over a serial line (SLIP, RFC 1055). For machines without a
//supported network card: TempleOS sends IP packets out of COM1 and the
//other end turns them back into a network interface (Linux: slattach,
//or tools/nettest/slip_relay.py).
//
//  END=0xC0 ends a packet; inside, 0xC0 -> DB DC and 0xDB -> DB DD.

#define SLIP_END	0xC0
#define SLIP_ESC	0xDB
#define SLIP_ESC_END	0xDC
#define SLIP_ESC_ESC	0xDD
#define SLIP_MAX	2048

class CRosSlip
{
  CRosSerial *ser;	//a UART (TempleOS) or a pty (Linux tests)
  U8 *rx;
  I64 rx_len;
  Bool esc,overflow;
  U8 *tx;
};

//Encode a packet into buf (needs 2*len+2). Returns the length.
I64 SlipEncode(U8 *p,I64 len,U8 *buf)
{
  I64 i,n=0;
  buf[n++]=SLIP_END;	//flush line noise first
  for (i=0;i<len;i++) {
    if (p[i]==SLIP_END) {
      buf[n++]=SLIP_ESC;
      buf[n++]=SLIP_ESC_END;
    } else if (p[i]==SLIP_ESC) {
      buf[n++]=SLIP_ESC;
      buf[n++]=SLIP_ESC_ESC;
    } else
      buf[n++]=p[i];
  }
  buf[n++]=SLIP_END;
  return n;
}

//Feed one received byte. Returns the packet length when one is
//complete (packet in s->rx), else -1.
I64 SlipFeed(CRosSlip *s,U8 c)
{
  I64 n;
  if (c==SLIP_END) {
    n=s->rx_len;
    s->rx_len=0;
    s->esc=FALSE;
    if (s->overflow) {
      s->overflow=FALSE;
      return -1;
    }
    if (n)
      return n;
    return -1;
  }
  if (s->esc) {
    s->esc=FALSE;
    if (c==SLIP_ESC_END)
      c=SLIP_END;
    else if (c==SLIP_ESC_ESC)
      c=SLIP_ESC;
  } else if (c==SLIP_ESC) {
    s->esc=TRUE;
    return -1;
  }
  if (s->rx_len>=SLIP_MAX)
    s->overflow=TRUE;
  else {
    s->rx[s->rx_len]=c;
    s->rx_len++;
  }
  return -1;
}

Bool SlipTx(CRosSlip *s,U8 *p,I64 len)
{
  I64 n;
  if (len>SLIP_MAX)
    return FALSE;
  n=SlipEncode(p,len,s->tx);
  RosSerialWrite(s->ser,s->tx,n);
  return TRUE;
}

I64 SlipRx(CRosSlip *s,U8 *buf,I64 cap)
{
  I64 c,n;
  while (TRUE) {
    c=RosSerialGetC(s->ser);
    if (c<0)
      return -1;
    n=SlipFeed(s,c);
    if (n>0) {
      if (n>cap)
        n=cap;
      MemCpy(buf,s->rx,n);
      return n;
    }
  }
  return -1;
}

CRosNetIf *SlipLink(CRosSlip *s)
{
  CRosNetIf *nif=CAlloc(sizeof(CRosNetIf));
  s->rx=MAlloc(SLIP_MAX+16);
  s->tx=MAlloc(2*SLIP_MAX+16);
  nif->ethernet=FALSE;	//raw IP packets
  nif->mtu=1500;
  nif->tx=&SlipTx;
  nif->rx=&SlipRx;
  nif->drv=s;
  return nif;
}

//SLIP on a COM port (0x3F8 = COM1, 0x2F8 = COM2) at 115200 8N1.
CRosNetIf *SlipOpenCom(I64 base)
{
  CRosSlip *s=CAlloc(sizeof(CRosSlip));
  s->ser=RosSerialOpenCom(base,115200);
  return SlipLink(s);
}

//Host tests: SLIP over a file descriptor (a pty).
CRosNetIf *SlipOpenFd(I64 fd)
{
  CRosSlip *s=CAlloc(sizeof(CRosSlip));
  s->ser=RosSerialOpenFd(fd);
  return SlipLink(s);
}
