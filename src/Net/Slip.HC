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
  I64 port;		//TempleOS: UART base (0x3F8 = COM1)
  I64 fd;		//host: a pty or socket
  U8 *rx;
  I64 rx_len;
  Bool esc,overflow;
  U8 *tx;
  U8 *in;		//host: buffered reads
  I64 in_len,in_pos;
};

//---- one byte at a time (UART on TempleOS, a file descriptor on Linux)

U0 SlipPutByte(CRosSlip *s,U8 c)
{
//@host-skip-begin
  while (!(InU8(s->port+5)&0x20))	//transmit holding register empty
    Yield();
  OutU8(s->port,c);
//@host-skip-end
}

//Next byte, or -1 if none waiting.
I64 SlipGetByte(CRosSlip *s)
{
//@host-skip-begin
  if (InU8(s->port+5)&1)	//data ready
    return InU8(s->port);
//@host-skip-end
//@host: if (s->in_pos>=s->in_len) { ssize_t r=read(s->fd,s->in,4096); if (r<=0) return -1; s->in_len=r; s->in_pos=0; }
//@host: return s->in[s->in_pos++];
  return -1;
}

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
  I64 n,i;
  if (len>SLIP_MAX)
    return FALSE;
  n=SlipEncode(p,len,s->tx);
//@host-skip-begin
  for (i=0;i<n;i++)
    SlipPutByte(s,s->tx[i]);
//@host-skip-end
//@host: { I64 off=0; while (off<n) { ssize_t w=write(s->fd,s->tx+off,n-off); if (w>0) off+=w; else if (errno!=EAGAIN) return FALSE; else usleep(100); } }
  return TRUE;
}

I64 SlipRx(CRosSlip *s,U8 *buf,I64 cap)
{
  I64 c,n;
  while (TRUE) {
    c=SlipGetByte(s);
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

//@host-skip-begin
//SLIP on a COM port (0x3F8 = COM1, 0x2F8 = COM2) at 115200 8N1.
CRosNetIf *SlipOpenCom(I64 base)
{
  CRosSlip *s=CAlloc(sizeof(CRosSlip));
  s->port=base;
  OutU8(base+1,0x00);	//no interrupts
  OutU8(base+3,0x80);	//DLAB: set the divisor
  OutU8(base+0,0x01);	//115200 baud
  OutU8(base+1,0x00);
  OutU8(base+3,0x03);	//8 bits, no parity, 1 stop
  OutU8(base+2,0xC7);	//FIFO on, cleared, 14-byte threshold
  OutU8(base+4,0x0B);	//DTR | RTS | OUT2
  return SlipLink(s);
}
//@host-skip-end

//Host tests: SLIP over a file descriptor (a pty).
CRosNetIf *SlipOpenFd(I64 fd)
{
  CRosSlip *s=CAlloc(sizeof(CRosSlip));
  s->fd=fd;
  s->in=MAlloc(4096);
  return SlipLink(s);
}
