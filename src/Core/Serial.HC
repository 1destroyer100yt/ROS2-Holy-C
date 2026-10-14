//Serial: a serial port for talking to hardware (motor controllers,
//sensors). On TempleOS (and in the QEMU guest test) a 16550 UART by its
//I/O base (0x3F8 COM1, 0x2F8 COM2); on Linux an open file descriptor
//(a pty in tests, a /dev/ttyUSB0...).
//
//  CRosSerial *s=RosSerialOpenCom(0x2F8,38400);
//  RosSerialWrite(s,buf,n);
//  n=RosSerialRead(s,buf,cap,10*ROS_NS_PER_MS);   //waits up to 10 ms

class CRosSerial
{
  I64 port;		//UART base, or 0
  I64 fd;		//host file descriptor, or -1
  I64 rx_bytes,tx_bytes;
  U8 *in;		//host: buffered reads
  I64 in_len,in_pos;
};

//A 16550 at base, 8N1, polled (no interrupts).
CRosSerial *RosSerialOpenCom(I64 base,I64 baud)
{
  CRosSerial *s=CAlloc(sizeof(CRosSerial));
  I64 div=115200/baud;
  s->port=base;
  s->fd=-1;
  if (div<1)
    div=1;
  OutU8(base+1,0x00);		//no interrupts
  OutU8(base+3,0x80);		//DLAB: set the divisor
  OutU8(base+0,div&255);
  OutU8(base+1,(div>>8)&255);
  OutU8(base+3,0x03);		//8 bits, no parity, 1 stop
  OutU8(base+2,0xC7);		//FIFO on, cleared, 14-byte threshold
  OutU8(base+4,0x0B);		//DTR | RTS | OUT2
  return s;
}

//An already open (non-blocking) file descriptor. Host only.
CRosSerial *RosSerialOpenFd(I64 fd)
{
  CRosSerial *s=CAlloc(sizeof(CRosSerial));
  s->fd=fd;
  s->in=MAlloc(4096);
  return s;
}

U0 RosSerialClose(CRosSerial *s)
{
  if (!s)
    return;
//@host: if (s->fd>=0) close(s->fd);
  Free(s->in);
  Free(s);
}

//Send n bytes (waits while the UART is busy).
U0 RosSerialWrite(CRosSerial *s,U8 *p,I64 n)
{
  I64 i,spin;
  s->tx_bytes+=n;
  if (s->port) {
    for (i=0;i<n;i++) {
      spin=0;
      while (!(InU8(s->port+5)&0x20) && spin<10000000)	//transmit holding empty
        spin++;
      OutU8(s->port,p[i]);
    }
    return;
  }
//@host: while (n>0) { ssize_t w=write(s->fd,p,n); if (w>0) { p+=w; n-=w; } else if (w<0 && errno!=EAGAIN) break; else usleep(100); }
}

//One byte if one is there now, else -1.
I64 RosSerialGetC(CRosSerial *s)
{
  U8 c;
  if (s->port) {
    if (InU8(s->port+5)&1)	//data ready
      return InU8(s->port);
    return -1;
  }
//@host: if (s->in_pos>=s->in_len) { ssize_t r=read(s->fd,s->in,4096); if (r<=0) return -1; s->in_len=r; s->in_pos=0; }
//@host: c=s->in[s->in_pos++]; return c;
  return -1;
}

//Read up to cap bytes, waiting up to wait_ns for them to arrive
//(returns early once cap bytes are in). Returns how many.
I64 RosSerialRead(CRosSerial *s,U8 *buf,I64 cap,I64 wait_ns)
{
  I64 n=0,c,end=RosTimeSteadyNs()+wait_ns;
  while (n<cap) {
    c=RosSerialGetC(s);
    if (c>=0) {
      buf[n++]=c;
      goto next_serial_1;
    }
    if (RosTimeSteadyNs()>=end)
      break;
    Yield();
    next_serial_1:;
  }
  s->rx_bytes+=n;
  return n;
}

//Throw away whatever is waiting.
U0 RosSerialFlush(CRosSerial *s)
{
  I64 n=0;
  while (RosSerialGetC(s)>=0 && n<100000)
    n++;
}
