//E1000: driver for the Intel 8254x network card (QEMU's default "e1000",
//and VirtualBox's "Intel PRO/1000 MT"). Polled, no interrupts.
//
//Registers are memory mapped (BAR0). (The card's I/O-port window
//isn't implemented by QEMU's e1000: reads give 0.) TempleOS
//identity-maps memory, so a buffer's address is also its physical
//(DMA) address; device memory is reached uncached through
//dev.uncached_alias, like TempleOS's own HPET and LAPIC code.
//
//Tested on QEMU's e1000 from a Linux guest (tools/e1000test): the same
//code, with /dev/mem for BAR0, iopl() for the PCI ports and
///proc/self/pagemap for physical addresses.

#define E1000_CTRL	0x0000
#define E1000_STATUS	0x0008
#define E1000_ICR	0x00C0
#define E1000_IMC	0x00D8
#define E1000_RCTL	0x0100
#define E1000_TCTL	0x0400
#define E1000_TIPG	0x0410
#define E1000_RDBAL	0x2800
#define E1000_RDBAH	0x2804
#define E1000_RDLEN	0x2808
#define E1000_RDH	0x2810
#define E1000_RDT	0x2818
#define E1000_TDBAL	0x3800
#define E1000_TDBAH	0x3804
#define E1000_TDLEN	0x3808
#define E1000_TDH	0x3810
#define E1000_TDT	0x3818
#define E1000_MTA	0x5200
#define E1000_RAL0	0x5400
#define E1000_RAH0	0x5404
#define E1000_NDESC	32
#define E1000_BUF	2048

class CE1000Rx
{
  U64 addr;
  U16 len,csum;
  U8 status,err;
  U16 special;
};

class CE1000Tx
{
  U64 addr;
  U16 len;
  U8 cso,cmd,status,css;
  U16 special;
};

class CE1000
{
  U8 *mmio;		//registers (BAR0)
  CE1000Rx *rx;
  CE1000Tx *tx;
  U8 *rx_buf,*tx_buf;
  I64 rx_cur,tx_cur;
  U8 *mem;		//what we allocated (for alignment)
};

//The physical (DMA) address of p. TempleOS identity-maps memory; the
//Linux-guest test looks it up in /proc/self/pagemap.
U64 E1000Phys(U8 *p)
{
  U64 a=p;
//@host: { int fd=open("/proc/self/pagemap",O_RDONLY); uint64_t e=0; long ps=getpagesize();
//@host:   if (fd>=0) { if (pread(fd,&e,8,(off_t)((uintptr_t)p/ps)*8)!=8) e=0; close(fd); }
//@host:   a=((e&((1ULL<<55)-1))*ps)|((uintptr_t)p%ps); }
  return a;
}

//---- PCI configuration space (mechanism #1, ports 0xCF8/0xCFC)

U32 PciRead32(I64 bus,I64 dev,I64 fn,I64 r)
{
  OutU32(0xCF8,0x80000000|(bus<<16)|(dev<<11)|(fn<<8)|(r&0xFC));
  return InU32(0xCFC);
}

U0 PciWrite32(I64 bus,I64 dev,I64 fn,I64 r,I64 v)
{
  OutU32(0xCF8,0x80000000|(bus<<16)|(dev<<11)|(fn<<8)|(r&0xFC));
  OutU32(0xCFC,v);
}

//---- registers

U32 E1000Rd(CE1000 *e,I64 r)
{
  U32 *p=e->mmio+r,v;
//@host: v=*(volatile U32 *)p; return v;
//@host-skip-begin
  v=*p;
//@host-skip-end
  return v;
}

U0 E1000Wr(CE1000 *e,I64 r,I64 v)
{
  U32 *p=e->mmio+r;
//@host: __asm__ volatile("" ::: "memory"); *(volatile U32 *)p=v; return;
//@host-skip-begin
  *p=v;
//@host-skip-end
}

//Where to reach BAR0 (physical address phys, size bytes) from here.
U8 *E1000MapBar(I64 phys,I64 size)
{
  U8 *p=NULL;
//@host-skip-begin
  if (phys<0x100000000)	//TempleOS maps the low 4 GB uncached here (as for HPET/LAPIC)
    p=dev.uncached_alias+phys;
//@host-skip-end
//@host: int fd=open("/dev/mem",O_RDWR|O_SYNC); p=NULL;
//@host: if (fd>=0) { p=mmap(NULL,size,PROT_READ|PROT_WRITE,MAP_SHARED,fd,phys); close(fd); }
//@host: if (p==MAP_FAILED) p=NULL;
  return p;
}

//Find an 8254x on the PCI bus: TRUE and its location if found.
Bool E1000Find(I64 *bus,I64 *dev,I64 *fn)
{
  I64 b,d,f;
  U32 id;
  for (b=0;b<256;b++)
    for (d=0;d<32;d++)
      for (f=0;f<8;f++) {
        id=PciRead32(b,d,f,0);
        if ((id&0xFFFF)!=0x8086)
          goto next_e1000_1;
        switch (id>>16) {
          case 0x100E:	//82540EM (QEMU e1000)
          case 0x100F:	//82545EM
          case 0x1004:	//82543GC (VirtualBox)
          case 0x10D3:	//82574L (QEMU e1000e) - VERIFY: e1000e may need more
            *bus=b;
            *dev=d;
            *fn=f;
            return TRUE;
        }
        next_e1000_1:;
      }
  return FALSE;
}

Bool E1000Tx(CE1000 *e,U8 *frame,I64 len)
{
  CE1000Tx *t=&e->tx[e->tx_cur];
  I64 spin=0;
  if (len>E1000_BUF)
    return FALSE;
  while (t->cmd && !(t->status&1)) {	//still in flight: wait for DD
    if (++spin>1000000)
      return FALSE;
//@host: __asm__ volatile("" ::: "memory");
  }
  MemCpy(e->tx_buf+e->tx_cur*E1000_BUF,frame,len);
  if (len<60) {			//minimum ethernet frame
    MemSet(e->tx_buf+e->tx_cur*E1000_BUF+len,0,60-len);
    len=60;
  }
  t->addr=E1000Phys(e->tx_buf+e->tx_cur*E1000_BUF);
  t->len=len;
  t->status=0;
  t->cmd=0x0B;			//EOP | IFCS | RS
  e->tx_cur=(e->tx_cur+1)%E1000_NDESC;
  E1000Wr(e,E1000_TDT,e->tx_cur);
  return TRUE;
}

I64 E1000Rx(CE1000 *e,U8 *buf,I64 cap)
{
  CE1000Rx *r=&e->rx[e->rx_cur];
  I64 n;
  if (!(r->status&1))		//DD: descriptor done
    return -1;
  n=r->len;
  if (n>cap)
    n=cap;
  if (r->status&2)		//EOP: whole frame in one buffer
    MemCpy(buf,e->rx_buf+e->rx_cur*E1000_BUF,n);
  else
    n=0;			//frame bigger than a buffer: drop
  r->status=0;
  E1000Wr(e,E1000_RDT,e->rx_cur);
  e->rx_cur=(e->rx_cur+1)%E1000_NDESC;
  if (!n)
    return E1000Rx(e,buf,cap);
  return n;
}

//Find and start the card. NULL if there isn't one.
CRosNetIf *E1000Open()
{
  I64 bus,dev,fn,i,bar,addr,hi;
  U32 v;
  U8 *p;
  CE1000 *e;
  CRosNetIf *nif;
  if (!E1000Find(&bus,&dev,&fn))
    return NULL;
  e=CAlloc(sizeof(CE1000));
  bar=PciRead32(bus,dev,fn,0x10);	//BAR0: 128 KB of registers
  if (bar&1) {
    Free(e);
    return NULL;
  }
  addr=bar&0xFFFFFFF0;
  if ((bar&6)==4) {		//64-bit BAR
    hi=PciRead32(bus,dev,fn,0x14);	//through an I64: a U32 shifted by 32 is 0 in C
    addr|=hi<<32;
  }
  e->mmio=E1000MapBar(addr,0x20000);
  if (!e->mmio) {
    Free(e);
    return NULL;
  }
  v=PciRead32(bus,dev,fn,0x04);
  PciWrite32(bus,dev,fn,0x04,v|0x06);	//memory space + bus master
  E1000Wr(e,E1000_CTRL,E1000Rd(e,E1000_CTRL)|(1<<26));	//reset
  Sleep(10);
  while (E1000Rd(e,E1000_CTRL)&(1<<26))
    Yield();
  E1000Wr(e,E1000_IMC,0xFFFFFFFF);	//no interrupts: we poll
  E1000Rd(e,E1000_ICR);
  v=E1000Rd(e,E1000_CTRL);
  v=(v|(1<<6)|(1<<5))&~((1<<3)|(1<<7));	//SLU | ASDE, not LRST / ILOS
  E1000Wr(e,E1000_CTRL,v&0x7FFFFFFF);	//not PHY_RST
  for (i=0;i<128;i++)
    E1000Wr(e,E1000_MTA+4*i,0);
  //memory, page aligned so nothing the card reads crosses a page: both
  //descriptor rings in the first page, then 2 KB buffers, two a page
  e->mem=CAlloc(4096+4096+2*E1000_NDESC*E1000_BUF+4096);
  addr=e->mem;
  p=e->mem+(4096-(addr&4095));
//@host: mlock(e->mem,4096+4096+2*E1000_NDESC*E1000_BUF+4096);	//keep it where it is
  e->rx=p;
  e->tx=p+E1000_NDESC*16;
  e->rx_buf=p+4096;
  e->tx_buf=e->rx_buf+E1000_NDESC*E1000_BUF;
  for (i=0;i<E1000_NDESC;i++)
    e->rx[i].addr=E1000Phys(e->rx_buf+i*E1000_BUF);
  addr=E1000Phys(e->rx);
  E1000Wr(e,E1000_RDBAL,addr&0xFFFFFFFF);
  E1000Wr(e,E1000_RDBAH,addr>>32);
  E1000Wr(e,E1000_RDLEN,E1000_NDESC*16);
  E1000Wr(e,E1000_RDH,0);
  E1000Wr(e,E1000_RDT,E1000_NDESC-1);
  //EN | MPE (all multicast: DDS) | BAM (broadcast) | 2048 buffers | SECRC
  E1000Wr(e,E1000_RCTL,(1<<1)|(1<<4)|(1<<15)|(1<<26));
  addr=E1000Phys(e->tx);
  E1000Wr(e,E1000_TDBAL,addr&0xFFFFFFFF);
  E1000Wr(e,E1000_TDBAH,addr>>32);
  E1000Wr(e,E1000_TDLEN,E1000_NDESC*16);
  E1000Wr(e,E1000_TDH,0);
  E1000Wr(e,E1000_TDT,0);
  E1000Wr(e,E1000_TIPG,0x0060200A);
  E1000Wr(e,E1000_TCTL,(1<<1)|(1<<3)|(0x0F<<4)|(0x40<<12));	//EN | PSP | CT | COLD
  nif=CAlloc(sizeof(CRosNetIf));
  v=E1000Rd(e,E1000_RAL0);
  nif->mac[0]=v&255;
  nif->mac[1]=(v>>8)&255;
  nif->mac[2]=(v>>16)&255;
  nif->mac[3]=(v>>24)&255;
  v=E1000Rd(e,E1000_RAH0);
  nif->mac[4]=v&255;
  nif->mac[5]=(v>>8)&255;
  nif->ethernet=TRUE;
  nif->mtu=1500;
  nif->tx=&E1000Tx;
  nif->rx=&E1000Rx;
  nif->drv=e;
  Print("e1000: %02X:%02X:%02X:%02X:%02X:%02X, link %d\n",nif->mac[0],nif->mac[1],
	nif->mac[2],nif->mac[3],nif->mac[4],nif->mac[5],(E1000Rd(e,E1000_STATUS)>>1)&1);
  return nif;
}
