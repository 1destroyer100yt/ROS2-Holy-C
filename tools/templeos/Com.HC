//Send text out of COM1 (0x3F8) so a script outside TempleOS can read it
//(tools/templeos/run.py: QEMU writes COM1 to a file).

I64 com_lock=0;	//one writer at a time (tasks print alongside)

U0 ComPutS(U8 *s)
{
  I64 spin;
  while (LBts(&com_lock,0))
    Yield;
  while (*s) {
    spin=0;
    while (!(InU8(0x3F8+5)&0x20) && spin<1000000)	//transmit holding empty
      spin++;
    OutU8(0x3F8,*s);
    s++;
  }
  LBtr(&com_lock,0);
}

//The terminal's text so far, then an end marker.
U0 ComDump()
{
  U8 *s=DocSave(DocPut,NULL);
  ComPutS("\n@@BEGIN@@\n");
  ComPutS(s);
  ComPutS("\n@@END@@\n");
  Free(s);
}

//---- live: everything printed also goes out of COM1 as it's printed
//(tools/templeos/hw.py waits for READY and RESULT lines). A hook early in
//TempleOS's output chain (KeyDevAdd) that passes everything on.
CKeyDevEntry *com_echo=NULL;

Bool ComEchoPutKey(I64 ch,I64 sc)
{
  U8 buf[2];
  if (!sc && ch>0 && ch<256) {	//printed, not a key going to the editor
    buf[0]=ch;
    buf[1]=0;
    ComPutS(buf);
  }
  return FALSE;
}

Bool ComEchoPutS(U8 *st)
{
  ComPutS(st);
  return FALSE;
}

U0 ComTailStart()
{
  if (!com_echo)
    com_echo=KeyDevAdd(&ComEchoPutKey,&ComEchoPutS,0x10000000);
}

U0 ComTailStop()
{
  if (com_echo)
    KeyDevRem(com_echo);
  com_echo=NULL;
}
