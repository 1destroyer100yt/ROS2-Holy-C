//Send text out of COM1 (0x3F8) so a script outside TempleOS can read it
//(tools/templeos/run.py: QEMU writes COM1 to a file).

U0 ComPutS(U8 *s)
{
  I64 spin;
  while (*s) {
    spin=0;
    while (!(InU8(0x3F8+5)&0x20) && spin<1000000)	//transmit holding empty
      spin++;
    OutU8(0x3F8,*s);
    s++;
  }
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
