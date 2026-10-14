//Send text out of COM1 (0x3F8) so a script outside TempleOS can read it
//(tools/templeos/run.py: QEMU writes COM1 to a file).

I64 com_lock=0;	//one writer at a time (ComTailTask runs alongside)

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

//---- live: a task that sends the terminal's new text out of COM1 as it
//appears (tools/templeos/hw.py waits for READY and RESULT lines).
CTask *com_tail_watch=NULL,*com_tail_task=NULL;
I64 com_tail_sent=0;

U0 ComTailTask(U8 *data)
{
  U8 *s;
  I64 n;
  while (TRUE) {
    s=DocSave(DocPut(com_tail_watch),NULL);
    n=StrLen(s);
    if (n<com_tail_sent)	//cleared: start over
      com_tail_sent=0;
    if (n>com_tail_sent) {
      ComPutS(s+com_tail_sent);
      com_tail_sent=n;
    }
    Free(s);
    Sleep(250);
  }
}

U0 ComTailStart()
{
  U8 *s=DocSave(DocPut(Fs),NULL);
  com_tail_watch=Fs;
  com_tail_sent=StrLen(s);
  Free(s);
  com_tail_task=Spawn(&ComTailTask,NULL,"ComTail");
}

U0 ComTailStop()
{
  if (com_tail_task)
    Kill(com_tail_task);
  com_tail_task=NULL;
}
