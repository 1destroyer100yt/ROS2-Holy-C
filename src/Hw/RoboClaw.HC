//RoboClaw: Basicmicro RoboClaw motor controllers in packet serial
//mode. Every packet is address, command, data (big endian), CRC16
//(CCITT, polynomial 0x1021); writes are acknowledged with 0xFF, reads
//answer data + a CRC over the whole exchange.
//
//  CRosRoboClaw *rc=RoboClawOpen(RosSerialOpenCom(0x2F8,38400),0x80);
//  RoboClawSpeedM1M2(rc,1000,-1000);     //encoder counts per second
//  RoboClawEncoders(rc,&left,&right);
//
//Checked against a simulator that the official Basicmicro Python
//library talks to (tools/hwtest/roboclaw_sim.py).

#define ROBOCLAW_M1M2_DUTY	34
#define ROBOCLAW_M1M2_SPEED	37
#define ROBOCLAW_GET_ENCODERS	78
#define ROBOCLAW_RESET_ENC	20
#define ROBOCLAW_GET_VERSION	21
#define ROBOCLAW_TIMEOUT_NS	20000000	//per reply

class CRosRoboClaw
{
  CRosSerial *ser;
  I64 addr,crc;
  I64 errors;		//packets that went unanswered or failed their CRC
};

CRosRoboClaw *RoboClawOpen(CRosSerial *s,I64 addr)
{
  CRosRoboClaw *rc=CAlloc(sizeof(CRosRoboClaw));
  rc->ser=s;
  rc->addr=addr;
  return rc;
}

U0 RoboClawClose(CRosRoboClaw *rc)
{
  if (!rc)
    return;
  RosSerialClose(rc->ser);
  Free(rc);
}

U0 RoboClawCrc(CRosRoboClaw *rc,I64 b)
{
  I64 i;
  rc->crc=rc->crc^((b&255)<<8);
  for (i=0;i<8;i++) {
    if (rc->crc&0x8000)
      rc->crc=((rc->crc<<1)^0x1021)&0xFFFF;
    else
      rc->crc=(rc->crc<<1)&0xFFFF;
  }
}

//CRC16 of n bytes (for tests).
I64 RoboClawCrcOf(U8 *p,I64 n)
{
  CRosRoboClaw rc;
  I64 i;
  rc.crc=0;
  for (i=0;i<n;i++)
    RoboClawCrc(&rc,p[i]);
  return rc.crc;
}

//Put v (n bytes, big endian) into the packet at *len.
U0 RoboClawPut(CRosRoboClaw *rc,U8 *pkt,I64 *len,I64 v,I64 n)
{
  I64 i;
  for (i=n-1;i>=0;i--) {
    pkt[*len]=(v>>(8*i))&255;
    RoboClawCrc(rc,pkt[*len]);
    *len=*len+1;
  }
}

//Send a write command with data already in pkt[2..len). TRUE when acked.
Bool RoboClawSend(CRosRoboClaw *rc,U8 *pkt,I64 len)
{
  U8 ack;
  I64 tries;
  for (tries=0;tries<3;tries++) {
    RosSerialFlush(rc->ser);
    RosSerialWrite(rc->ser,pkt,len);
    if (RosSerialRead(rc->ser,&ack,1,ROBOCLAW_TIMEOUT_NS)==1 && ack==0xFF)
      return TRUE;
    rc->errors++;
  }
  return FALSE;
}

//Start a packet: address and command.
I64 RoboClawBegin(CRosRoboClaw *rc,U8 *pkt,I64 cmd)
{
  rc->crc=0;
  pkt[0]=rc->addr;
  pkt[1]=cmd;
  RoboClawCrc(rc,pkt[0]);
  RoboClawCrc(rc,pkt[1]);
  return 2;
}

U0 RoboClawEnd(CRosRoboClaw *rc,U8 *pkt,I64 *len)
{
  I64 c=rc->crc;
  pkt[*len]=(c>>8)&255;
  pkt[*len+1]=c&255;
  *len=*len+2;
}

//Both motors' speeds in encoder counts per second (signed).
Bool RoboClawSpeedM1M2(CRosRoboClaw *rc,I64 m1,I64 m2)
{
  U8 pkt[16];
  I64 len=RoboClawBegin(rc,pkt,ROBOCLAW_M1M2_SPEED);
  RoboClawPut(rc,pkt,&len,m1,4);
  RoboClawPut(rc,pkt,&len,m2,4);
  RoboClawEnd(rc,pkt,&len);
  return RoboClawSend(rc,pkt,len);
}

//Both motors' duty cycles, -32767..32767 (no encoders needed).
Bool RoboClawDutyM1M2(CRosRoboClaw *rc,I64 m1,I64 m2)
{
  U8 pkt[16];
  I64 len=RoboClawBegin(rc,pkt,ROBOCLAW_M1M2_DUTY);
  RoboClawPut(rc,pkt,&len,m1,2);
  RoboClawPut(rc,pkt,&len,m2,2);
  RoboClawEnd(rc,pkt,&len);
  return RoboClawSend(rc,pkt,len);
}

Bool RoboClawResetEncoders(CRosRoboClaw *rc)
{
  U8 pkt[8];
  I64 len=RoboClawBegin(rc,pkt,ROBOCLAW_RESET_ENC);
  RoboClawEnd(rc,pkt,&len);
  return RoboClawSend(rc,pkt,len);
}

//Read command: send address+command, then n data bytes + CRC come back.
Bool RoboClawRead(CRosRoboClaw *rc,I64 cmd,U8 *data,I64 n)
{
  U8 pkt[4],crc[2];
  I64 tries,i;
  for (tries=0;tries<3;tries++) {
    RoboClawBegin(rc,pkt,cmd);
    RosSerialFlush(rc->ser);
    RosSerialWrite(rc->ser,pkt,2);
    if (RosSerialRead(rc->ser,data,n,ROBOCLAW_TIMEOUT_NS)==n &&
	RosSerialRead(rc->ser,crc,2,ROBOCLAW_TIMEOUT_NS)==2) {
      for (i=0;i<n;i++)
        RoboClawCrc(rc,data[i]);
      if (((crc[0]<<8)|crc[1])==rc->crc)
        return TRUE;
    }
    rc->errors++;
  }
  return FALSE;
}

I64 RoboClawGet32(U8 *p)	//big endian, signed
{
  I64 v=p[0];		//through an I64 (C would shift a U8 as a 32-bit int)
  v=(v<<24)|(p[1]<<16)|(p[2]<<8)|p[3];
  if (v&0x80000000)
    v-=0x100000000;
  return v;
}

//Both encoder counts (signed).
Bool RoboClawEncoders(CRosRoboClaw *rc,I64 *m1,I64 *m2)
{
  U8 d[8];
  if (!RoboClawRead(rc,ROBOCLAW_GET_ENCODERS,d,8))
    return FALSE;
  *m1=RoboClawGet32(d);
  *m2=RoboClawGet32(d+4);
  return TRUE;
}

//Firmware version text (new string), or NULL. It ends with a NUL,
//then the CRC.
U8 *RoboClawVersion(CRosRoboClaw *rc)
{
  U8 pkt[4],buf[48],crc[2];
  I64 n=0;
  RoboClawBegin(rc,pkt,ROBOCLAW_GET_VERSION);
  RosSerialFlush(rc->ser);
  RosSerialWrite(rc->ser,pkt,2);
  while (n<48 && RosSerialRead(rc->ser,buf+n,1,ROBOCLAW_TIMEOUT_NS)==1) {
    RoboClawCrc(rc,buf[n]);
    if (!buf[n])
      break;
    n++;
  }
  if (n>=48 || RosSerialRead(rc->ser,crc,2,ROBOCLAW_TIMEOUT_NS)!=2 ||
      ((crc[0]<<8)|crc[1])!=rc->crc) {
    rc->errors++;
    return NULL;
  }
  return StrNew(buf);
}
