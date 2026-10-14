//Unit tests for src/Core. Run with RosTestsRun (see Run.HC).

U0 TestF32()
{
  RosTestGroup("F32");
  RosTestI64("1.0",RosF64ToF32Bits(1.0),0x3F800000);
  RosTestI64("-2.0",RosF64ToF32Bits(-2.0),0xC0000000);
  RosTestI64("0.1 rounds",RosF64ToF32Bits(0.1),0x3DCCCCCD);
  RosTestI64("+0",RosF64ToF32Bits(0.0),0);
  RosTestI64("-0",RosF64ToF32Bits(RosBitsToF64(0x8000000000000000)),0x80000000);
  RosTestI64("max float",RosF64ToF32Bits(RosTestLit("3.4028234663852886e38")),0x7F7FFFFF);
  RosTestI64("overflow -> inf",RosF64ToF32Bits(RosTestLit("1e39")),0x7F800000);
  RosTestI64("-overflow -> -inf",RosF64ToF32Bits(RosTestLit("-1e39")),0xFF800000);
  RosTestI64("rounds up to inf",RosF64ToF32Bits(RosTestLit("3.4028235677973366e38")),0x7F800000);
  RosTestI64("smallest subnormal",RosF64ToF32Bits(RosTestLit("1.401298464324817e-45")),1);
  RosTestI64("half smallest -> 0 (ties even)",RosF64ToF32Bits(RosTestLit("7.006492321624085e-46")),0);
  RosTestI64("tiny -> 0",RosF64ToF32Bits(RosTestLit("1e-50")),0);
  RosTestI64("largest subnormal",RosF64ToF32Bits(RosTestLit("1.1754942106924411e-38")),0x7FFFFF);
  RosTestI64("smallest normal",RosF64ToF32Bits(RosTestLit("1.1754943508222875e-38")),0x800000);
  RosTestI64("inf",RosF64ToF32Bits(RosBitsToF64(ROS_F64_INF_BITS)),0x7F800000);
  RosTestTrue("nan stays nan",
	(RosF64ToF32Bits(RosBitsToF64(ROS_F64_NAN_BITS))&0x7FC00000)==0x7FC00000);
  RosTestF64("back 1.0",RosF32BitsToF64(0x3F800000),1.0,0.0);
  RosTestF64("back 0.1",RosF32BitsToF64(0x3DCCCCCD),RosTestLit("0.10000000149011612"),0.0);
  RosTestF64("back subnormal",RosF32BitsToF64(1),RosTestLit("1.401298464324817e-45"),0.0);
  RosTestF64("back -max",RosF32BitsToF64(0xFF7FFFFF),RosTestLit("-3.4028234663852886e38"),0.0);
  RosTestI64("back inf",RosF64ToBits(RosF32BitsToF64(0x7F800000)),ROS_F64_INF_BITS);
  RosTestI64("back -0",RosF64ToBits(RosF32BitsToF64(0x80000000)),0x8000000000000000);
  RosTestF64("F32Round",RosF32Round(RosTestLit("0.1")),RosTestLit("0.10000000149011612"),0.0);
}

U0 TestMem()
{
  CRosPool *p=RosPoolNew(20,4);
  U8 *a,*b,*c,*blocks[10];
  I64 i;
  Bool ok=TRUE;
  RosTestGroup("Mem");
  RosTestI64("block size rounded",p->block_size,24);
  a=RosPoolAlloc(p);
  b=RosPoolAlloc(p);
  RosTestTrue("distinct blocks",a!=b);
  RosTestI64("used",p->used,2);
  a[0]=55;
  RosPoolFree(p,a);
  c=RosPoolAlloc(p);
  RosTestTrue("reuses freed block",c==a);
  RosTestI64("comes back zeroed",c[0],0);
  for (i=0;i<10;i++)
    blocks[i]=RosPoolAlloc(p);
  RosTestI64("grew to 3 chunks",p->total,12);
  RosTestI64("used after growth",p->used,12);
  for (i=0;i<10;i++)
    if (!RosPoolOwns(p,blocks[i]))
      ok=FALSE;
  RosTestTrue("owns all blocks",ok);
  RosTestTrue("not owns middle of block",!RosPoolOwns(p,blocks[0]+1));
  p->debug=TRUE;
  RosTestTrue("debug refuses foreign ptr",!RosPoolFree(p,blocks[0]+1));
  for (i=0;i<10;i++)
    RosPoolFree(p,blocks[i]);
  RosTestI64("used after frees",p->used,2);
  RosPoolDel(p);
}

U0 TestVec()
{
  CRosVec *v=RosVecNew(sizeof(I64));
  I64 i,x,sum=0;
  RosTestGroup("Vec");
  for (i=0;i<100;i++)
    RosVecPushI64(v,i);
  RosTestI64("len",v->len,100);
  RosTestTrue("cap grew",v->cap>=100);
  for (i=0;i<100;i++)
    sum+=RosVecGetI64(v,i);
  RosTestI64("sum",sum,4950);
  RosTestTrue("out of range is NULL",RosVecAt(v,100)==NULL && RosVecAt(v,-1)==NULL);
  x=-7;
  RosVecInsert(v,0,&x);
  RosTestI64("insert front",RosVecGetI64(v,0),-7);
  RosTestI64("shifted",RosVecGetI64(v,1),0);
  RosVecRemove(v,0);
  RosTestI64("remove front",RosVecGetI64(v,0),0);
  RosVecRemove(v,50);
  RosTestI64("remove middle",RosVecGetI64(v,50),51);
  RosTestTrue("pop",RosVecPop(v,&x));
  RosTestI64("popped value",x,99);
  RosTestI64("len now",v->len,98);
  RosTestTrue("insert at end",RosVecInsert(v,98,&x));
  RosTestI64("last",RosVecGetI64(v,98),99);
  RosTestTrue("bad insert",!RosVecInsert(v,1000,&x));
  RosVecClear(v);
  RosTestTrue("pop empty",!RosVecPop(v,&x));
  RosVecDel(v);
}

//Random F64 bits -> shortest digits (Dtoa) -> RosStrToF64: same bits.
U0 TestF64RoundTrip()
{
  I64 i,bad=0;
  U64 u;
  F64 f;
  CRosStrBuf *sb=RosStrBufNew();
  for (i=0;i<3000;i++) {
    u=RandU64()&0x7FFFFFFFFFFFFFFF;
    if ((u>>52)==0x7FF)
      u=u>>12;		//not inf/nan: try a small number instead
    RosStrBufClear(sb);
    RosStrBufAddF64(sb,RosBitsToF64(u));
    if (!RosStrToF64(sb->buf,&f) || RosF64ToBits(f)!=u) {
      if (!bad)
        Print("     %X -> %s -> %X\n",u,sb->buf,RosF64ToBits(f));
      bad++;
    }
  }
  RosStrBufDel(sb);
  RosTestI64("f64 round trips (3000 random)",bad,0);
}

U0 TestStr()
{
  CRosStrBuf *sb=RosStrBufNew();
  CRosVec *parts;
  U8 *s;
  I64 n;
  F64 f;
  RosTestGroup("Str");
  RosStrBufAdd(sb,"abc");
  RosStrBufAddC(sb,'-');
  RosStrBufAddI64(sb,-1234);
  RosStrBufAddC(sb,' ');
  RosStrBufAddI64(sb,0);
  RosStrBufAddC(sb,' ');
  RosStrBufAddI64(sb,0x8000000000000000);
  for (n=0;n<50;n++)
    RosStrBufAddC(sb,'x');
  s=RosStrBufTake(sb);
  RosTestTrue("builder prefix",RosStrStartsWith(s,"abc--1234 0 -9223372036854775808xxx"));
  RosTestI64("builder len",StrLen(s),32+50);
  Free(s);

  RosTestTrue("eq",RosStrEq("a","a") && !RosStrEq("a","b") && RosStrEq(NULL,NULL) && !RosStrEq("a",NULL));
  RosTestTrue("ends with",RosStrEndsWith("talker.HC",".HC") && !RosStrEndsWith("HC","talker.HC"));
  RosTestI64("findc",RosStrFindC("a/b/c",'/'),1);
  RosTestI64("findc missing",RosStrFindC("abc",'/'),-1);
  RosTestI64("find",RosStrFind("rt/chatter","chat"),3);
  RosTestI64("find missing",RosStrFind("rt/chatter","zz"),-1);
  s=RosStrSub("hello world",6,100);
  RosTestStr("sub clipped",s,"world");
  Free(s);
  s=RosStrTrim("  \t spaced out \n");
  RosTestStr("trim",s,"spaced out");
  Free(s);
  s=RosStrTrim("   ");
  RosTestStr("trim all",s,"");
  Free(s);

  parts=RosStrSplit("/a//b/",'/');
  RosTestI64("split count",parts->len,5);
  RosTestStr("split 0",RosVecGetI64(parts,0),"");
  RosTestStr("split 1",RosVecGetI64(parts,1),"a");
  RosTestStr("split 2",RosVecGetI64(parts,2),"");
  RosTestStr("split 3",RosVecGetI64(parts,3),"b");
  RosStrVecDel(parts);

  RosTestTrue("i64 ok",RosStrToI64(" -42 ",&n));
  RosTestI64("i64 value",n,-42);
  RosTestTrue("hex",RosStrToI64("0x7fFF",&n) && n==0x7FFF);
  RosTestTrue("max",RosStrToI64("9223372036854775807",&n) && n==0x7FFFFFFFFFFFFFFF);
  RosTestTrue("min",RosStrToI64("-9223372036854775808",&n) && n==0x8000000000000000);
  RosTestTrue("overflow",!RosStrToI64("9223372036854775808",&n));
  RosTestTrue("junk",!RosStrToI64("12a",&n) && !RosStrToI64("",&n) && !RosStrToI64("-",&n));

  RosTestTrue("f64 ok",RosStrToF64("3.25",&f));
  RosTestI64("f64 value",RosF64ToBits(f),0x400A000000000000);	//3.25
  RosTestTrue("f64 exp",RosStrToF64("-1.5e3",&f) && RosF64ToBits(f)==0xC097700000000000);
  RosTestTrue("f64 neg exp",RosStrToF64("25e-2",&f) && RosF64ToBits(f)==0x3FD0000000000000);
  RosTestTrue("f64 0.1",RosStrToF64("0.1",&f) && RosF64ToBits(f)==0x3FB999999999999A);
  //correctly rounded, also exactly halfway, at the edges and past 40 digits
  RosTestI64("exact 9007199254740993",RosF64ToBits(RosTestLit("9007199254740993")),0x4340000000000000);
  RosTestI64("exact 9007199254740995",RosF64ToBits(RosTestLit("9007199254740995")),0x4340000000000002);
  RosTestI64("exact 2.4703282292062327e-324",RosF64ToBits(RosTestLit("2.4703282292062327e-324")),0x0);
  RosTestI64("exact 2.4703282292062328e-324",RosF64ToBits(RosTestLit("2.4703282292062328e-324")),0x1);
  RosTestI64("exact 1.7976931348623157e308",RosF64ToBits(RosTestLit("1.7976931348623157e308")),0x7FEFFFFFFFFFFFFF);
  RosTestI64("exact 1.7976931348623158e308",RosF64ToBits(RosTestLit("1.7976931348623158e308")),0x7FEFFFFFFFFFFFFF);
  RosTestI64("exact 1.7976931348623159e308",RosF64ToBits(RosTestLit("1.7976931348623159e308")),0x7FF0000000000000);
  RosTestI64("exact 2.2250738585072011e-308",RosF64ToBits(RosTestLit("2.2250738585072011e-308")),0xFFFFFFFFFFFFF);
  RosTestI64("exact 0.1",RosF64ToBits(RosTestLit("0.1")),0x3FB999999999999A);
  RosTestI64("exact 123456789012345678901234",RosF64ToBits(RosTestLit("123456789012345678901234567890123456789012345678901234567890e-30")),0x45F8EE90FF6C373E);
  RosTestI64("exact 1e23",RosF64ToBits(RosTestLit("1e23")),0x44B52D02C7E14AF6);
  RosTestI64("exact 8.98846567431158e307",RosF64ToBits(RosTestLit("8.98846567431158e307")),0x7FE0000000000000);
  TestF64RoundTrip();
  RosTestTrue("f64 int",RosStrToF64("7",&f) && f==7.0);
  RosTestTrue("f64 .5",RosStrToF64(".5",&f) && f==0.5);
  RosTestTrue("f64 inf",RosStrToF64("-inf",&f) && RosF64ToBits(f)==0xFFF0000000000000);
  RosTestTrue("f64 junk",!RosStrToF64("1.2.3",&f) && !RosStrToF64("e5",&f) && !RosStrToF64("1e",&f));
}

U0 TestRing()
{
  CRosRing *r=RosRingNew(sizeof(I64),5);
  I64 i,x,ok=0,*pk;
  RosTestGroup("Ring");
  RosTestI64("cap rounded to pow2",r->cap,8);
  RosTestTrue("empty",RosRingEmpty(r) && RosRingPeek(r)==NULL);
  for (i=0;i<8;i++)
    if (RosRingPush(r,&i))
      ok++;
  RosTestI64("8 pushes fit",ok,8);
  RosTestTrue("full",RosRingFull(r));
  RosTestTrue("9th refused",!RosRingPush(r,&i));
  RosTestI64("dropped",r->dropped,1);
  RosTestTrue("pop",RosRingPop(r,&x) && x==0);
  pk=RosRingPeek(r);
  RosTestI64("peek",*pk,1);
  //wrap around many times
  ok=TRUE;
  for (i=8;i<1000;i++) {
    RosRingPush(r,&i);
    if (!RosRingPop(r,&x) || x!=i-7)
      ok=FALSE;
  }
  RosTestTrue("fifo order through wrap",ok);
  RosTestI64("count",RosRingCount(r),7);
  RosRingPushOverwrite(r,&i);
  RosRingPushOverwrite(r,&i);
  RosTestI64("overwrite keeps cap",RosRingCount(r),8);
  RosTestTrue("oldest dropped",RosRingPop(r,&x) && x==994);
  RosRingDel(r);
}

U0 TestMap()
{
  CRosMap *m=RosMapNew();
  CRosStrBuf *sb;
  U8 *k,*v;
  I64 i,it=0,seen=0;
  Bool ok=TRUE;
  RosTestGroup("Map");
  RosMapSet(m,"/chatter",1);
  RosMapSet(m,"/cmd_vel",2);
  RosTestI64("get",RosMapGet(m,"/chatter"),1);
  RosTestTrue("missing",RosMapGet(m,"/nope")==NULL && !RosMapHas(m,"/nope"));
  RosMapSet(m,"/chatter",3);
  RosTestI64("replace",RosMapGet(m,"/chatter"),3);
  RosTestI64("count",m->count,2);
  for (i=0;i<1000;i++) {
    sb=RosStrBufNew();
    RosStrBufAdd(sb,"/topic_");
    RosStrBufAddI64(sb,i);
    k=RosStrBufTake(sb);
    RosMapSet(m,k,i+100);
    Free(k);
  }
  RosTestI64("count big",m->count,1002);
  for (i=0;i<1000;i+=2) {
    sb=RosStrBufNew();
    RosStrBufAdd(sb,"/topic_");
    RosStrBufAddI64(sb,i);
    k=RosStrBufTake(sb);
    if (!RosMapRemove(m,k))
      ok=FALSE;
    Free(k);
  }
  RosTestTrue("removes",ok);
  RosTestI64("count after remove",m->count,502);
  RosTestI64("odd still there",RosMapGet(m,"/topic_999"),1099);
  RosTestTrue("even gone",!RosMapHas(m,"/topic_998"));
  RosTestTrue("remove twice",!RosMapRemove(m,"/topic_998"));
  RosMapSet(m,"/topic_998",7);
  RosTestI64("reinsert after delete",RosMapGet(m,"/topic_998"),7);
  while (RosMapNext(m,&it,&k,&v))
    seen++;
  RosTestI64("iterate all",seen,503);
  RosTestI64("fnv-1a",RosHashStr("a"),0xAF63DC4C8601EC8C);
  RosMapDel(m);
}

U0 TestTime()
{
  I64 a,b,t;
  CRosRate *r;
  RosTestGroup("Time");
  a=RosTimeSteadyNs();
  RosSleepNs(3*ROS_NS_PER_MS);
  b=RosTimeSteadyNs();
  RosTestTrue("steady moves forward",b-a>=3*ROS_NS_PER_MS);
  RosTestTrue("sleep not way too long",b-a<500*ROS_NS_PER_MS);
  t=RosTimeMake(-2,500000000);
  RosTestI64("sec of -1.5s",RosTimeSec(t),-2);
  RosTestI64("nsec of -1.5s",RosTimeNsec(t),500000000);
  RosTestI64("sec",RosTimeSec(RosTimeMake(12,34)),12);
  RosTestI64("nsec",RosTimeNsec(RosTimeMake(12,34)),34);
  RosTestF64("to sec",RosTimeToSec(1500000000),1.5,0.0);
  RosTestI64("from sec",RosTimeFromSec(0.25),250000000);
  RosTimeSetSystemNs(RosTimeMake(1700000000,0));
  t=RosTimeSystemNs();
  RosTestTrue("system time set",RosTimeSec(t)>=1700000000 && RosTimeSec(t)<1700000010);
  RosClockSetSim(42);
  RosTestI64("sim time",RosClockNow(ROS_CLOCK_ROS),42);
  RosTestTrue("steady ignores sim",RosClockNow(ROS_CLOCK_STEADY)!=42);
  RosClockSimOff();
  RosTestTrue("sim off",RosClockNow(ROS_CLOCK_ROS)!=42);
  r=RosRateNew(200.0);
  RosTestI64("rate period",r->period_ns,5000000);
  a=RosTimeSteadyNs();
  for (t=0;t<4;t++)
    RosRateSleep(r);
  b=RosTimeSteadyNs();
  RosTestTrue("4 ticks at 200 Hz ~20ms",b-a>=15*ROS_NS_PER_MS && b-a<500*ROS_NS_PER_MS);
  RosRateDel(r);
}

U0 TestLog()
{
  I64 before;
  RosTestGroup("Log");
  ros_log_quiet=TRUE;
  before=ros_log_counts[ROS_LOG_WARN];
  RosTestTrue("warn passes",RosLogWrite(ROS_LOG_WARN,"test","w"));
  RosTestI64("counted",ros_log_counts[ROS_LOG_WARN],before+1);
  RosTestTrue("debug filtered",!RosLogWrite(ROS_LOG_DEBUG,"test","d"));
  ros_log_level=ROS_LOG_DEBUG;
  RosTestTrue("debug passes when enabled",RosLogWrite(ROS_LOG_DEBUG,"test","d"));
  ros_log_level=ROS_LOG_INFO;
  RosTestStr("level name",RosLogLevelName(ROS_LOG_ERROR),"ERROR");
  ros_log_quiet=FALSE;
}

U0 TestCore()
{
  TestF32();
  TestMem();
  TestVec();
  TestStr();
  TestRing();
  TestMap();
  TestTime();
  TestLog();
}
