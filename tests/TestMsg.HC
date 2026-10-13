//Unit tests for src/Msg. TestGolden.HC and TestLayout.HC are
//generated; this file has the hand-written ones.

U8 RosHexNib(U8 c)
{
  if (c>='a')
    return c-'a'+10;
  if (c>='A')
    return c-'A'+10;
  return c-'0';
}

//"0a1b" -> new byte buffer, *len=2.
U8 *RosHexDecode(U8 *hex,I64 *len)
{
  I64 n=StrLen(hex)/2,i;
  U8 *b=MAlloc(n+1);
  for (i=0;i<n;i++)
    b[i]=(RosHexNib(hex[2*i])<<4)|RosHexNib(hex[2*i+1]);
  *len=n;
  return b;
}

U8 *RosHexEncode(U8 *b,I64 n)
{
  CRosStrBuf *sb=RosStrBufNew();
  U8 *d="0123456789abcdef";
  I64 i;
  for (i=0;i<n;i++) {
    RosStrBufAddC(sb,d[b[i]>>4]);
    RosStrBufAddC(sb,d[b[i]&15]);
  }
  return RosStrBufTake(sb);
}

//Decode golden bytes, check the YAML, re-encode (must match), and
//check a deep copy encodes the same too.
U0 RosTestGolden(U8 *name,U8 *hex,U8 *yaml)
{
  CRosType *t=RosTypeFind(name);
  U8 *data,*m,*m2,*y,*h;
  I64 len;
  CRosBuf *b;
  CRosStrBuf *what;
  if (!t) {
    RosTestFailed(name);
    Print("     type not loaded\n");
    return;
  }
  what=RosStrBufNew();
  RosStrBufAdd(what,name);
  data=RosHexDecode(hex,&len);
  m=RosMsgNew(t);
  if (!RosCdrDeser(t,data,len,m)) {
    RosStrBufAdd(what," decode");
    RosTestFailed(what->buf);
    Print("     %s\n",ros_msg_error);
  } else {
    y=RosMsgToYaml(t,m);
    RosStrBufAdd(what," yaml");
    RosTestStr(what->buf,y,yaml);
    Free(y);
    b=RosBufNew();
    RosCdrSer(t,m,b);
    h=RosHexEncode(b->data,b->len);
    RosStrBufClear(what);
    RosStrBufAdd(what,name);
    RosStrBufAdd(what," re-encode");
    RosTestStr(what->buf,h,hex);
    Free(h);
    RosBufDel(b);
    m2=RosMsgDup(t,m);
    RosMsgDel(t,m);	//the copy must not share anything with m
    m=NULL;
    b=RosBufNew();
    RosCdrSer(t,m2,b);
    h=RosHexEncode(b->data,b->len);
    RosStrBufClear(what);
    RosStrBufAdd(what,name);
    RosStrBufAdd(what," copy");
    RosTestStr(what->buf,h,hex);
    Free(h);
    RosBufDel(b);
    RosMsgDel(t,m2);
  }
  RosMsgDel(t,m);
  Free(data);
  RosStrBufDel(what);
}

U0 TestF64Fmt1(F64 x,U8 *want)
{
  CRosStrBuf *sb=RosStrBufNew();
  RosStrBufAddF64(sb,x);
  RosTestStr("format F64",sb->buf,want);
  RosStrBufDel(sb);
}

U0 TestF64Fmt()
{
  RosTestGroup("F64 format");
  TestF64Fmt1(1.0,"1.0");
  TestF64Fmt1(0.1,"0.1");
  TestF64Fmt1(-2.5,"-2.5");
  TestF64Fmt1(100.0,"100.0");
  TestF64Fmt1(123.456,"123.456");
  TestF64Fmt1(0.0001,"0.0001");
  TestF64Fmt1(0.00001,"1e-05");
  TestF64Fmt1(2.5e-08,"2.5e-08");
  TestF64Fmt1(1e16,"1e+16");
  TestF64Fmt1(1234567890123456.0,"1234567890123456.0");
  TestF64Fmt1(1.5e300,"1.5e+300");
  TestF64Fmt1(3.141592653589793,"3.141592653589793");
  TestF64Fmt1(0.30000000000000004,"0.30000000000000004");
  TestF64Fmt1(RosF32Round(0.1),"0.10000000149011612");
  TestF64Fmt1(RosBitsToF64(0x8000000000000000),"-0.0");
  TestF64Fmt1(RosBitsToF64(ROS_F64_INF_BITS),"inf");
  TestF64Fmt1(RosBitsToF64(ROS_F64_NAN_BITS),"nan");
}

U0 TestMsgParse()
{
  CRosType *t,*p;
  CRosField *f;
  CRosConst *c;
  CRosSrvType *st;
  U8 *m,*s;
  I64 *ip;
  RosTestGroup("Msg parse");
  t=RosMsgDefine("test_msgs/msg/Prims",
	"# comment line\nbool b true\nint8 i8 -5\nuint8 u8 200 # trailing\nfloat32 f 0.5\nstring s 'it''s # not a comment'\nstring<=4 small\nint32[3] fixed [1, 2, 3]\nfloat64[] seq [1.5, -2.0]\nuint8[<=4] bytes\nint32 ANSWER=42\nstring NAME=\"bob\"\nfloat64 HALF = 0.5\n");
  RosTestTrue("define ok",t!=NULL);
  if (!t) {
    Print("     %s\n",ros_msg_error);
    return;
  }
  RosTestI64("fields",t->fields->len,9);
  RosTestI64("consts",t->consts->len,3);
  c=RosTypeConst(t,"ANSWER");
  RosTestI64("const int",c->i,42);
  RosTestStr("const string",RosTypeConst(t,"NAME")->s,"bob");
  RosTestF64("const float w/ spaces",RosTypeConst(t,"HALF")->f,0.5,0.0);
  f=RosTypeField(t,"small");
  RosTestI64("string bound",f->str_bound,4);
  f=RosTypeField(t,"fixed");
  RosTestTrue("fixed array",f->arr==ROS_ARR_FIXED && f->arr_len==3 && f->size==24);
  f=RosTypeField(t,"bytes");
  RosTestTrue("bounded byte seq",f->arr==ROS_ARR_SEQ && f->arr_len==4 && f->stride==1);
  RosTestTrue("owns memory",t->owns_mem);
  RosTestI64("size",t->size,6*8+3*24);
  m=RosMsgNew(t);
  s=RosMsgToYaml(t,m);
  RosTestStr("defaults",s,"b: true\ni8: -5\nu8: 200\nf: 0.5\ns: 'it''s # not a comment'\nsmall: ''\nfixed:\n- 1\n- 2\n- 3\nseq:\n- 1.5\n- -2.0\nbytes: []\n");
  Free(s);

  RosTestTrue("set int",RosMsgSet(t,m,"i8","-128"));
  RosTestTrue("set out of range",!RosMsgSet(t,m,"i8","128"));
  RosTestTrue("set bool",RosMsgSet(t,m,"b","false"));
  RosTestTrue("set float32 rounds",RosMsgSet(t,m,"f","0.1"));
  s=RosMsgGet(t,m,"f");
  RosTestStr("float32 value",s,"0.10000000149011612");
  Free(s);
  RosTestTrue("bound ok",RosMsgSet(t,m,"small","abcd"));
  RosTestTrue("bound too long",!RosMsgSet(t,m,"small","abcde"));
  RosTestTrue("set list",RosMsgSet(t,m,"seq","[1, 2, 3, 4]"));
  s=RosMsgGet(t,m,"seq[3]");
  RosTestStr("list item",s,"4.0");
  Free(s);
  RosTestTrue("grow by index",RosMsgSet(t,m,"bytes[2]","7"));
  s=RosMsgGet(t,m,"bytes");
  RosTestStr("grown",s,"bytes:\n- 0\n- 0\n- 7\n");
  Free(s);
  RosTestTrue("bounded seq refuses index 4",!RosMsgSet(t,m,"bytes[4]","1"));
  RosTestTrue("fixed refuses index 3",!RosMsgSet(t,m,"fixed[3]","1"));
  RosTestTrue("fixed wrong count",!RosMsgSet(t,m,"fixed","[1, 2]"));
  RosTestTrue("bad field",!RosMsgSet(t,m,"nope","1"));
  RosTestTrue("bad number",!RosMsgSet(t,m,"u8","12x"));
  ip=m+RosTypeField(t,"i8")->offset;
  RosTestI64("raw storage",*ip,-128);
  RosMsgDel(t,m);

  p=RosMsgDefine("test_msgs/Outer","Prims one\nPrims[2] two\nPrims[] many\nHeader h\n");
  RosTestTrue("nested define",p!=NULL);
  if (p) {
    m=RosMsgNew(p);
    RosTestTrue("deep set",RosMsgSet(p,m,"many[1].fixed[2]","9"));
    RosTestTrue("deep set 2",RosMsgSet(p,m,"two[1].s","x"));
    RosTestTrue("header",RosMsgSet(p,m,"h.frame_id","map"));
    s=RosMsgGet(p,m,"many[1].fixed");
    RosTestStr("deep get",s,"fixed:\n- 1\n- 2\n- 9\n");
    Free(s);
    s=RosMsgGet(p,m,"many[0].i8");
    RosTestStr("grown element has defaults",s,"-5");
    Free(s);
    RosMsgDel(p,m);
  }
  RosTestTrue("unknown type",!RosMsgDefine("test_msgs/Bad","Nope x\n"));
  RosTestTrue("error says why",RosStrFind(ros_msg_error,"unknown type")>=0);
  RosTestTrue("wstring refused",!RosMsgDefine("test_msgs/Bad2","wstring w\n"));
  RosTestTrue("bad default",!RosMsgDefine("test_msgs/Bad3","int8 x 300\n"));
  RosTestTrue("bad array",!RosMsgDefine("test_msgs/Bad4","int8[0] x\n"));
  RosTestTrue("same name returns same type",RosMsgDefine("test_msgs/msg/Prims","")==t);
  st=RosSrvDefine("test_msgs/srv/Add","int64 a\nint64 b\n---\nint64 sum\n");
  RosTestTrue("srv",st && st->req->fields->len==2 && st->resp->fields->len==1);
  RosTestTrue("srv names",st && RosTypeFind("test_msgs/srv/Add_Response")==st->resp);
  st=RosSrvDefine("test_msgs/srv/Nothing","---\n");
  RosTestTrue("empty srv gets placeholder",st && st->req->fields->len==1 && st->req->size==8);
}

U0 TestCdrSafety()
{
  CRosType *t=RosTypeFind("std_msgs/msg/String");
  U8 *m=RosMsgNew(t),*d;
  I64 len;
  RosTestGroup("CDR safety");
  d=RosHexDecode("0001000003000000686900",&len);
  RosTestTrue("good",RosCdrDeser(t,d,len,m));
  RosTestTrue("cut short",!RosCdrDeser(t,d,len-1,m));
  RosTestTrue("no header",!RosCdrDeser(t,d,3,m));
  Free(d);
  d=RosHexDecode("00010000ffffff7f6869",&len);
  RosTestTrue("huge string length",!RosCdrDeser(t,d,len,m));
  Free(d);
  d=RosHexDecode("000100000300000068696a",&len);
  RosTestTrue("missing NUL",!RosCdrDeser(t,d,len,m));
  Free(d);
  d=RosHexDecode("0000000000000003686900",&len);
  RosTestTrue("big endian",RosCdrDeser(t,d,len,m));
  RosTestStr("big endian value",RosElemGetS(m),"hi");
  Free(d);
  RosMsgDel(t,m);
  t=RosTypeFind("sensor_msgs/msg/LaserScan");
  m=RosMsgNew(t);
  d=RosHexDecode("00010000000000000000000000000000000000000000000000000000000000000000000000000000ffffff7f",&len);
  RosTestTrue("huge sequence count",!RosCdrDeser(t,d,len,m));
  Free(d);
  RosMsgDel(t,m);
}

U0 TestMsg()
{
  TestF64Fmt();
  TestMsgParse();
  TestCdrSafety();
}
