//Tests for src/Core/Sha256.HC and src/Msg/Hash.HC.

U0 TestSha256()
{
  U8 *s,*buf;
  RosTestGroup("SHA-256");
  s=RosSha256Hex("",0);
  RosTestStr("empty",s,"e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
  Free(s);
  s=RosSha256Hex("abc",3);
  RosTestStr("abc",s,"ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
  Free(s);
  buf="abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq";	//56 bytes: two blocks of padding
  s=RosSha256Hex(buf,StrLen(buf));
  RosTestStr("448 bits",s,"248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1");
  Free(s);
  buf=MAlloc(1000000);
  MemSet(buf,'a',1000000);
  s=RosSha256Hex(buf,1000000);
  RosTestStr("a million a",s,"cdc76e5c9914fb9281a1c7e284d73e67f1809a48a497200e046d39ccc7112cd0");
  Free(s);
  Free(buf);
}

U0 TestTypeHash()
{
  CRosSrvType *st=RosSrvFind("example_interfaces/srv/AddTwoInts");
  U8 *h;
  RosTestGroup("Type hashes");
  h=RosTypeHash(ros_t_geometry_msgs_Twist);
  RosTestStr("Twist",h,"RIHS01_9c45bf16fe0983d80e3cfe750d6835843d265a9a6c46bd2e609fcddde6fb8d2a");
  RosTestTrue("cached",RosTypeHash(ros_t_geometry_msgs_Twist)==h);
  h=RosSrvTypeHash(st);
  RosTestTrue("service hash",h && RosStrStartsWith(h,"RIHS01_") && StrLen(h)==71);
  RosTestTrue("service hash differs from request",StrCmp(h,RosTypeHash(st->req))!=0);
  TestHashGolden();
}

U0 TestUserData()
{
  CRosBuf *b=RosBufNew();
  CRtpsIn in;
  U8 *s;
  RosTestGroup("USER_DATA keys");
  RtpsW32(b,StrLen("enclave=/;typehash=RIHS01_ab;x=1"));
  RtpsWBytes(b,"enclave=/;typehash=RIHS01_ab;x=1",StrLen("enclave=/;typehash=RIHS01_ab;x=1"));
  RtpsInInit(&in,b->data,b->len,FALSE);
  s=RtpsPlReadUserKey(&in,"typehash");
  RosTestStr("typehash",s,"RIHS01_ab");
  Free(s);
  RtpsInInit(&in,b->data,b->len,FALSE);
  s=RtpsPlReadUserKey(&in,"x");
  RosTestStr("last key",s,"1");
  Free(s);
  RtpsInInit(&in,b->data,b->len,FALSE);
  RosTestTrue("missing key",RtpsPlReadUserKey(&in,"hash")==NULL);
  RosBufDel(b);
}

U0 TestHash()
{
  TestSha256();
  TestTypeHash();
  TestUserData();
}
