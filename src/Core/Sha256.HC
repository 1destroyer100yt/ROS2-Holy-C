//Sha256: SHA-256 (FIPS 180-4), for the RIHS01 type hashes of Jazzy.
//32-bit words live in I64s and are masked after every step.
//
//  U8 out[32];
//  RosSha256(data,len,out);
//  U8 *hex=RosSha256Hex(data,len);   //64 hex digits, yours to Free

#define ROS_U32_MASK 0xFFFFFFFF

I64 ros_sha256_k[64]={
  0x428a2f98,0x71374491,0xb5c0fbcf,0xe9b5dba5,0x3956c25b,0x59f111f1,0x923f82a4,0xab1c5ed5,
  0xd807aa98,0x12835b01,0x243185be,0x550c7dc3,0x72be5d74,0x80deb1fe,0x9bdc06a7,0xc19bf174,
  0xe49b69c1,0xefbe4786,0x0fc19dc6,0x240ca1cc,0x2de92c6f,0x4a7484aa,0x5cb0a9dc,0x76f988da,
  0x983e5152,0xa831c66d,0xb00327c8,0xbf597fc7,0xc6e00bf3,0xd5a79147,0x06ca6351,0x14292967,
  0x27b70a85,0x2e1b2138,0x4d2c6dfc,0x53380d13,0x650a7354,0x766a0abb,0x81c2c92e,0x92722c85,
  0xa2bfe8a1,0xa81a664b,0xc24b8b70,0xc76c51a3,0xd192e819,0xd6990624,0xf40e3585,0x106aa070,
  0x19a4c116,0x1e376c08,0x2748774c,0x34b0bcb5,0x391c0cb3,0x4ed8aa4a,0x5b9cca4f,0x682e6ff3,
  0x748f82ee,0x78a5636f,0x84c87814,0x8cc70208,0x90befffa,0xa4506ceb,0xbef9a3f7,0xc67178f2};

I64 RosRotr32(I64 x,I64 n)
{
  return ((x>>n)|(x<<(32-n)))&ROS_U32_MASK;
}

//One 64-byte block into the state h[8].
U0 RosSha256Block(I64 *h,U8 *p)
{
  I64 w[64],a,b,c,d,e,f,g,hh,i,s0,s1,t1,t2,ch,maj;
  for (i=0;i<16;i++) {
    a=p[4*i];		//through an I64 (C would shift a U8 as a 32-bit int)
    w[i]=((a<<24)|(p[4*i+1]<<16)|(p[4*i+2]<<8)|p[4*i+3])&ROS_U32_MASK;
  }
  for (i=16;i<64;i++) {
    s0=RosRotr32(w[i-15],7)^RosRotr32(w[i-15],18)^(w[i-15]>>3);
    s1=RosRotr32(w[i-2],17)^RosRotr32(w[i-2],19)^(w[i-2]>>10);
    w[i]=(w[i-16]+s0+w[i-7]+s1)&ROS_U32_MASK;
  }
  a=h[0];
  b=h[1];
  c=h[2];
  d=h[3];
  e=h[4];
  f=h[5];
  g=h[6];
  hh=h[7];
  for (i=0;i<64;i++) {
    s1=RosRotr32(e,6)^RosRotr32(e,11)^RosRotr32(e,25);
    ch=(e&f)^((~e)&g);
    ch=ch&ROS_U32_MASK;
    t1=(hh+s1+ch+ros_sha256_k[i]+w[i])&ROS_U32_MASK;
    s0=RosRotr32(a,2)^RosRotr32(a,13)^RosRotr32(a,22);
    maj=(a&b)^(a&c)^(b&c);
    t2=(s0+maj)&ROS_U32_MASK;
    hh=g;
    g=f;
    f=e;
    e=(d+t1)&ROS_U32_MASK;
    d=c;
    c=b;
    b=a;
    a=(t1+t2)&ROS_U32_MASK;
  }
  h[0]=(h[0]+a)&ROS_U32_MASK;
  h[1]=(h[1]+b)&ROS_U32_MASK;
  h[2]=(h[2]+c)&ROS_U32_MASK;
  h[3]=(h[3]+d)&ROS_U32_MASK;
  h[4]=(h[4]+e)&ROS_U32_MASK;
  h[5]=(h[5]+f)&ROS_U32_MASK;
  h[6]=(h[6]+g)&ROS_U32_MASK;
  h[7]=(h[7]+hh)&ROS_U32_MASK;
}

//The 32-byte digest of data[0,len) into out.
U0 RosSha256(U8 *data,I64 len,U8 *out)
{
  I64 h[8],i,full=len/64,rest,bits=len*8;
  U8 tail[128];
  h[0]=0x6a09e667;
  h[1]=0xbb67ae85;
  h[2]=0x3c6ef372;
  h[3]=0xa54ff53a;
  h[4]=0x510e527f;
  h[5]=0x9b05688c;
  h[6]=0x1f83d9ab;
  h[7]=0x5be0cd19;
  for (i=0;i<full;i++)
    RosSha256Block(h,data+i*64);
  rest=len-full*64;
  MemSet(tail,0,128);
  MemCpy(tail,data+full*64,rest);
  tail[rest]=0x80;
  if (rest<56)
    full=64;
  else
    full=128;
  for (i=0;i<8;i++)
    tail[full-1-i]=(bits>>(8*i))&255;
  RosSha256Block(h,tail);
  if (full==128)
    RosSha256Block(h,tail+64);
  for (i=0;i<32;i++)
    out[i]=(h[i/4]>>(24-8*(i%4)))&255;
}

U8 *RosSha256Hex(U8 *data,I64 len)
{
  U8 d[32],*s=MAlloc(65),*hex="0123456789abcdef";
  I64 i;
  RosSha256(data,len,d);
  for (i=0;i<32;i++) {
    s[2*i]=hex[d[i]>>4];
    s[2*i+1]=hex[d[i]&15];
  }
  s[64]=0;
  return s;
}
