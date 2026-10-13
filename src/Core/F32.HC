//F32: HolyC only has F64, but ROS messages are full of float32.
//We keep values as F64 in memory and convert to/from IEEE-754 single
//precision bits only when (de)serializing.
//
//F64ToF32Bits rounds to nearest, ties to even (same as a C cast),
//handles +-0, subnormals, overflow to +-inf, and NaN.

#define ROS_F64_INF_BITS	0x7FF0000000000000
#define ROS_F64_NAN_BITS	0x7FF8000000000000
#define ROS_F32_INF_BITS	0x7F800000
#define ROS_F32_NAN_BITS	0x7FC00000

union CRosF64Bits
{
  F64 f;
  U64 u;
};

//Raw bits of an F64 (for CDR float64).
U64 RosF64ToBits(F64 x)
{
  CRosF64Bits b;
  b.f=x;
  return b.u;
}

F64 RosBitsToF64(U64 u)
{
  CRosF64Bits b;
  b.u=u;
  return b.f;
}

//v >> s (1 <= s <= 63), rounded to nearest, ties to even.
U64 RosShrRound(U64 v,I64 s)
{
  U64 one=1,q,rem,half;
  q=v>>s;
  rem=v&((one<<s)-1);
  half=one<<(s-1);
  if (rem>half || (rem==half && (q&1)))
    q++;
  return q;
}

U32 RosF64ToF32Bits(F64 x)
{
  U64 u=RosF64ToBits(x),sign,mant,r;
  I64 exp,e;
  sign=(u>>63)<<31;
  exp=(u>>52)&0x7FF;
  mant=u&0xFFFFFFFFFFFFF;
  if (exp==0x7FF) {
    if (mant)
      return sign|ROS_F32_NAN_BITS|(mant>>29);
    return sign|ROS_F32_INF_BITS;
  }
  if (!exp)		//F64 subnormal or zero: far below F32 range
    return sign;
  e=exp-1023+127;
  if (e>=0xFF)
    return sign|ROS_F32_INF_BITS;
  mant=mant|0x10000000000000;	//put back the hidden 1 (bit 52)
  if (e<=0) {
    if (e< -24)
      return sign;
    //F32 subnormal: value = q * 2^-149. A carry into bit 23 makes
    //the smallest normal number, which is still the right encoding.
    return sign|RosShrRound(mant,30-e);
  }
  r=RosShrRound(mant,29);	//24 bits incl. hidden 1, may carry to 25
  //(e<<23)+(r-(1<<23)): the carry bumps the exponent, up to inf.
  r=(e<<23)+(r-0x800000);
  if (r>=ROS_F32_INF_BITS)
    return sign|ROS_F32_INF_BITS;
  return sign|r;
}

F64 RosF32BitsToF64(U32 v)
{
  U64 sign,mant,u;
  I64 exp,e;
  sign=(v>>31)&1;
  sign=sign<<63;
  exp=(v>>23)&0xFF;
  mant=v&0x7FFFFF;
  if (exp==0xFF) {
    u=sign|ROS_F64_INF_BITS|(mant<<29);
    return RosBitsToF64(u);
  }
  if (!exp) {
    if (!mant)
      return RosBitsToF64(sign);
    e=-126;
    while (!(mant&0x800000)) {
      mant=mant<<1;
      e--;
    }
    mant=mant&0x7FFFFF;
  } else
    e=exp-127;
  u=sign|((e+1023)<<52)|(mant<<29);
  return RosBitsToF64(u);
}

//Round an F64 to the nearest float32 value (still as F64). Handy
//for comparing what will come out of the other side of the wire.
F64 RosF32Round(F64 x)
{
  return RosF32BitsToF64(RosF64ToF32Bits(x));
}
