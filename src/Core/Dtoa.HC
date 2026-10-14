//Dtoa: exact shortest decimal digits for an F64 ("free-format"
//printing, Burger & Dybvig 1996). The digits always read back as the
//same F64, and are the shortest such digits (closest to the true
//value when there's a choice), like Python's repr().
//
//Uses small fixed-size big integers: 32-bit limbs kept in U64s.

#define ROS_BIG_LIMBS	48	//1536 bits, enough for any F64

class CRosBig
{
  I64 n;			//limbs in use
  U64 d[ROS_BIG_LIMBS];	//little-endian 32-bit limbs
};

U0 RosBigSet(CRosBig *a,U64 v)
{
  a->n=0;
  while (v) {
    a->d[a->n]=v&0xFFFFFFFF;
    a->n++;
    v=v>>32;
  }
}

U0 RosBigCopy(CRosBig *dst,CRosBig *src)
{
  I64 i;
  dst->n=src->n;
  for (i=0;i<src->n;i++)
    dst->d[i]=src->d[i];
}

U0 RosBigMulSmall(CRosBig *a,U64 k)
{
  U64 carry=0,t;
  I64 i;
  for (i=0;i<a->n;i++) {
    t=a->d[i]*k+carry;
    a->d[i]=t&0xFFFFFFFF;
    carry=t>>32;
  }
  while (carry) {
    a->d[a->n]=carry&0xFFFFFFFF;
    a->n++;
    carry=carry>>32;
  }
}

U0 RosBigMulPow10(CRosBig *a,I64 n)
{
  while (n>=9) {
    RosBigMulSmall(a,1000000000);
    n-=9;
  }
  while (n>0) {
    RosBigMulSmall(a,10);
    n--;
  }
}

U0 RosBigShl(CRosBig *a,I64 bits)
{
  I64 i,w=bits/32,b=bits%32;
  U64 hi;
  if (!a->n)
    return;
  if (b) {
    a->d[a->n]=0;
    a->n++;
    i=a->n-1;
    while (i>0) {
      hi=(a->d[i]<<b)|(a->d[i-1]>>(32-b));
      a->d[i]=hi&0xFFFFFFFF;
      i--;
    }
    a->d[0]=(a->d[0]<<b)&0xFFFFFFFF;
  }
  if (w) {
    i=a->n-1;
    while (i>=0) {
      a->d[i+w]=a->d[i];
      i--;
    }
    for (i=0;i<w;i++)
      a->d[i]=0;
    a->n+=w;
  }
  while (a->n && !a->d[a->n-1])
    a->n--;
}

I64 RosBigCmp(CRosBig *a,CRosBig *b)
{
  I64 i;
  if (a->n!=b->n) {
    if (a->n>b->n)
      return 1;
    return -1;
  }
  i=a->n-1;
  while (i>=0) {
    if (a->d[i]!=b->d[i]) {
      if (a->d[i]>b->d[i])
        return 1;
      return -1;
    }
    i--;
  }
  return 0;
}

//dst=a+b (dst may be a or b)
U0 RosBigAdd(CRosBig *dst,CRosBig *a,CRosBig *b)
{
  I64 i,n=a->n;
  U64 carry=0,t,x,y;
  if (b->n>n)
    n=b->n;
  for (i=0;i<n;i++) {
    x=0;
    y=0;
    if (i<a->n)
      x=a->d[i];
    if (i<b->n)
      y=b->d[i];
    t=x+y+carry;
    dst->d[i]=t&0xFFFFFFFF;
    carry=t>>32;
  }
  dst->n=n;
  if (carry) {
    dst->d[n]=carry;
    dst->n++;
  }
}

//a-=b, needs a>=b
U0 RosBigSub(CRosBig *a,CRosBig *b)
{
  I64 i;
  U64 borrow=0,x,y;
  for (i=0;i<a->n;i++) {
    x=a->d[i];
    y=borrow;
    if (i<b->n)
      y+=b->d[i];
    if (x>=y) {
      a->d[i]=x-y;
      borrow=0;
    } else {
      a->d[i]=x+0x100000000-y;
      borrow=1;
    }
  }
  while (a->n && !a->d[a->n-1])
    a->n--;
}

//Compare a+b with c.
I64 RosBigCmpSum(CRosBig *a,CRosBig *b,CRosBig *c)
{
  CRosBig t;
  RosBigAdd(&t,a,b);
  return RosBigCmp(&t,c);
}

//Shortest digits of x (x>0, finite) into dig (up to 17 chars + NUL).
//Returns the number of digits; *k is the decimal exponent so that
//x = 0.DIGITS * 10^k.
I64 RosF64Digits(F64 x,U8 *dig,I64 *k)
{
  U64 bits=RosF64ToBits(x),f;
  I64 e,be,est,n=0,dd,tc1,tc2,c;
  Bool even;
  CRosBig r,s,mp,mm,t;
  be=(bits>>52)&0x7FF;
  f=bits&0xFFFFFFFFFFFFF;
  if (be) {
    f=f|0x10000000000000;
    e=be-1075;
  } else
    e=-1074;
  even=!(f&1);
  //r/s is x scaled; mp/mm are the gaps to the neighbours (times 2)
  if (e>=0) {
    RosBigSet(&r,f);
    RosBigShl(&r,e+1);
    RosBigSet(&s,2);
    RosBigSet(&mp,1);
    RosBigShl(&mp,e);
    RosBigSet(&mm,1);
    RosBigShl(&mm,e);
    if (f==0x10000000000000) {	//lower gap is half as big
      RosBigShl(&r,1);
      RosBigShl(&s,1);
      RosBigShl(&mp,1);
    }
  } else {
    RosBigSet(&r,f);
    RosBigShl(&r,1);
    RosBigSet(&s,1);
    RosBigShl(&s,1-e);
    RosBigSet(&mp,1);
    RosBigSet(&mm,1);
    if (f==0x10000000000000 && be>1) {
      RosBigShl(&r,1);
      RosBigShl(&s,1);
      RosBigShl(&mp,1);
    }
  }
  //estimate k = ceil(log10(x)), then fix it exactly
  est=ToI64(Ceil((ToF64(e)+52.0)*0.30102999566398120-0.0000001));
  if (est>=0)
    RosBigMulPow10(&s,est);
  else {
    RosBigMulPow10(&r,-est);
    RosBigMulPow10(&mp,-est);
    RosBigMulPow10(&mm,-est);
  }
  while (TRUE) {	//too small: x + gap reaches s
    c=RosBigCmpSum(&r,&mp,&s);
    if (c>0 || (even && c==0)) {
      RosBigMulSmall(&s,10);
      est++;
    } else
      break;
  }
  while (TRUE) {	//too big: x*10 + gap*10 still below s
    RosBigCopy(&t,&r);
    RosBigAdd(&t,&t,&mp);
    RosBigMulSmall(&t,10);
    c=RosBigCmp(&t,&s);
    if (c<0 || (!even && c==0)) {
      RosBigMulSmall(&r,10);
      RosBigMulSmall(&mp,10);
      RosBigMulSmall(&mm,10);
      est--;
    } else
      break;
  }
  *k=est;
  while (n<17) {
    RosBigMulSmall(&r,10);
    RosBigMulSmall(&mp,10);
    RosBigMulSmall(&mm,10);
    dd=0;
    while (RosBigCmp(&r,&s)>=0) {
      RosBigSub(&r,&s);
      dd++;
    }
    c=RosBigCmp(&r,&mm);
    tc1=c<0 || (even && c==0);
    c=RosBigCmpSum(&r,&mp,&s);
    tc2=c>0 || (even && c==0);
    if (!tc1 && !tc2) {
      dig[n]='0'+dd;
      n++;
      goto next_dtoa_1;
    }
    if (tc1 && tc2) {	//both ends ok: pick the closer one
      RosBigCopy(&t,&r);
      RosBigShl(&t,1);
      c=RosBigCmp(&t,&s);
      if (c>0 || (c==0 && (dd&1)))
        dd++;
    } else if (tc2)
      dd++;
    dig[n]='0'+dd;
    n++;
    break;
    next_dtoa_1:;
  }
  dig[n]=0;
  return n;
}
