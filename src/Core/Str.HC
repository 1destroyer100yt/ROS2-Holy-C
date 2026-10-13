//Str: string helpers and a string builder. Strings are plain
//NUL-terminated U8 *. Functions that return a new string say so;
//Free() it when done.

class CRosStrBuf
{
  U8 *buf;
  I64 len,cap;
};

CRosStrBuf *RosStrBufNew()
{
  CRosStrBuf *sb=CAlloc(sizeof(CRosStrBuf));
  sb->cap=32;
  sb->buf=CAlloc(sb->cap);
  return sb;
}

U0 RosStrBufDel(CRosStrBuf *sb)
{
  if (!sb)
    return;
  Free(sb->buf);
  Free(sb);
}

U0 RosStrBufGrow(CRosStrBuf *sb,I64 need)
{
  U8 *b;
  I64 cap=sb->cap;
  if (sb->len+need+1<=cap)
    return;
  while (sb->len+need+1>cap)
    cap*=2;
  b=MAlloc(cap);
  MemCpy(b,sb->buf,sb->len+1);
  Free(sb->buf);
  sb->buf=b;
  sb->cap=cap;
}

U0 RosStrBufAddN(CRosStrBuf *sb,U8 *s,I64 n)
{
  RosStrBufGrow(sb,n);
  MemCpy(sb->buf+sb->len,s,n);
  sb->len+=n;
  sb->buf[sb->len]=0;
}

U0 RosStrBufAdd(CRosStrBuf *sb,U8 *s)
{
  RosStrBufAddN(sb,s,StrLen(s));
}

U0 RosStrBufAddC(CRosStrBuf *sb,U8 c)
{
  U8 tmp[2];
  tmp[0]=c;
  RosStrBufAddN(sb,tmp,1);
}

U0 RosStrBufAddI64(CRosStrBuf *sb,I64 v)
{
  U8 tmp[24];
  I64 i=23;
  Bool neg=v<0;
  U64 u=v;
  if (neg)
    u=-v;	//also right for the most negative I64, as U64
  tmp[i]=0;
  if (!u) {
    i--;
    tmp[i]='0';
  }
  while (u) {
    i--;
    tmp[i]='0'+u%10;
    u/=10;
  }
  if (neg) {
    i--;
    tmp[i]='-';
  }
  RosStrBufAdd(sb,tmp+i);
}

U0 RosStrBufClear(CRosStrBuf *sb)
{
  sb->len=0;
  sb->buf[0]=0;
}

//Return the built string (yours to Free) and free the builder.
U8 *RosStrBufTake(CRosStrBuf *sb)
{
  U8 *s=sb->buf;
  Free(sb);
  return s;
}

//NULL-safe equality.
Bool RosStrEq(U8 *a,U8 *b)
{
  if (!a || !b)
    return a==b;
  return !StrCmp(a,b);
}

Bool RosStrStartsWith(U8 *s,U8 *prefix)
{
  return !StrNCmp(s,prefix,StrLen(prefix));
}

Bool RosStrEndsWith(U8 *s,U8 *suffix)
{
  I64 n=StrLen(s),k=StrLen(suffix);
  if (k>n)
    return FALSE;
  return !StrCmp(s+n-k,suffix);
}

//Index of the first c in s, or -1.
I64 RosStrFindC(U8 *s,U8 c)
{
  I64 i=0;
  while (s[i]) {
    if (s[i]==c)
      return i;
    i++;
  }
  return -1;
}

//Index of the first needle in s, or -1.
I64 RosStrFind(U8 *s,U8 *needle)
{
  I64 i=0,n=StrLen(needle);
  if (!n)
    return 0;
  while (s[i]) {
    if (!StrNCmp(s+i,needle,n))
      return i;
    i++;
  }
  return -1;
}

//New string: n chars of s starting at start (clipped to s).
U8 *RosStrSub(U8 *s,I64 start,I64 n)
{
  I64 len=StrLen(s);
  U8 *r;
  if (start<0)
    start=0;
  if (start>len)
    start=len;
  if (n<0 || start+n>len)
    n=len-start;
  r=MAlloc(n+1);
  MemCpy(r,s+start,n);
  r[n]=0;
  return r;
}

Bool RosIsSpace(U8 c)
{
  return c==' ' || c=='\t' || c=='\n' || c=='\r';
}

//New string without leading/trailing spaces, tabs and newlines.
U8 *RosStrTrim(U8 *s)
{
  I64 a=0,b=StrLen(s);
  while (a<b && RosIsSpace(s[a]))
    a++;
  while (b>a && RosIsSpace(s[b-1]))
    b--;
  return RosStrSub(s,a,b-a);
}

//Split on sep. Returns a vec of new strings (elem_size 8).
//Free with RosStrVecDel.
CRosVec *RosStrSplit(U8 *s,U8 sep)
{
  CRosVec *v=RosVecNew(8);
  I64 start=0,i=0;
  while (TRUE) {
    if (s[i]==sep || !s[i]) {
      RosVecPushI64(v,RosStrSub(s,start,i-start));
      if (!s[i])
        break;
      start=i+1;
    }
    i++;
  }
  return v;
}

U0 RosStrVecDel(CRosVec *v)
{
  I64 i;
  if (!v)
    return;
  for (i=0;i<v->len;i++)
    Free(RosVecGetI64(v,i));
  RosVecDel(v);
}

//Parse a decimal (or 0x hex) integer. The whole string must be
//used, spaces around it are allowed. FALSE on bad input or overflow.
Bool RosStrToI64(U8 *s,I64 *out)
{
  U64 v=0,lim,d;
  Bool neg=FALSE,any=FALSE;
  I64 base=10;
  while (RosIsSpace(*s))
    s++;
  if (*s=='-' || *s=='+') {
    neg=*s=='-';
    s++;
  }
  if (s[0]=='0' && (s[1]=='x' || s[1]=='X')) {
    base=16;
    s+=2;
  }
  lim=0x7FFFFFFFFFFFFFFF;
  if (neg)
    lim++;
  while (*s) {
    if (*s>='0' && *s<='9')
      d=*s-'0';
    else if (base==16 && *s>='a' && *s<='f')
      d=*s-'a'+10;
    else if (base==16 && *s>='A' && *s<='F')
      d=*s-'A'+10;
    else
      break;
    if (v>(lim-d)/base)
      return FALSE;
    v=v*base+d;
    any=TRUE;
    s++;
  }
  while (RosIsSpace(*s))
    s++;
  if (!any || *s)
    return FALSE;
  if (neg)
    *out=-v;
  else
    *out=v;
  return TRUE;
}

//Parse a decimal floating point number: [-+]digits[.digits][e[-+]digits],
//also inf/nan. FALSE on bad input.
Bool RosStrToF64(U8 *s,F64 *out)
{
  F64 v=0.0,scale=1.0,p10=10.0;
  Bool neg=FALSE,any=FALSE,eneg=FALSE;
  I64 e=0,frac=0;
  while (RosIsSpace(*s))
    s++;
  if (*s=='-' || *s=='+') {
    neg=*s=='-';
    s++;
  }
  if (!StrNCmp(s,"inf",3) || !StrNCmp(s,"nan",3)) {
    if (s[0]=='i')
      v=RosBitsToF64(ROS_F64_INF_BITS);
    else
      v=RosBitsToF64(ROS_F64_NAN_BITS);
    s+=3;
    any=TRUE;
  } else {
    while (*s>='0' && *s<='9') {
      v=v*10.0+(*s-'0');
      s++;
      any=TRUE;
    }
    if (*s=='.') {
      s++;
      while (*s>='0' && *s<='9') {
        v=v*10.0+(*s-'0');
        frac++;
        s++;
        any=TRUE;
      }
    }
    if (any && (*s=='e' || *s=='E')) {
      s++;
      if (*s=='-' || *s=='+') {
        eneg=*s=='-';
        s++;
      }
      if (!(*s>='0' && *s<='9'))
        return FALSE;
      while (*s>='0' && *s<='9') {
        if (e<10000)
          e=e*10+(*s-'0');
        s++;
      }
    }
    if (eneg)
      e=-e;
    e-=frac;
    if (e<0) {
      e=-e;
      while (e) {
        if (e&1)
          scale*=p10;
        p10*=p10;
        e>>=1;
      }
      v=v/scale;
    } else {
      while (e) {
        if (e&1)
          scale*=p10;
        p10*=p10;
        e>>=1;
      }
      v=v*scale;
    }
  }
  while (RosIsSpace(*s))
    s++;
  if (!any || *s)
    return FALSE;
  if (neg)
    v=-v;
  *out=v;
  return TRUE;
}

U0 RosStrBufAddU64(CRosStrBuf *sb,U64 u)
{
  U8 tmp[24];
  I64 i=23;
  tmp[i]=0;
  if (!u) {
    i--;
    tmp[i]='0';
  }
  while (u) {
    i--;
    tmp[i]='0'+u%10;
    u/=10;
  }
  RosStrBufAdd(sb,tmp+i);
}

//Add an F64 formatted like Python's repr() (which is what ros2 topic
//echo shows): the shortest digits that read back as the same value
//(exact, see Dtoa.HC): "1.0", "0.1", "1e-05", "1.5e+20", "nan", "inf".
U0 RosStrBufAddF64(CRosStrBuf *sb,F64 x)
{
  U64 bits=RosF64ToBits(x);
  I64 p,n,i;
  U8 dig[24];
  if (((bits>>52)&0x7FF)==0x7FF) {
    if (bits&0xFFFFFFFFFFFFF)
      RosStrBufAdd(sb,"nan");
    else if (bits>>63)
      RosStrBufAdd(sb,"-inf");
    else
      RosStrBufAdd(sb,"inf");
    return;
  }
  if (bits>>63) {
    RosStrBufAddC(sb,'-');
    x=-x;
  }
  if (x==0.0) {
    RosStrBufAdd(sb,"0.0");
    return;
  }
  p=RosF64Digits(x,dig,&n);
  n--;	//exponent of the first digit
  if (n>=-4 && n<16) {
    if (n>=0) {
      for (i=0;i<=n;i++) {
        if (i<p)
          RosStrBufAddC(sb,dig[i]);
        else
          RosStrBufAddC(sb,'0');
      }
      RosStrBufAddC(sb,'.');
      if (p>n+1)
        RosStrBufAdd(sb,dig+n+1);
      else
        RosStrBufAddC(sb,'0');
    } else {
      RosStrBufAdd(sb,"0.");
      for (i=0;i< -n-1;i++)
        RosStrBufAddC(sb,'0');
      RosStrBufAdd(sb,dig);
    }
    return;
  }
  RosStrBufAddC(sb,dig[0]);
  if (p>1) {
    RosStrBufAddC(sb,'.');
    RosStrBufAdd(sb,dig+1);
  }
  RosStrBufAddC(sb,'e');
  if (n<0) {
    RosStrBufAddC(sb,'-');
    n=-n;
  } else
    RosStrBufAddC(sb,'+');
  if (n<10)
    RosStrBufAddC(sb,'0');
  RosStrBufAddI64(sb,n);
}

//Add s as a YAML single-quoted string: 'it''s'.
U0 RosStrBufAddQuoted(CRosStrBuf *sb,U8 *s)
{
  RosStrBufAddC(sb,'\'');
  if (s)
    while (*s) {
      if (*s=='\'')
        RosStrBufAddC(sb,'\'');
      RosStrBufAddC(sb,*s);
      s++;
    }
  RosStrBufAddC(sb,'\'');
}

//If s is quoted with ' or ", return a new unquoted copy (handles
//'' inside single quotes and \" \\ \' \n \t inside double quotes).
//Otherwise a plain copy. Yours to Free.
U8 *RosStrUnquote(U8 *s)
{
  CRosStrBuf *sb;
  I64 n=StrLen(s),i;
  U8 q;
  if (n<2 || (s[0]!='\'' && s[0]!='"') || s[n-1]!=s[0])
    return StrNew(s);
  q=s[0];
  sb=RosStrBufNew();
  for (i=1;i<n-1;i++) {
    if (q=='\'' && s[i]=='\'' && s[i+1]=='\'' && i+1<n-1) {
      RosStrBufAddC(sb,'\'');
      i++;
    } else if (q=='"' && s[i]=='\\' && i+1<n-1) {
      i++;
      if (s[i]=='n')
        RosStrBufAddC(sb,'\n');
      else if (s[i]=='t')
        RosStrBufAddC(sb,'\t');
      else
        RosStrBufAddC(sb,s[i]);
    } else
      RosStrBufAddC(sb,s[i]);
  }
  return RosStrBufTake(sb);
}

//Sort a vec of U8 * strings in place (insertion sort: lists are short).
U0 RosStrVecSort(CRosVec *v)
{
  I64 i,j;
  U8 **a=v->data,*x;
  for (i=1;i<v->len;i++) {
    x=a[i];
    j=i-1;
    while (j>=0 && StrCmp(a[j],x)>0) {
      a[j+1]=a[j];
      j--;
    }
    a[j+1]=x;
  }
}

//Add a fixed-point number with d decimals.
U0 RosStrBufAddFixed(CRosStrBuf *sb,F64 x,I64 d)
{
  I64 scale=1,i,v,ip,fp;
  for (i=0;i<d;i++)
    scale*=10;
  v=ToI64(Abs(x)*scale+0.5);
  if (x<0.0 && v)
    RosStrBufAddC(sb,'-');
  ip=v/scale;
  fp=v%scale;
  RosStrBufAddI64(sb,ip);
  if (d) {
    RosStrBufAddC(sb,'.');
    i=scale/10;
    while (i>fp && i>1) {
      RosStrBufAddC(sb,'0');
      i/=10;
    }
    if (fp)
      RosStrBufAddI64(sb,fp);
    else if (scale>1)
      RosStrBufAddC(sb,'0');
  }
}

//Same text? NULL counts as "" (empty message strings are NULL).
Bool RosStrSame(U8 *a,U8 *b)
{
  if (!a)
    a="";
  if (!b)
    b="";
  return !StrCmp(a,b);
}
