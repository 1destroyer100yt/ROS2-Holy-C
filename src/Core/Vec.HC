//Vec: growable array of fixed-size elements (our std::vector).
//Elements are copied in and out by value.
//
//  CRosVec *v=RosVecNew(sizeof(I64));
//  I64 x=5;
//  RosVecPush(v,&x);
//  I64 *p=RosVecAt(v,0);

class CRosVec
{
  U8 *data;
  I64 len,cap,elem_size;
};

CRosVec *RosVecNew(I64 elem_size)
{
  CRosVec *v=CAlloc(sizeof(CRosVec));
  v->elem_size=elem_size;
  return v;
}

U0 RosVecDel(CRosVec *v)
{
  if (!v)
    return;
  Free(v->data);
  Free(v);
}

//Make room for at least cap elements.
U0 RosVecReserve(CRosVec *v,I64 cap)
{
  U8 *d;
  if (cap<=v->cap)
    return;
  d=MAlloc(cap*v->elem_size);
  if (v->len)
    MemCpy(d,v->data,v->len*v->elem_size);
  Free(v->data);
  v->data=d;
  v->cap=cap;
}

//Pointer to element i, or NULL if out of range.
U8 *RosVecAt(CRosVec *v,I64 i)
{
  if (i<0 || i>=v->len)
    return NULL;
  return v->data+i*v->elem_size;
}

//Copy *elem onto the end. Returns pointer to the stored copy.
U8 *RosVecPush(CRosVec *v,U8 *elem)
{
  U8 *dst;
  if (v->len==v->cap) {
    if (v->cap<4)
      RosVecReserve(v,4);
    else
      RosVecReserve(v,v->cap*2);
  }
  dst=v->data+v->len*v->elem_size;
  if (elem)
    MemCpy(dst,elem,v->elem_size);
  else
    MemSet(dst,0,v->elem_size);
  v->len++;
  return dst;
}

//Push an I64 or pointer (vectors made with elem_size 8).
U0 RosVecPushI64(CRosVec *v,I64 x)
{
  I64 tmp[1];
  tmp[0]=x;
  RosVecPush(v,tmp);
}

//Read element i of an elem_size 8 vector.
I64 RosVecGetI64(CRosVec *v,I64 i)
{
  I64 *p=RosVecAt(v,i);
  if (!p)
    return 0;
  return *p;
}

//Remove the last element, copying it to *out if out isn't NULL.
Bool RosVecPop(CRosVec *v,U8 *out)
{
  if (!v->len)
    return FALSE;
  v->len--;
  if (out)
    MemCpy(out,v->data+v->len*v->elem_size,v->elem_size);
  return TRUE;
}

//Insert before index i (0..len).
Bool RosVecInsert(CRosVec *v,I64 i,U8 *elem)
{
  U8 *src;
  if (i<0 || i>v->len)
    return FALSE;
  RosVecPush(v,NULL);
  src=v->data+i*v->elem_size;
  RosMemMove(src+v->elem_size,src,(v->len-1-i)*v->elem_size);
  if (elem)
    MemCpy(src,elem,v->elem_size);
  else
    MemSet(src,0,v->elem_size);
  return TRUE;
}

//Remove element i, keeping order.
Bool RosVecRemove(CRosVec *v,I64 i)
{
  U8 *dst;
  if (i<0 || i>=v->len)
    return FALSE;
  dst=v->data+i*v->elem_size;
  MemCpy(dst,dst+v->elem_size,(v->len-1-i)*v->elem_size);
  v->len--;
  return TRUE;
}

U0 RosVecClear(CRosVec *v)
{
  v->len=0;
}
