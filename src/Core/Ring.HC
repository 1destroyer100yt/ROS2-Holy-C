//Ring: fixed-size queue of fixed-size elements.
//
//Safe with exactly one producer task and one consumer task, even on
//different cores: the producer only writes head, the consumer only
//writes tail, and x86 keeps stores in order. More than one producer or
//consumer needs a lock around it.
//
//Capacity is rounded up to a power of two.

class CRosRing
{
  U8 *data;
  I64 elem_size,cap,mask;
  I64 head;	//next slot to write, only the producer changes it
  I64 tail;	//next slot to read, only the consumer changes it
  I64 dropped;	//pushes refused because the ring was full
};

CRosRing *RosRingNew(I64 elem_size,I64 cap)
{
  CRosRing *r=CAlloc(sizeof(CRosRing));
  I64 c=1;
  while (c<cap)
    c*=2;
  r->elem_size=elem_size;
  r->cap=c;
  r->mask=c-1;
  r->data=CAlloc(c*elem_size);
  return r;
}

U0 RosRingDel(CRosRing *r)
{
  if (!r)
    return;
  Free(r->data);
  Free(r);
}

I64 RosRingCount(CRosRing *r)
{
  return r->head-r->tail;
}

Bool RosRingEmpty(CRosRing *r)
{
  return r->head==r->tail;
}

Bool RosRingFull(CRosRing *r)
{
  return r->head-r->tail==r->cap;
}

//Copy *elem in. FALSE (and dropped++) if full.
Bool RosRingPush(CRosRing *r,U8 *elem)
{
  I64 slot;
  if (r->head-r->tail==r->cap) {
    r->dropped++;
    return FALSE;
  }
  slot=r->head&r->mask;
  MemCpy(r->data+slot*r->elem_size,elem,r->elem_size);
  r->head++;	//publish only after the copy is done
  return TRUE;
}

//Copy the oldest element to *out and remove it. FALSE if empty.
Bool RosRingPop(CRosRing *r,U8 *out)
{
  I64 slot;
  if (r->head==r->tail)
    return FALSE;
  slot=r->tail&r->mask;
  if (out)
    MemCpy(out,r->data+slot*r->elem_size,r->elem_size);
  r->tail++;
  return TRUE;
}

//Pointer to the oldest element without removing it. NULL if empty.
U8 *RosRingPeek(CRosRing *r)
{
  if (r->head==r->tail)
    return NULL;
  return r->data+(r->tail&r->mask)*r->elem_size;
}

//"Keep last N" push used by QoS history: if full, drop the oldest.
//Only safe when the same task does all pushes and pops.
U0 RosRingPushOverwrite(CRosRing *r,U8 *elem)
{
  if (r->head-r->tail==r->cap) {
    r->tail++;
    r->dropped++;
  }
  RosRingPush(r,elem);
}
