//Map: string -> pointer hash map (open addressing, linear probing).
//Keys are copied. Values are whatever pointer/I64 you store.
//
//  CRosMap *m=RosMapNew;
//  RosMapSet(m,"/chatter",topic);
//  topic=RosMapGet(m,"/chatter");
//
//  I64 it=0; U8 *k,*v;
//  while (RosMapNext(m,&it,&k,&v)) Print("%s\n",k);

#define ROS_MAP_EMPTY	0
#define ROS_MAP_USED	1
#define ROS_MAP_DELETED	2

class CRosMapSlot
{
  U8 *key;
  U8 *val;
  U64 hash;
  I64 state;
};

class CRosMap
{
  CRosMapSlot *slots;
  I64 cap,count,deleted;
};

//FNV-1a, 64 bit.
U64 RosHashStr(U8 *s)
{
  U64 h=0xCBF29CE484222325;
  while (*s) {
    h=h^*s;
    h=h*0x100000001B3;
    s++;
  }
  return h;
}

CRosMap *RosMapNewCap(I64 cap)
{
  CRosMap *m=CAlloc(sizeof(CRosMap));
  I64 c=8;
  while (c<cap)
    c*=2;
  m->cap=c;
  m->slots=CAlloc(c*sizeof(CRosMapSlot));
  return m;
}

CRosMap *RosMapNew()
{
  return RosMapNewCap(8);
}

U0 RosMapDel(CRosMap *m)
{
  I64 i;
  if (!m)
    return;
  for (i=0;i<m->cap;i++)
    if (m->slots[i].state==ROS_MAP_USED)
      Free(m->slots[i].key);
  Free(m->slots);
  Free(m);
}

//Slot holding key, or NULL.
CRosMapSlot *RosMapFind(CRosMap *m,U8 *key)
{
  U64 h=RosHashStr(key);
  I64 i=h&(m->cap-1),n=0;
  CRosMapSlot *s;
  while (n<m->cap) {
    s=&m->slots[i];
    if (s->state==ROS_MAP_EMPTY)
      return NULL;
    if (s->state==ROS_MAP_USED && s->hash==h && !StrCmp(s->key,key))
      return s;
    i=(i+1)&(m->cap-1);
    n++;
  }
  return NULL;
}

extern U0 RosMapSetHashed(CRosMap *m,U8 *key,U64 h,U8 *val,Bool copy_key);

//Rebuild with a new capacity (also clears out tombstones).
U0 RosMapResize(CRosMap *m,I64 cap)
{
  CRosMapSlot *old=m->slots;
  I64 old_cap=m->cap,i;
  m->cap=cap;
  m->slots=CAlloc(cap*sizeof(CRosMapSlot));
  m->count=0;
  m->deleted=0;
  for (i=0;i<old_cap;i++)
    if (old[i].state==ROS_MAP_USED)
      RosMapSetHashed(m,old[i].key,old[i].hash,old[i].val,FALSE);
  Free(old);
}

U0 RosMapSetHashed(CRosMap *m,U8 *key,U64 h,U8 *val,Bool copy_key)
{
  I64 i=h&(m->cap-1);
  CRosMapSlot *s,*grave=NULL;
  while (TRUE) {
    s=&m->slots[i];
    if (s->state==ROS_MAP_EMPTY)
      break;
    if (s->state==ROS_MAP_DELETED) {
      if (!grave)
        grave=s;
    } else if (s->hash==h && !StrCmp(s->key,key)) {
      s->val=val;
      if (!copy_key)
        Free(key);
      return;
    }
    i=(i+1)&(m->cap-1);
  }
  if (grave) {
    s=grave;
    m->deleted--;
  }
  if (copy_key)
    s->key=StrNew(key);
  else
    s->key=key;
  s->val=val;
  s->hash=h;
  s->state=ROS_MAP_USED;
  m->count++;
}

//Add or replace.
U0 RosMapSet(CRosMap *m,U8 *key,U8 *val)
{
  if ((m->count+m->deleted+1)*4>=m->cap*3)
    RosMapResize(m,m->cap*2);
  RosMapSetHashed(m,key,RosHashStr(key),val,TRUE);
}

//Value for key, or NULL. Use RosMapHas if NULL is a valid value.
U8 *RosMapGet(CRosMap *m,U8 *key)
{
  CRosMapSlot *s=RosMapFind(m,key);
  if (!s)
    return NULL;
  return s->val;
}

Bool RosMapHas(CRosMap *m,U8 *key)
{
  return RosMapFind(m,key)!=NULL;
}

//Remove key. Returns FALSE if it wasn't there.
Bool RosMapRemove(CRosMap *m,U8 *key)
{
  CRosMapSlot *s=RosMapFind(m,key);
  if (!s)
    return FALSE;
  Free(s->key);
  s->key=NULL;
  s->val=NULL;
  s->state=ROS_MAP_DELETED;
  m->count--;
  m->deleted++;
  return TRUE;
}

//Walk all entries. Start with *it=0. Order is not defined.
Bool RosMapNext(CRosMap *m,I64 *it,U8 **key,U8 **val)
{
  while (*it<m->cap) {
    if (m->slots[*it].state==ROS_MAP_USED) {
      if (key)
        *key=m->slots[*it].key;
      if (val)
        *val=m->slots[*it].val;
      *it=*it+1;
      return TRUE;
    }
    *it=*it+1;
  }
  return FALSE;
}
