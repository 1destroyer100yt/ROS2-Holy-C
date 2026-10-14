//Mem: fixed-size block pool allocator.
//
//Messages get allocated and freed all the time. MAlloc/Free for each
//one is slow and fragments the heap, so hot paths take blocks from a
//pool instead. Blocks come back zeroed.
//
//Not thread safe: use one pool per task/core (see PLAN.md, Exec).
//
//  CRosPool *p=RosPoolNew(64,32);   //64-byte blocks, 32 per chunk
//  U8 *b=RosPoolAlloc(p);
//  RosPoolFree(p,b);
//  RosPoolDel(p);

//MemCpy that works when the two ranges overlap. TempleOS's MemCpy
//copies forward a byte at a time, so copying to a higher address that
//overlaps the source repeats the first bytes; this copies backward then.
U8 *RosMemMove(U8 *dst,U8 *src,I64 n)
{
  I64 i;
  if (n<=0 || dst==src)
    return dst;
  if (dst<src || dst>=src+n)
    return MemCpy(dst,src,n);
  i=n;
  while (i>0) {
    i--;
    dst[i]=src[i];
  }
  return dst;
}

class CRosPoolChunk
{
  CRosPoolChunk *next;
  U8 *mem;
};

class CRosPoolFree
{
  CRosPoolFree *next;
};

class CRosPool
{
  I64 block_size,per_chunk;
  CRosPoolFree *free_list;
  CRosPoolChunk *chunks;
  I64 used,total;
  Bool debug;		//check every free; costs a chunk walk
};

//New pool. block_size is rounded up to a multiple of 8.
CRosPool *RosPoolNew(I64 block_size,I64 per_chunk)
{
  CRosPool *p=CAlloc(sizeof(CRosPool));
  if (block_size<8)
    block_size=8;
  block_size=(block_size+7)/8*8;
  if (per_chunk<1)
    per_chunk=1;
  p->block_size=block_size;
  p->per_chunk=per_chunk;
  return p;
}

//Add one chunk of blocks to the free list.
U0 RosPoolGrow(CRosPool *p)
{
  CRosPoolChunk *c=CAlloc(sizeof(CRosPoolChunk));
  CRosPoolFree *f;
  I64 i;
  c->mem=MAlloc(p->block_size*p->per_chunk);
  c->next=p->chunks;
  p->chunks=c;
  i=p->per_chunk-1;
  while (i>=0) {
    f=c->mem+i*p->block_size;
    f->next=p->free_list;
    p->free_list=f;
    i--;
  }
  p->total+=p->per_chunk;
}

//Get a zeroed block.
U8 *RosPoolAlloc(CRosPool *p)
{
  CRosPoolFree *f;
  if (!p->free_list)
    RosPoolGrow(p);
  f=p->free_list;
  p->free_list=f->next;
  MemSet(f,0,p->block_size);
  p->used++;
  return f;
}

//True if ptr is the start of a block that belongs to this pool.
Bool RosPoolOwns(CRosPool *p,U8 *ptr)
{
  CRosPoolChunk *c=p->chunks;
  I64 off;
  while (c) {
    off=ptr-c->mem;
    if (off>=0 && off<p->block_size*p->per_chunk)
      return off%p->block_size==0;
    c=c->next;
  }
  return FALSE;
}

//Give a block back. In debug mode, foreign pointers are refused
//instead of corrupting the pool.
Bool RosPoolFree(CRosPool *p,U8 *ptr)
{
  CRosPoolFree *f=ptr;
  if (!ptr)
    return FALSE;
  if (p->debug && !RosPoolOwns(p,ptr)) {
    Print("RosPoolFree: pointer does not belong to this pool\n");
    return FALSE;
  }
  f->next=p->free_list;
  p->free_list=f;
  p->used--;
  return TRUE;
}

//Free the pool and every block in it.
U0 RosPoolDel(CRosPool *p)
{
  CRosPoolChunk *c,*next;
  if (!p)
    return;
  c=p->chunks;
  while (c) {
    next=c->next;
    Free(c->mem);
    Free(c);
    c=next;
  }
  Free(p);
}
