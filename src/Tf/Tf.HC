//Tf: the transform tree, like tf2. Every frame has one parent and a
//short history of where it was relative to that parent; lookups walk
//the tree and interpolate in time.
//
//  CRosTfBuffer *tb=RosTfBufferNew();
//  RosTfListenerNew(node,tb);                 //fills it from /tf and /tf_static
//  CRosTf t;
//  if (RosTfLookup(tb,"map","base_link",0,&t)) ...  //0 = latest
//
//RosTfLookup(target,source) gives T_target_source: it maps points in
//the source frame into the target frame (same as tf2's lookupTransform).

#define ROS_TF_CACHE_NS	10000000000	//keep 10 s of history, like tf2
#define ROS_TF_MAX_DEPTH 100

class CRosTfSample
{
  I64 stamp;
  CRosTf tf;
};

class CRosTfFrame
{
  U8 *name,*parent;
  Bool is_static;
  CRosVec *hist;	//CRosTfSample, oldest first
};

class CRosTfBuffer
{
  CRosMap *frames;
  I64 lock,cache_ns;
};

U8 ros_tf_error[160];

U0 RosTfErr(U8 *what,U8 *a,U8 *b)
{
  U8 *s;
  if (b)
    s=MStrPrint("%s: '%s' -> '%s'",what,a,b);
  else
    s=MStrPrint("%s: '%s'",what,a);
  if (StrLen(s)>150)
    s[150]=0;
  StrCpy(ros_tf_error,s);
  Free(s);
}

//tf2 ignores a leading '/' on frame names.
U8 *RosTfName(U8 *s)
{
  if (s && s[0]=='/')
    return s+1;
  return s;
}

CRosTfBuffer *RosTfBufferNew()
{
  CRosTfBuffer *b=CAlloc(sizeof(CRosTfBuffer));
  b->frames=RosMapNew();
  b->cache_ns=ROS_TF_CACHE_NS;
  return b;
}

U0 RosTfBufferDel(CRosTfBuffer *b)
{
  I64 it=0;
  U8 *k;
  CRosTfFrame *f;
  if (!b)
    return;
  while (RosMapNext(b->frames,&it,&k,&f)) {
    Free(f->name);
    Free(f->parent);
    RosVecDel(f->hist);
    Free(f);
  }
  RosMapDel(b->frames);
  Free(b);
}

//Add where child is relative to parent at stamp (ns).
Bool RosTfSet(CRosTfBuffer *b,U8 *parent,U8 *child,I64 stamp,CRosTf *t,Bool is_static)
{
  CRosTfFrame *f;
  CRosTfSample s,*last,*second;
  I64 i;
  parent=RosTfName(parent);
  child=RosTfName(child);
  if (!parent[0] || !child[0] || !StrCmp(parent,child)) {
    RosTfErr("bad frame names",parent,child);
    return FALSE;
  }
  RosLockGet(&b->lock);
  f=RosMapGet(b->frames,child);
  if (!f) {
    f=CAlloc(sizeof(CRosTfFrame));
    f->name=StrNew(child);
    f->hist=RosVecNew(sizeof(CRosTfSample));
    RosMapSet(b->frames,child,f);
  }
  if (!f->parent || StrCmp(f->parent,parent)) {	//new parent: start over
    Free(f->parent);
    f->parent=StrNew(parent);
    RosVecClear(f->hist);
  }
  f->is_static=is_static;
  s.stamp=stamp;
  MemCpy(&s.tf,t,sizeof(CRosTf));
  RosTfNormalize(&s.tf);
  if (is_static)
    RosVecClear(f->hist);
  //keep it sorted by time (almost always an append)
  i=f->hist->len;
  while (i>0) {
    last=RosVecAt(f->hist,i-1);
    if (last->stamp<=stamp)
      break;
    i--;
  }
  if (i>0) {
    last=RosVecAt(f->hist,i-1);
    if (last->stamp==stamp) {	//same time: replace
      MemCpy(last,&s,sizeof(CRosTfSample));
      RosLockPut(&b->lock);
      return TRUE;
    }
  }
  RosVecInsert(f->hist,i,&s);
  if (!is_static) {	//forget what's older than the cache window
    last=RosVecAt(f->hist,f->hist->len-1);
    while (f->hist->len>2) {
      second=RosVecAt(f->hist,1);
      if (second->stamp>last->stamp-b->cache_ns)
        break;
      RosVecRemove(f->hist,0);
      last=RosVecAt(f->hist,f->hist->len-1);
    }
  }
  RosLockPut(&b->lock);
  return TRUE;
}

//Newest stamp of a frame (0 for static frames: valid at any time).
I64 RosTfNewest(CRosTfFrame *f)
{
  CRosTfSample *s;
  if (f->is_static || !f->hist->len)
    return 0;
  s=RosVecAt(f->hist,f->hist->len-1);
  return s->stamp;
}

//f relative to its parent at time t (0 = newest). FALSE if t is
//outside the history (tf2's extrapolation error).
Bool RosTfFrameAt(CRosTfFrame *f,I64 t,CRosTf *out)
{
  CRosTfSample *a,*b;
  I64 i,n=f->hist->len;
  if (!n) {
    RosTfErr("no data for frame",f->name,NULL);
    return FALSE;
  }
  if (f->is_static || !t) {
    a=RosVecAt(f->hist,n-1);
    MemCpy(out,&a->tf,sizeof(CRosTf));
    return TRUE;
  }
  a=RosVecAt(f->hist,0);
  b=RosVecAt(f->hist,n-1);
  if (t<a->stamp || t>b->stamp) {
    if (t>b->stamp)
      RosTfErr("lookup would require extrapolation into the future",f->name,NULL);
    else
      RosTfErr("lookup would require extrapolation into the past",f->name,NULL);
    return FALSE;
  }
  for (i=1;i<n;i++) {
    b=RosVecAt(f->hist,i);
    if (b->stamp>=t)
      break;
  }
  a=RosVecAt(f->hist,i-1);
  if (i>=n || b->stamp==a->stamp) {
    MemCpy(out,&b->tf,sizeof(CRosTf));
    return TRUE;
  }
  RosTfLerp(out,&a->tf,&b->tf,ToF64(t-a->stamp)/ToF64(b->stamp-a->stamp));
  return TRUE;
}

class CRosTfChain	//a frame and T(frame <- start)
{
  U8 *name;
  CRosTf tf;
};

//Walk up from start: every ancestor with T(ancestor <- start).
//Also lowers *latest to the newest common time along the way.
CRosVec *RosTfChainUp(CRosTfBuffer *b,U8 *start,I64 t,I64 *latest,Bool *ok)
{
  CRosVec *v=RosVecNew(sizeof(CRosTfChain));
  CRosTfChain c,*prev;
  CRosTfFrame *f;
  CRosTf step;
  I64 depth=0,nw;
  c.name=start;
  RosTfIdentity(&c.tf);
  RosVecPush(v,&c);
  *ok=TRUE;
  while (depth<ROS_TF_MAX_DEPTH) {
    prev=RosVecAt(v,v->len-1);
    f=RosMapGet(b->frames,prev->name);
    if (!f)
      break;	//reached a root
    if (latest) {
      nw=RosTfNewest(f);
      if (nw && (!*latest || nw<*latest))
        *latest=nw;
    } else if (!RosTfFrameAt(f,t,&step)) {
      *ok=FALSE;
      break;
    }
    if (latest)
      RosTfIdentity(&step);
    c.name=f->parent;
    RosTfCompose(&c.tf,&step,&prev->tf);
    RosVecPush(v,&c);
    depth++;
  }
  return v;
}

I64 RosTfChainFind(CRosVec *v,U8 *name)
{
  I64 i;
  CRosTfChain *c;
  for (i=0;i<v->len;i++) {
    c=RosVecAt(v,i);
    if (!StrCmp(c->name,name))
      return i;
  }
  return -1;
}

//Is name some frame's parent (a root has no entry of its own)?
Bool RosTfChainFindRoot(CRosTfBuffer *b,U8 *name)
{
  I64 it=0;
  U8 *k;
  CRosTfFrame *f;
  while (RosMapNext(b->frames,&it,&k,&f))
    if (f->parent && !StrCmp(f->parent,name))
      return TRUE;
  return FALSE;
}

//T(target <- source) at time t (ns; 0 = the newest time every frame
//on the path has). FALSE + ros_tf_error if it can't.
Bool RosTfLookup(CRosTfBuffer *b,U8 *target,U8 *source,I64 t,CRosTf *out)
{
  CRosVec *vs,*vt;
  CRosTfChain *cs,*ct;
  CRosTf inv;
  I64 i,j,latest=0;
  Bool ok_s,ok_t,ok=FALSE;
  target=RosTfName(target);
  source=RosTfName(source);
  if (!StrCmp(target,source)) {
    RosTfIdentity(out);
    return TRUE;
  }
  RosLockGet(&b->lock);
  if (!RosMapHas(b->frames,source) && !RosTfChainFindRoot(b,source)) {
    RosTfErr("frame does not exist",source,NULL);
    RosLockPut(&b->lock);
    return FALSE;
  }
  if (!RosMapHas(b->frames,target) && !RosTfChainFindRoot(b,target)) {
    RosTfErr("frame does not exist",target,NULL);
    RosLockPut(&b->lock);
    return FALSE;
  }
  if (!t) {	//latest common time: min of the newest stamps on both paths
    vs=RosTfChainUp(b,source,0,&latest,&ok_s);
    RosVecDel(vs);
    vt=RosTfChainUp(b,target,0,&latest,&ok_t);
    RosVecDel(vt);
    t=latest;
  }
  vs=RosTfChainUp(b,source,t,NULL,&ok_s);
  vt=RosTfChainUp(b,target,t,NULL,&ok_t);
  if (ok_s && ok_t) {
    for (i=0;i<vt->len && !ok;i++) {
      ct=RosVecAt(vt,i);
      j=RosTfChainFind(vs,ct->name);
      if (j>=0) {	//common ancestor
        cs=RosVecAt(vs,j);
        RosTfInverse(&inv,&ct->tf);
        RosTfCompose(out,&inv,&cs->tf);
        ok=TRUE;
      }
    }
    if (!ok)
      RosTfErr("frames are not connected",target,source);
  }
  RosVecDel(vs);
  RosVecDel(vt);
  RosLockPut(&b->lock);
  return ok;
}

//The newest time a lookup between the two frames can use (0 if it's
//all static frames).
I64 RosTfLatestTime(CRosTfBuffer *b,U8 *target,U8 *source)
{
  CRosVec *v;
  I64 latest=0;
  Bool ok;
  RosLockGet(&b->lock);
  v=RosTfChainUp(b,RosTfName(source),0,&latest,&ok);
  RosVecDel(v);
  v=RosTfChainUp(b,RosTfName(target),0,&latest,&ok);
  RosVecDel(v);
  RosLockPut(&b->lock);
  return latest;
}

Bool RosTfCanTransform(CRosTfBuffer *b,U8 *target,U8 *source,I64 t)
{
  CRosTf x;
  return RosTfLookup(b,target,source,t,&x);
}

//All frames as "child <- parent" lines (like view_frames, in text).
U8 *RosTfFramesStr(CRosTfBuffer *b)
{
  CRosVec *v=RosVecNew(8);
  CRosStrBuf *sb=RosStrBufNew();
  I64 it=0,i;
  U8 *k;
  CRosTfFrame *f;
  RosLockGet(&b->lock);
  while (RosMapNext(b->frames,&it,&k,&f)) {
    if (f->is_static)
      RosVecPushI64(v,MStrPrint("%s <- %s (static)\n",f->name,f->parent));
    else
      RosVecPushI64(v,MStrPrint("%s <- %s\n",f->name,f->parent));
  }
  RosLockPut(&b->lock);
  RosStrVecSort(v);
  for (i=0;i<v->len;i++)
    RosStrBufAdd(sb,RosVecGetI64(v,i));
  RosStrVecDel(v);
  return RosStrBufTake(sb);
}
