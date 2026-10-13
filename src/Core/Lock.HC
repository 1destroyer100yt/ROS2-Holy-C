//Lock: tiny spinlocks for data shared between tasks and cores.
//A lock is just an I64 that starts at 0. Not re-entrant: never take
//a lock you already hold.
//
//  I64 my_lock=0;
//  RosLockGet(&my_lock); ...; RosLockPut(&my_lock);

U0 RosLockGet(I64 *l)
{
  while (LBts(l,0))	//atomic test-and-set of bit 0
    Yield();
}

U0 RosLockPut(I64 *l)
{
  LBtr(l,0);
}

//Reference counts: one global lock is plenty (very short sections).
I64 ros_ref_lock=0;

//Add v to *ref under the lock, return the new value.
I64 RosRefAdd(I64 *ref,I64 v)
{
  I64 r;
  RosLockGet(&ros_ref_lock);
  *ref=*ref+v;
  r=*ref;
  RosLockPut(&ros_ref_lock);
  return r;
}

//A flag one task sets and others poll (x86 loads and stores of an
//aligned I64 are atomic; the host build needs it spelled out).
I64 RosAtomicGet(I64 *p)
{
//@host: return __atomic_load_n(p,__ATOMIC_ACQUIRE);
//@host-skip-begin
  return *p;
//@host-skip-end
}

U0 RosAtomicSet(I64 *p,I64 v)
{
//@host: __atomic_store_n(p,v,__ATOMIC_RELEASE); return;
//@host-skip-begin
  *p=v;
//@host-skip-end
}
