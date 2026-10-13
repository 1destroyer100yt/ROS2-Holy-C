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
