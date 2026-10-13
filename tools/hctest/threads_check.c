/* Concurrency check for src/Graph: publisher threads and an executor
 * thread on the same topic, like TempleOS tasks on different cores.
 * Build (tools/check.sh does this) with -fsanitize=thread:
 *   python3 tools/hctest/hc2c.py c Load.HC -o lib.c
 *   gcc -O1 -g -fsanitize=thread -include lib.c threads_check.c -lpthread -lm */
#include <pthread.h>
#define PUBS 3
#define PER_PUB 20000

static CRosPub *pubs[PUBS];
static I64 last_seen[PUBS], received, out_of_order;
static volatile int pubs_done;

static void on_msg(CMsg_std_msgs_Int64 *m, U8 *user)
{
  I64 who = m->data / 1000000, seq = m->data % 1000000;
  if (seq <= last_seen[who]) out_of_order++;
  last_seen[who] = seq;
  received++;
}

static void *pub_thread(void *arg)
{
  long k = (long)arg;
  for (I64 i = 1; i <= PER_PUB; i++) {
    CMsg_std_msgs_Int64 *m = RosPubLoan(pubs[k]);
    m->data = k * 1000000 + i;
    RosPublishLoaned(pubs[k], m);
  }
  __atomic_add_fetch(&pubs_done, 1, __ATOMIC_SEQ_CST);
  return NULL;
}

static void *exec_thread(void *arg)
{
  CRosExec *e = arg;
  while (__atomic_load_n(&pubs_done, __ATOMIC_SEQ_CST) < PUBS)
    RosExecSpinSome(e);
  while (RosExecSpinSome(e)) ;
  return NULL;
}

/* Phase 2: the multi-threaded executor. MT_SUBS nodes each subscribe
 * to "flood2"; publisher threads flood it while RosExecSpinMT runs the
 * callbacks on 4 threads. Each node's callbacks must never overlap. */
#define MT_SUBS 4
static I64 mt_got[MT_SUBS], mt_busy[MT_SUBS], mt_overlap, mt_total;
static CRosPub *mt_pubs[PUBS];

static void on_mt(CMsg_std_msgs_Int64 *m, U8 *user)
{
  long k = (long)user;
  if (__atomic_add_fetch(&mt_busy[k], 1, __ATOMIC_SEQ_CST) > 1)
    __atomic_add_fetch(&mt_overlap, 1, __ATOMIC_SEQ_CST);
  mt_got[k]++;  /* plain increment: TSan flags it if two threads get here */
  __atomic_add_fetch(&mt_total, 1, __ATOMIC_SEQ_CST);
  __atomic_sub_fetch(&mt_busy[k], 1, __ATOMIC_SEQ_CST);
}

static void *mt_pub_thread(void *arg)
{
  long k = (long)arg;
  for (I64 i = 1; i <= PER_PUB / 10; i++) {
    CMsg_std_msgs_Int64 *m = RosPubLoan(mt_pubs[k]);
    m->data = i;
    RosPublishLoaned(mt_pubs[k], m);
  }
  return NULL;
}

static void *mt_exec_thread(void *arg)
{
  RosExecSpinMT((CRosExec *)arg, 4);
  return NULL;
}

static int mt_check(void)
{
  CRosNode *pn = RosNodeNew("mt_pubs", NULL), *sn[MT_SUBS];
  CRosQoS q; RosQoSDefault(&q); q.depth = 100000;
  CRosExec *e = RosExecNew();
  char name[32];
  for (long k = 0; k < MT_SUBS; k++) {
    snprintf(name, sizeof name, "mt_sub%ld", k);
    sn[k] = RosNodeNew((U8 *)name, NULL);
    RosSubNew(sn[k], "flood2", ros_t_std_msgs_Int64, &q, (void *)on_mt, (U8 *)k);
    RosExecAdd(e, sn[k]);
  }
  for (int k = 0; k < PUBS; k++) mt_pubs[k] = RosPubNew(pn, "flood2", ros_t_std_msgs_Int64, NULL);
  pthread_t t[PUBS], ex;
  pthread_create(&ex, NULL, mt_exec_thread, e);
  for (long k = 0; k < PUBS; k++) pthread_create(&t[k], NULL, mt_pub_thread, (void *)k);
  for (int k = 0; k < PUBS; k++) pthread_join(t[k], NULL);
  I64 want = (I64)PUBS * (PER_PUB / 10) * MT_SUBS;
  for (int i = 0; i < 5000 && __atomic_load_n(&mt_total, __ATOMIC_SEQ_CST) < want; i++)
    usleep(1000);
  __atomic_store_n(&e->stop, 1, __ATOMIC_SEQ_CST);
  pthread_join(ex, NULL);
  printf("threads_check: multi-threaded executor: %lld of %lld callbacks, overlaps %lld\n",
         (long long)mt_total, (long long)want, (long long)mt_overlap);
  for (int k = 0; k < MT_SUBS; k++) RosNodeDel(sn[k]);
  RosNodeDel(pn); RosExecDel(e);
  return mt_total == want && mt_overlap == 0;
}

int main(void)
{
  RosIfaceLoadAll();
  ros_node_builtins = FALSE;
  CRosNode *pn = RosNodeNew("pubs", NULL), *sn = RosNodeNew("sub", NULL);
  CRosQoS q; RosQoSDefault(&q); q.depth = 1000;
  CRosSub *s = RosSubNew(sn, "flood", ros_t_std_msgs_Int64, &q, (void *)on_msg, NULL);
  for (int k = 0; k < PUBS; k++) pubs[k] = RosPubNew(pn, "flood", ros_t_std_msgs_Int64, NULL);
  CRosExec *e = RosExecNew(); RosExecAdd(e, sn);
  pthread_t t[PUBS + 1];
  pthread_create(&t[PUBS], NULL, exec_thread, e);
  for (long k = 0; k < PUBS; k++) pthread_create(&t[k], NULL, pub_thread, (void *)k);
  for (int k = 0; k <= PUBS; k++) pthread_join(t[k], NULL);
  I64 total = PUBS * PER_PUB;
  printf("threads_check: sent %lld, received %lld, dropped %lld, out of order %lld\n",
         (long long)total, (long long)received, (long long)s->dropped, (long long)out_of_order);
  int ok = received + s->dropped == total && out_of_order == 0 && received > 0;
  RosNodeDel(pn); RosNodeDel(sn); RosExecDel(e);
  if (!mt_check()) ok = 0;
  return !ok;
}
