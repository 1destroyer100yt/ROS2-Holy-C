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
  return !ok;
}
