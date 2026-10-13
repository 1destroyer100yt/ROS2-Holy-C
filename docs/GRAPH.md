# Nodes, topics, services, actions and parameters

Everything lives in `src/Graph/`. Load it all with `#include "~/ROS2-Holy-C/Load.HC"`.
HolyC has no lambdas or closures, so every callback takes a `user` pointer you pass in when
you create it. Put whatever state you need in a class and hand that over.

## Nodes
```
CRosNode *n=RosNodeNew("talker","/robot1");   //namespace optional (NULL = "/")
RosNodeRemap(n,"chatter","/news");            //like -r chatter:=/news
RosNodeLog(n,ROS_LOG_INFO,"hello");           //prints + publishes on /rosout
RosNodeInfo(n,"x=%d",x);                      //printf-style (TempleOS only)
RosNodeDel(n);                                //also deletes everything it made
```
Names follow ROS 2: `chatter` → `/robot1/chatter`, `/abs` stays, `~/state` → `/robot1/talker/state`.

## Topics
```
CRosPub *p=RosPubNew(n,"chatter",ros_t_std_msgs_String,NULL);   //NULL = default QoS
CMsg_std_msgs_String *m=RosPubLoan(p);   //zero copy: fill it in, then
m->data=StrNew("hi");
RosPublishLoaned(p,m);                   //...ownership passes to ROS
RosPublish(p,&my_msg);                   //or publish a copy of your own message

U0 OnMsg(CMsg_std_msgs_String *m,U8 *user) { Print("%s\n",m->data); }
CRosSub *s=RosSubNew(n,"chatter",ros_t_std_msgs_String,NULL,&OnMsg,NULL);
```
- Every subscriber gets **the same message pointer** (reference counted). Treat it as read-only.
- `cb=NULL` makes a polling subscription: `m=RosSubTake(s); ...; RosMsgRelease(m);`
- `RosPubSubCount(p)` / `RosSubPubCount(s)` count matched endpoints.

### QoS
`CRosQoS q; RosQoSDefault(&q);` (keep last 10, reliable, volatile). Other presets:
`RosQoSSensorData` (keep last 5, best effort), `RosQoSTransientLocal(&q,depth)` ("latched"),
and `RosQoSParameters`. You can also set the fields yourself, for example `q.lifespan_ns`.
Matching follows ROS 2: a best-effort publisher can't feed a reliable subscription, and a
volatile publisher can't feed a transient-local one. Incompatible pairs are counted in
`->incompatible`.

## Timers
```
U0 Tick(CMyState *st) {...}
RosTimerNew(n,100*ROS_NS_PER_MS,&Tick,st);
```

## Services
```
U0 Add(CSrv_example_interfaces_AddTwoInts_Request *req,
       CSrv_example_interfaces_AddTwoInts_Response *resp,U8 *user) { resp->sum=req->a+req->b; }
RosServiceNew(n,"add_two_ints",ros_srv_example_interfaces_AddTwoInts,&Add,NULL);

CRosClient *c=RosClientNew(n,"add_two_ints",ros_srv_example_interfaces_AddTwoInts);
RosClientCall(c,&req,&OnSum,NULL);           //async: OnSum(resp,seq,user)
resp=RosClientCallWait(c,exec,&req,timeout); //blocking: spins exec while waiting
```

A service callback can answer later: `ticket=RosServiceDefer(s);` inside the callback, then
`RosServiceReplyLater(s,ticket,resp);` when the answer is ready.

## Actions
Long-running goals with feedback, wire-compatible with rclcpp/rclpy (`<name>/_action/send_goal`,
`cancel_goal`, `get_result`, `feedback` and `status`, as rcl_action lays them out):
```
Bool OnGoal(CRosActionServer *as,CAct_example_interfaces_Fibonacci_Goal *g,U8 *user)
{ return g->order<50; }                          //accept or reject (NULL = accept all)
U0 OnAccepted(CRosGoal *g,U8 *user) { RosGoalExecute(g); ... }   //start the work
as=RosActionServerNew(n,"fibonacci",ros_act_example_interfaces_Fibonacci,
                      &OnGoal,NULL,&OnAccepted,NULL);  //NULL cancel callback = allow
RosGoalFeedback(g,&fb);                          //while working
if (RosGoalIsCanceling(g)) RosGoalCanceled(g,&res);
RosGoalSucceed(g,&res);                          //or RosGoalAbort

ac=RosActionClientNew(n,"fibonacci",ros_act_example_interfaces_Fibonacci);
gh=RosActionSendGoal(ac,&goal,&OnFeedback,&OnResult,user);
RosActionCancel(gh);
```
`.action` files in `interfaces/` generate `CAct_pkg_Name_Goal`, `_Result`, `_Feedback` (and
the `_SendGoal`/`_GetResult` service types and `_FeedbackMessage`) plus a `ros_act_pkg_Name`
handle. Results are kept for 15 minutes after a goal ends, like rcl. Turtlesim serves
`/turtle1/rotate_absolute`. Delete action servers and clients before their node.

## Parameters
```
RosParamLoadYaml(text);  //or RosParamFile("~/params.yaml"); applies to nodes made later
RosParamDeclare(n,"rate","10.0");            //type comes from the default
F64 hz=RosParamF64(n,"rate");                //also RosParamI64/Bool/Str/Text
RosParamSet(n,"rate","20");                  //FALSE + ros_param_error if refused
RosParamOnSet(n,&Guard,NULL);                //Guard returns FALSE to refuse a change
```
Every node serves `~/get_parameters`, `~/set_parameters`, `~/list_parameters`,
`~/describe_parameters`, `~/get_parameter_types` and `~/set_parameters_atomically`, and it
publishes `/parameter_events`, the same as ROS 2. One difference: an integer is accepted for a
double parameter, so `rate: 2` in YAML works.

## Executors
Callbacks run in whichever task spins the node:
```
CRosExec *e=RosExecNew(); RosExecAdd(e,n1); RosExecAdd(e,n2);
RosExecSpin(e);              //until RosShutdown()
RosExecSpinSome(e);          //run what's ready now
RosExecSpinOnce(e,timeout);  //wait for work
RosSpin(n);                  //one node, its own executor
```
In TempleOS, give each part of the robot its own task:
`Spawn(&MyTask,NULL,"name");`, where `MyTask` makes its node and calls `RosSpin`.

Or let one executor use several cores:
```
RosExecSpinMT(e,4);          //this task + 3 worker tasks on other cores
RosExecStop(e);              //from a callback or another task: all of them return
```
Each node is a mutually exclusive callback group (rclcpp's default): one callback of a node
runs at a time, and different nodes run in parallel. Split work that should overlap into
separate nodes. Don't add nodes to `e` or spin it any other way while it runs.
`tools/hctest/threads_check.c` floods four nodes from three publisher threads under
ThreadSanitizer to check this.
`examples/TalkerListener.HC` does exactly this.

**Rule for now:** create and delete a node's publishers, subscriptions, timers, services and
clients in the same task that spins that node. Publishing and calling services from any task
is fine.
