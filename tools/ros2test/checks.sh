#!/bin/bash
# The ros2 command line against the HolyC demo node (tests/ros2/Demo.HC).
# Run by run.sh with ROS 2 sourced and RMW_IMPLEMENTATION set.
# Prints one "CHECK name ok|FAIL" line per check.
T=20
check() { # name, expected text, command... (3 tries: each ros2 run rediscovers)
  local name=$1 want=$2 out i; shift 2
  for i in 1 2 3; do
    out=$(timeout $T "$@" 2>&1)
    if grep -qF -- "$want" <<<"$out"; then echo "CHECK $name ok"; return; fi
  done
  echo "CHECK $name FAIL"; echo "$out" | head -15 | sed 's/^/    /'
}
for i in 1 2 3 4 5 6 7 8 9 10; do  # discovery takes a moment
  ros2 node list --no-daemon --spin-time 2 2>/dev/null | grep -q /holyc_demo && break; sleep 1; done
check node_list /holyc_demo ros2 node list --no-daemon --spin-time 5
check topic_echo "data: 'Hello from HolyC: " ros2 topic echo /chatter std_msgs/msg/String --once
check type_hash "RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18" \
  ros2 topic info -v /chatter --no-daemon --spin-time 5
check service_call "sum=5" ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 2, b: 3}"
check action_goal "Goal finished with status: SUCCEEDED" \
  ros2 action send_goal -f /fibonacci example_interfaces/action/Fibonacci "{order: 5}"
check param_get "String value is: Hello from HolyC" ros2 param get /holyc_demo greeting
timeout $T ros2 topic pub -t 15 -r 10 /ros_chatter std_msgs/msg/String "{data: hi from ros2}" >/dev/null 2>&1
echo "CHECK topic_pub sent"
