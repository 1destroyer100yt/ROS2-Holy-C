#!/bin/bash
# The HolyC stack against a real ROS 2 Jazzy install, with both
# Fast DDS (the default) and Cyclone DDS:
#   checks.sh  the ros2 CLI on a HolyC node: node list, topic echo/pub,
#              service call, action send_goal, param get, type hashes
#   client.sh  HolyC calling Python nodes (service, action, talker),
#              and rosbag2 both ways (ros2 bag record/info/play)
#   tools/ros2test/run.sh            (needs ros2 on PATH, ROS 2 sourced)
#   ROS_SETUP=/path/to/setup.bash tools/ros2test/run.sh
cd "$(dirname "$0")/../.."
[ -n "$ROS_SETUP" ] && source "$ROS_SETUP"
if ! command -v ros2 >/dev/null; then echo "ros2test: no ROS 2 found, skipping"; exit 0; fi
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_DOMAIN_ID=0
fail=0
for rmw in ${RMWS:-rmw_fastrtps_cpp rmw_cyclonedds_cpp}; do
  echo "== $rmw"
  tmp=$(mktemp -d)
  DEMO_SECONDS=120 DEMO_STOP=$tmp/stop python3 tools/hctest/hc2c.py run tests/ros2/Demo.HC \
    --main Ros2Demo > $tmp/demo.log 2>&1 &
  demo=$!
  for i in $(seq 1 300); do grep -q READY $tmp/demo.log && break; sleep 0.1; done
  RMW_IMPLEMENTATION=$rmw tools/ros2test/checks.sh | tee $tmp/checks.log
  touch $tmp/stop
  wait $demo
  grep RESULT $tmp/demo.log
  grep -q "FAIL" $tmp/checks.log && fail=1
  grep -q "heard=[1-9]" $tmp/demo.log || { echo "CHECK topic_pub FAIL (HolyC heard nothing)"; fail=1; }
  RMW_IMPLEMENTATION=$rmw tools/ros2test/client.sh | tee $tmp/client.log
  grep -q "FAIL" $tmp/client.log && fail=1
  rm -rf "$tmp"
done
[ $fail = 0 ] && echo "ros2test: OK" || { echo "ros2test: FAIL"; exit 1; }
