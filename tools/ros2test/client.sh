#!/bin/bash
# HolyC as a client of real ROS 2 nodes, and rosbag2 both ways.
# Run by run.sh with ROS 2 sourced and RMW_IMPLEMENTATION set.
tmp=$(mktemp -d)
pids=()
setsid ros2 run demo_nodes_py add_two_ints_server >/dev/null 2>&1 & pids+=($!)
setsid ros2 run action_tutorials_py fibonacci_action_server >/dev/null 2>&1 & pids+=($!)
setsid ros2 run demo_nodes_py talker >/dev/null 2>&1 & pids+=($!)
timeout 15 ros2 bag record -s mcap -o $tmp/ros_bag /chatter >/dev/null 2>&1
BAG_IN=$tmp/ros_bag BAG_OUT=$tmp/holyc_bag timeout 120 python3 tools/hctest/hc2c.py run \
  tests/ros2/Client.HC --main Ros2Client > $tmp/client.log 2>&1
grep -q "^CHECK" $tmp/client.log || { echo "CHECK holyc_client FAIL"; tail -20 $tmp/client.log; }
sed -n '/^CHECK/,$p' $tmp/client.log | grep -v "^\[" | head -60
info=$(timeout 30 ros2 bag info $tmp/holyc_bag 2>&1)
if grep -q "Count: 5" <<<"$info" && grep -q "/holyc_chatter" <<<"$info"; then echo "CHECK ros2_bag_info ok"
else echo "CHECK ros2_bag_info FAIL"; echo "$info" | sed 's/^/    /'; fi
play=$( (timeout 25 ros2 topic echo /holyc_chatter std_msgs/msg/String > $tmp/echo.txt 2>&1 &)
  sleep 3; timeout 20 ros2 bag play --delay 5 $tmp/holyc_bag 2>&1; sleep 3; cat $tmp/echo.txt)
if grep -q "from holyc" <<<"$play"; then echo "CHECK ros2_bag_play ok"
else echo "CHECK ros2_bag_play FAIL"; echo "$play" | tail -5 | sed 's/^/    /'; fi
for p in "${pids[@]}"; do kill -- -$p 2>/dev/null; done  # whole groups: ros2 run + its child
wait 2>/dev/null
rm -rf "$tmp"
