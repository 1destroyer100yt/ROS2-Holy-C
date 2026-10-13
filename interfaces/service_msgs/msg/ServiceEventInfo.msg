# service_msgs/msg/ServiceEventInfo (ROS 2 Jazzy definition, exported via rosbags)
uint8 REQUEST_SENT=0
uint8 REQUEST_RECEIVED=1
uint8 RESPONSE_SENT=2
uint8 RESPONSE_RECEIVED=3
uint8 event_type
builtin_interfaces/Time stamp
char[16] client_gid
int64 sequence_number
