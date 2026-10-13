//Small helpers that converted C++ code (cpp2hc output) and
//the rest of ROS2-Holy-C rely on. Keep this file tiny and
//free of anything TempleOS/ZealOS specific beyond the basics.

#define ROS_LOG_DEBUG	0
#define ROS_LOG_INFO	1
#define ROS_LOG_WARN	2
#define ROS_LOG_ERROR	3
#define ROS_LOG_FATAL	4

I64 ros_log_level=ROS_LOG_INFO;

//Print one log line with a level tag. fmt/argc/argv come from
//the variadic wrappers below.
U0 RosLogJoin(I64 level,U8 *fmt,I64 argc,I64 *argv)
{
  U8 *buf;
  if (level<ros_log_level) return;
  buf=StrPrintJoin(NULL,fmt,argc,argv);
  switch (level) {
    case ROS_LOG_DEBUG: Print("[DEBUG] %s\n",buf); break;
    case ROS_LOG_INFO:  Print("[INFO] %s\n",buf);  break;
    case ROS_LOG_WARN:  Print("[WARN] %s\n",buf);  break;
    case ROS_LOG_ERROR: Print("[ERROR] %s\n",buf); break;
    default:            Print("[FATAL] %s\n",buf); break;
  }
  Free(buf);
}

U0 RosLogDebug(U8 *fmt,...) {RosLogJoin(ROS_LOG_DEBUG,fmt,argc,argv);}
U0 RosLogInfo(U8 *fmt,...)  {RosLogJoin(ROS_LOG_INFO,fmt,argc,argv);}
U0 RosLogWarn(U8 *fmt,...)  {RosLogJoin(ROS_LOG_WARN,fmt,argc,argv);}
U0 RosLogError(U8 *fmt,...) {RosLogJoin(ROS_LOG_ERROR,fmt,argc,argv);}
U0 RosLogFatal(U8 *fmt,...) {RosLogJoin(ROS_LOG_FATAL,fmt,argc,argv);}

//F64 min/max for converted std::min/std::max.
F64 CppMin(F64 a,F64 b) {if (a<b) return a; return b;}
F64 CppMax(F64 a,F64 b) {if (a>b) return a; return b;}
