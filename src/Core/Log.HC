//Log: RosLogInfo("x=%d",x) etc. Lines look like ROS 2's:
//  [INFO] [12.345678] [talker]: Publishing: 'Hello 1'
//The logger name comes from ros_log_name (the node sets it).

#define ROS_LOG_DEBUG	0
#define ROS_LOG_INFO	1
#define ROS_LOG_WARN	2
#define ROS_LOG_ERROR	3
#define ROS_LOG_FATAL	4

I64 ros_log_level=ROS_LOG_INFO;
U8 *ros_log_name="";
Bool ros_log_quiet=FALSE;	//count but don't print (tests)
I64 ros_log_counts[5];

U8 *RosLogLevelName(I64 level)
{
  switch (level) {
    case ROS_LOG_DEBUG: return "DEBUG";
    case ROS_LOG_INFO:  return "INFO";
    case ROS_LOG_WARN:  return "WARN";
    case ROS_LOG_ERROR: return "ERROR";
  }
  return "FATAL";
}

U8 *RosLogLevelColor(I64 level)
{
  switch (level) {
    case ROS_LOG_DEBUG: return "$$FG,GREEN$$";
    case ROS_LOG_WARN:  return "$$FG,BROWN$$";
    case ROS_LOG_ERROR: return "$$FG,RED$$";
    case ROS_LOG_FATAL: return "$$FG,LTRED$$";
  }
  return "$$FG$$";
}

I64 RosLogClamp(I64 level)
{
  if (level<ROS_LOG_DEBUG)
    return ROS_LOG_DEBUG;
  if (level>ROS_LOG_FATAL)
    return ROS_LOG_FATAL;
  return level;
}

//Print a log line without level filtering.
U0 RosLogEmit(I64 level,U8 *name,U8 *msg)
{
  I64 t;
  level=RosLogClamp(level);
  ros_log_counts[level]++;
  if (ros_log_quiet)
    return;
  t=RosClockNow(ROS_CLOCK_ROS);
  Print("%s[%s] [%d.%06d] [%s]: %s$$FG$$\n",RosLogLevelColor(level),RosLogLevelName(level),
	RosTimeSec(t),RosTimeNsec(t)/1000,name,msg);
}

//Write an already formatted message. Returns TRUE if it passed the level filter.
Bool RosLogWrite(I64 level,U8 *name,U8 *msg)
{
  if (RosLogClamp(level)<ros_log_level)
    return FALSE;
  RosLogEmit(level,name,msg);
  return TRUE;
}

//@host-skip-begin
//Variadic front ends (TempleOS only; argc/argv are HolyC's ... args).
U0 RosLogJoin(I64 level,U8 *fmt,I64 argc,I64 *argv)
{
  U8 *buf;
  if (level<ros_log_level)
    return;
  buf=StrPrintJoin(NULL,fmt,argc,argv);
  RosLogWrite(level,ros_log_name,buf);
  Free(buf);
}

U0 RosLogDebug(U8 *fmt,...) {RosLogJoin(ROS_LOG_DEBUG,fmt,argc,argv);}
U0 RosLogInfo(U8 *fmt,...)  {RosLogJoin(ROS_LOG_INFO,fmt,argc,argv);}
U0 RosLogWarn(U8 *fmt,...)  {RosLogJoin(ROS_LOG_WARN,fmt,argc,argv);}
U0 RosLogError(U8 *fmt,...) {RosLogJoin(ROS_LOG_ERROR,fmt,argc,argv);}
U0 RosLogFatal(U8 *fmt,...) {RosLogJoin(ROS_LOG_FATAL,fmt,argc,argv);}
//@host-skip-end
