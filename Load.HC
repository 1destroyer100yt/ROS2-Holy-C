//Load ROS2-Holy-C: the library, every message/service package and the
//ROS graph layer.
//  #include "~/ROS2-Holy-C/Load.HC"

#ifndef ROS2_HOLYC_LOADED
#define ROS2_HOLYC_LOADED 1
Cd(__DIR__);;
#include "src/Load.HC"
Cd(__DIR__);;
#include "interfaces/Load.HC"
Cd(__DIR__);;
#include "src/Graph/Load.HC"
Cd(__DIR__);;
#include "src/Tools/Load.HC"
Cd(__DIR__);;
#include "src/Tf/Load.HC"
Cd(__DIR__);;
#include "src/Viz/Load.HC"
Cd(__DIR__);;
#include "src/Net/Load.HC"
Cd(__DIR__);;
#include "src/Dds/Load.HC"
#endif
