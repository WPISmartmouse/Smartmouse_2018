#pragma once

// all generates includes should be listed here
#include <sim/maze_location.pb.h>
#include "state.pb.h"

typedef const boost::shared_ptr<gzmaze::msgs::RobotState const> ConstRobotStatePtr;
typedef const boost::shared_ptr<gzmaze::msgs::MazeLocation const> ConstMazeLocationPtr;
