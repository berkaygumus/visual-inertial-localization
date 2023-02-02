#pragma once

#include <vector>
#include <arp/Autopilot.hpp>
#include <Eigen/Core>
#include <OccupancyMap.h>

namespace Planner {

std::vector<arp::Autopilot::Waypoint> planFlight(const Eigen::Vector3d& start, 
                                                 const Eigen::Vector3d& goal, 
                                                 const OccupancyMap& occupancyMap);

}