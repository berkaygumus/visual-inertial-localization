#include <Planner.h>

namespace Planner {

std::deque<arp::Autopilot::Waypoint> planFlight(const Eigen::Vector3d& start, 
                                                const Eigen::Vector3d& goal, 
                                                const OccupancyMap& occupancyMap)
{
    // TODO
    return { arp::Autopilot::Waypoint{goal(0), goal(1), goal(2), 0, 0.2} };
}

}