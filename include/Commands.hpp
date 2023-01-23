#pragma once

#include <arp/Autopilot.hpp>
#include <Renderer.hpp>
#include <arp/VisualInertialTracker.hpp>
#include <arp/InteractiveMarkerServer.hpp>

namespace Commands {

/// @brief Check all associated keys for a command.
/// @param autopilot 
/// @param renderer
/// @param visualInertialTracker
/// @param markerServer
void checkKeysForCommand(arp::Autopilot& autopilot, gui::Renderer& renderer,
                         arp::VisualInertialTracker& visualInertialTracker, 
                         arp::InteractiveMarkerServer& markerServer);

}