#pragma once

#include <deque>
#include <arp/Autopilot.hpp>
#include <arp/ViEkf.hpp>

class Task {
public:
    Task(arp::Autopilot& autopilot, arp::ViEkf& viEkf, Eigen::Vector3d goal, const OccupancyMap& occupancyMap);

    void execute();

    void pause();

private:

    enum class Journey {
        ToGoal,
        ToStart
    };

    void startJourney();

    void goalReachedCallback();

    void startReachedCallback();

    arp::Autopilot& autopilot_;
    arp::ViEkf& viEkf_;
    Eigen::Vector3d start_;
    Eigen::Vector3d goal_;
    const OccupancyMap& occupancyMap_;
    Journey currentJourney_{Journey::ToGoal};
    bool initialized_{false};
    bool running_{false};
};