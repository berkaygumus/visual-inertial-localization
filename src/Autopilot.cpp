/*
 * Autopilot.cpp
 *
 *  Created on: 10 Jan 2017
 *      Author: sleutene
 */

#include <arp/Autopilot.hpp>
#include <arp/kinematics/operators.hpp>

namespace arp {

Autopilot::Autopilot(ros::NodeHandle& nh)
    : nh_(&nh)
{
  isAutomatic_ = false; // always start in manual mode  

  // receive navdata
  subNavdata_ = nh.subscribe("ardrone/navdata", 50, &Autopilot::navdataCallback,
                             this);

  // commands
  pubReset_ = nh_->advertise<std_msgs::Empty>("/ardrone/reset", 1);
  pubTakeoff_ = nh_->advertise<std_msgs::Empty>("/ardrone/takeoff", 1);
  pubLand_ = nh_->advertise<std_msgs::Empty>("/ardrone/land", 1);
  pubMove_ = nh_->advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  // flattrim service
  srvFlattrim_ = nh_->serviceClient<std_srvs::Empty>(
      nh_->resolveName("ardrone/flattrim"), 1);

  // Initialize PID-Controller parameters (sensibly!)
  arp::PidController::Parameters controllerParameters;
  controllerParameters.k_p = 0.1; // 0.5
  controllerParameters.k_i = 0.0;
  controllerParameters.k_d = 0.1;
  x_controller_.setParameters(controllerParameters);
  y_controller_.setParameters(controllerParameters);
  controllerParameters.k_p = 1.0; // 0.5
  controllerParameters.k_i = 0.0;
  controllerParameters.k_d = 0.0;
  z_controller_.setParameters(controllerParameters);
  controllerParameters.k_p = 1.5; // 2.6
  controllerParameters.k_i = 0.0;
  controllerParameters.k_d = 0.0;
  yaw_controller_.setParameters(controllerParameters);
}

void Autopilot::navdataCallback(const ardrone_autonomy::NavdataConstPtr& msg)
{
  std::lock_guard<std::mutex> l(navdataMutex_);
  lastNavdata_ = *msg;
}

// Get the drone status.
Autopilot::DroneStatus Autopilot::droneStatus()
{
  ardrone_autonomy::Navdata navdata;
  {
    std::lock_guard<std::mutex> l(navdataMutex_);
    navdata = lastNavdata_;
  }
  return DroneStatus(navdata.state);
}

// Get the battery level
float Autopilot::getBatteryLevel()
{
  float batteryLevel;
  {
    std::lock_guard<std::mutex> l(navdataMutex_);
      batteryLevel = lastNavdata_.batteryPercent;
  }
  return batteryLevel;
}

// Request flattrim calibration.
bool Autopilot::flattrimCalibrate()
{
  DroneStatus status = droneStatus();
  if (status != DroneStatus::Landed) {
    return false;
  }
  // ARdrone -> flattrim calibrate
  std_srvs::Empty flattrimServiceRequest;
  srvFlattrim_.call(flattrimServiceRequest);
  return true;
}

// Takeoff.
bool Autopilot::takeoff()
{
  DroneStatus status = droneStatus();
  if (status != DroneStatus::Landed) {
    return false;
  }
  // ARdrone -> take off
  std_msgs::Empty takeoffMsg;
  pubTakeoff_.publish(takeoffMsg);
  return true;
}

// Land.
bool Autopilot::land()
{
  DroneStatus status = droneStatus();
  if (status != DroneStatus::Landed && status != DroneStatus::Landing
      && status != DroneStatus::Looping) {
    // ARdrone -> land
    std_msgs::Empty landMsg;
    pubLand_.publish(landMsg);
    return true;
  }
  return false;
}

// Turn off all motors and reboot.
bool Autopilot::estopReset()
{
  // ARdrone -> Emergency mode
  std_msgs::Empty resetMsg;
  pubReset_.publish(resetMsg);
  return true;
}

// Move the drone manually.
bool Autopilot::manualMove(double forward, double left, double up,
                           double rotateLeft)
{
  // Check for manual mode here?
  return move(forward, left, up, rotateLeft);
}

// Move the drone.
bool Autopilot::move(double forward, double left, double up, double rotateLeft)
{
  // Check for valid drone status.
  DroneStatus status = droneStatus();
  if (status != DroneStatus::Flying && status != DroneStatus::Hovering && status != DroneStatus::Flying2) {
    return false;
  }

  //check the boundaries
  if(fabs(forward) > 1.0 || fabs(left) > 1.0 || fabs(up) > 1.0 || fabs(rotateLeft) > 1.0){
    std::cout << std::endl << "desired command is out of boundary, it must be in the range [-1, 1]  "  << std::endl;
    return false;
  }
  
  geometry_msgs::Twist moveMsg;
  moveMsg.linear.x = forward; 
  moveMsg.linear.y = left; 
  moveMsg.linear.z = up; 
  moveMsg.angular.x = 0;
  moveMsg.angular.y = 0;
  moveMsg.angular.z = rotateLeft;
  pubMove_.publish(moveMsg);
  return true;
}

// Set to automatic control mode.
void Autopilot::setManual()
{
  isAutomatic_ = false;
}

// Set to manual control mode.
void Autopilot::setAutomatic()
{
  isAutomatic_ = true;
}

// Move the drone automatically.
bool Autopilot::setPoseReference(double x, double y, double z, double yaw)
{
  std::lock_guard<std::mutex> l(refMutex_);
  ref_x_ = x;
  ref_y_ = y;
  ref_z_ = z;
  ref_yaw_ = yaw;
  return true;
}

bool Autopilot::getPoseReference(double& x, double& y, double& z, double& yaw) {
  std::lock_guard<std::mutex> l(refMutex_);
  x = ref_x_;
  y = ref_y_;
  z = ref_z_;
  yaw = ref_yaw_;
  return true;
}

/// The callback from the estimator that sends control outputs to the drone
void Autopilot::controllerCallback(uint64_t timeMicroseconds,
                                  const arp::kinematics::RobotState& x)
{
  // only do anything here, if automatic
  if (!isAutomatic_) {
    // keep resetting this to make sure we use the current state as reference as soon as sent to automatic mode
    const double yaw = kinematics::yawAngle(x.q_WS);
    setPoseReference(x.t_WS[0], x.t_WS[1], x.t_WS[2], yaw);
    return;
  }

  // TODO: only enable when in flight
  DroneStatus status = droneStatus();
  if (status == DroneStatus::Flying || status == DroneStatus::Hovering || status == DroneStatus::Flying2) 
  {
    // Compute position error
    Eigen::Vector3d t_ref_WS{ref_x_, ref_y_, ref_z_};
    Eigen::Matrix3d R_SW = x.q_WS.toRotationMatrix().transpose();
    Eigen::Vector3d position_error = R_SW * (t_ref_WS - x.t_WS);
    // Compute yaw error
    double yaw_estimated = arp::kinematics::yawAngle(x.q_WS);
    double yaw_error = ref_yaw_ - yaw_estimated;
    // Ensure that yaw error is within the limits of [-pi,pi]
    yaw_error += M_PI;
    double num_shifts = floor(yaw_error / (2*M_PI));
    yaw_error += -num_shifts * 2*M_PI;
    yaw_error -= M_PI;
    // std::cout << "yaw_error: " << yaw_error << std::endl;

    // Compute the approximated time derivatives of the error signals
    Eigen::Vector3d position_error_dot = -R_SW * x.v_W;
    double yaw_error_dot = 0.0;

    // Get ros parameters (limits)
    bool success = true;
    double euler_angle_max, control_vz_max, control_yaw;
    success &= nh_->getParam("/ardrone_driver/euler_angle_max", euler_angle_max);
    success &= nh_->getParam("/ardrone_driver/control_vz_max", control_vz_max);
    success &= nh_->getParam("/ardrone_driver/control_yaw", control_yaw);
    if (!success) {
      ROS_ERROR("Error reading ROS parameters (limits).");
      return;
    }
    // Convert control_vz_max from mm/s to m/s
    control_vz_max *= 1.0e-3;
    
    // Set the output limits of the contollers
    x_controller_.setOutputLimits(-euler_angle_max, euler_angle_max);
    y_controller_.setOutputLimits(-euler_angle_max, euler_angle_max);
    z_controller_.setOutputLimits(-control_vz_max, control_vz_max);
    yaw_controller_.setOutputLimits(-control_yaw, control_yaw);

    double output_x = x_controller_.control(timeMicroseconds, position_error(0), position_error_dot(0));
    double output_y = y_controller_.control(timeMicroseconds, position_error(1), position_error_dot(1));
    double output_z = z_controller_.control(timeMicroseconds, position_error(2), position_error_dot(2));
    double output_yaw = yaw_controller_.control(timeMicroseconds, yaw_error, yaw_error_dot);

    // Scale the controller outputs for the move command
    double eps = 1.0e-6;
    if (euler_angle_max < eps || control_vz_max < eps || control_yaw < eps) // Check if limits are too small before dividing!
    {
      std::cout << "Output limits of controller close to zero (division by zero). Move command won't be sent" << std::endl;
      return;
    }
    output_x /= euler_angle_max;
    output_y /= euler_angle_max;
    output_z /= control_vz_max;
    output_yaw /= control_yaw;

    // Command the drone to move
    move(output_x, output_y, output_z, output_yaw);
  }

  // TODO: get ros parameter
  
  // TODO: compute control output

  // TODO: send to move

}

}  // namespace arp

