/*
 * Imu.cpp
 *
 *  Created on: 8 Feb 2017
 *      Author: sleutene
 */

#include <arp/kinematics/Imu.hpp>
#include <iostream>

namespace arp {
namespace kinematics {

bool Imu::stateTransition(const RobotState & state_k_minus_1,
                          const ImuMeasurement & z_k_minus_1,
                          const ImuMeasurement & z_k, RobotState & state_k,
                          ImuKinematicsJacobian* jacobian)
{
  // get the time delta
  const double dt = double(
      z_k.timestampMicroseconds - z_k_minus_1.timestampMicroseconds) * 1.0e-6;
  if (dt < 1.0e-12 || dt > 0.05) {
    // for safety, we assign reasonable values here.
    state_k = state_k_minus_1;
    if(jacobian) {
      jacobian->setIdentity();
    }
    return false;  // negative, no or too large time increments not permitted
  }

  // TODO: implement trapezoidal integration
  
  //////////// Compute current state ///////////////////
  
  // Create gravity vector
  Eigen::Vector3d g_W{0, 0, -9.81};
  
  // Get the rotation matrix R_WS_k_minus_1
  Eigen::Matrix3d R_WS_k_minus_1 = state_k_minus_1.q_WS.matrix();
  
  // Compute temp state
  RobotState temp;
  temp.t_WS = state_k_minus_1.t_WS + dt*state_k_minus_1.v_W;
  // temp.q_WS = ;
  temp.v_W = state_k_minus_1.v_W + dt*(R_WS_k_minus_1*(z_k.acc_S - state_k_minus_1.b_a) + g_W);
  temp.b_g = state_k_minus_1.b_g;
  temp.b_a = state_k_minus_1.b_a;
  
  // compute next translation vector
  // t_WS
  state_k.t_WS = state_k_minus_1.t_WS + 0.5*(dt*state_k_minus_1.v_W + dt*temp.t_WS);
  // q_WS
  const Eigen::Vector3d& delta_alpha_1 = dt*R_WS_k_minus_1*(z_k_minus_1.omega_S - state_k_minus_1.b_g);
  const Eigen::Quaterniond q_WS_k_minus_1_step = (deltaQ(delta_alpha_1) * state_k_minus_1.q_WS).normalized(); 
  const Eigen::Vector3d& delta_alpha_2 = dt*q_WS_k_minus_1_step.matrix()*(z_k.omega_S - state_k_minus_1.b_g);   // maybe replace state_k_minus_1.b_g by state_k.b_g to be consistent with the task sheet (even though they are the same)
  state_k.q_WS = (deltaQ(0.5*delta_alpha_1 + 0.5*delta_alpha_2) * state_k_minus_1.q_WS).normalized();
  // v_W
  state_k.v_W = state_k_minus_1.v_W + 0.5*dt*(R_WS_k_minus_1*(z_k_minus_1.acc_S - state_k_minus_1.b_a) + g_W + state_k_minus_1.v_W + dt*(R_WS_k_minus_1*(z_k.acc_S - state_k_minus_1.b_a)+ g_W));
  // b_g
  state_k.b_g = state_k_minus_1.b_g;
  // b_a
  state_k.b_a = state_k_minus_1.b_a;

  if (jacobian) {
    // TODO: if requested, impement jacobian of trapezoidal integration with chain rule
    ImuKinematicsJacobian Fc_xkMinus1_tkMinus1;
    Fc_xkMinus1_tkMinus1.setZero();
    Fc_xkMinus1_tkMinus1.block(0,6,3,3) = Eigen::Matrix3d::Identity();
    Fc_xkMinus1_tkMinus1.block(3,3,3,3) = -crossMx(R_WS_k_minus_1*(z_k_minus_1.omega_S - state_k_minus_1.b_g));
    Fc_xkMinus1_tkMinus1.block(6,3,3,3) = -crossMx(R_WS_k_minus_1*(z_k_minus_1.acc_S - state_k_minus_1.b_a));
    Fc_xkMinus1_tkMinus1.block(3,9,3,3) = -R_WS_k_minus_1;
    Fc_xkMinus1_tkMinus1.block(6,12,3,3) = -R_WS_k_minus_1;

    ImuKinematicsJacobian Fc_xkMinus1_plus_delta_x1_tkMinus1;
    Fc_xkMinus1_plus_delta_x1_tkMinus1.setZero();
    Fc_xkMinus1_tkMinus1.block(0,6,3,3) = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d R_WS_k_minus_1_plus_delta = state_k.q_WS.matrix(); // How does delta_x_1 change R_WS???
    Fc_xkMinus1_tkMinus1.block(3,3,3,3) = -crossMx(R_WS_k_minus_1_plus_delta*(z_k.omega_S - state_k.b_g));
    Fc_xkMinus1_tkMinus1.block(6,3,3,3) = -crossMx(R_WS_k_minus_1_plus_delta*(z_k.acc_S - state_k.b_a));
    Fc_xkMinus1_tkMinus1.block(3,9,3,3) = -R_WS_k_minus_1_plus_delta;
    Fc_xkMinus1_tkMinus1.block(6,12,3,3) = -R_WS_k_minus_1_plus_delta;
    
    ImuKinematicsJacobian I_15;
    I_15.setIdentity();
    *jacobian = I_15 + 0.5*dt*(Fc_xkMinus1_tkMinus1 + Fc_xkMinus1_plus_delta_x1_tkMinus1*(I_15 + dt*Fc_xkMinus1_tkMinus1));
    jacobian->block(3,3,3,3).setIdentity();
    jacobian->block(3,9,3,3) = -0.5*dt*(R_WS_k_minus_1 + state_k.q_WS.matrix()); // maybe need to get state_k.q_WS.matrix() from somewhere else.
    jacobian->block(6,3,3,3) = -0.5*dt*crossMx(R_WS_k_minus_1*(z_k_minus_1.acc_S - state_k_minus_1.b_a) + state_k.q_WS.matrix()*(z_k.acc_S - state_k.b_a));  // maybe need to get state_k.q_WS.matrix() from somewhere else. Can we use "z_k.acc_S" and "state_k.b_a" here, since we get them from the non-linear computation. 
  }
  return true;
}

}
}  // namespace arp

