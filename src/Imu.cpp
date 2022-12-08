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

ImuKinematicsJacobian Fc(const RobotState &state, const ImuMeasurement &z)
{
  auto R_WS = state.q_WS.matrix();
  ImuKinematicsJacobian Fc;
  Fc.setZero();
  Fc.block(0,6,3,3) = Eigen::Matrix3d::Identity();
  Fc.block(3,3,3,3) = -crossMx(R_WS*(z.omega_S - state.b_g));
  Fc.block(6,3,3,3) = -crossMx(R_WS*(z.acc_S - state.b_a));
  Fc.block(3,9,3,3) = -R_WS;
  Fc.block(6,12,3,3) = -R_WS;
  return Fc;
}

DeltaRobotState delta_t_fc(const float &dt, const RobotState &state, const ImuMeasurement &z, const Eigen::Vector3d &g_W)
{
  auto R_WS = state.q_WS.matrix();
  DeltaRobotState delta_chi;
  delta_chi.delta_t_WS = dt * state.v_W;
  delta_chi.delta_alpha_WS = dt * R_WS*(z.omega_S - state.b_g);
  delta_chi.delta_v_W = dt * (R_WS*(z.acc_S - state.b_a) + g_W);
  delta_chi.delta_b_g.setZero();
  delta_chi.delta_b_a.setZero();
  return delta_chi;
}

RobotState boxPlus(const RobotState &state, const DeltaRobotState &delta_state)
{
  RobotState res_state;
  res_state.t_WS = state.t_WS + delta_state.delta_t_WS;
  res_state.q_WS = (deltaQ(delta_state.delta_alpha_WS) * state.q_WS).normalized();
  res_state.v_W = state.v_W + delta_state.delta_v_W;
  res_state.b_g = state.b_g + delta_state.delta_b_g;
  res_state.b_a = state.b_a + delta_state.delta_b_a;
  return res_state;
}

bool Imu::stateTransition(const RobotState & state_k_minus_1,
                          const ImuMeasurement & z_k_minus_1,
                          const ImuMeasurement & z_k, RobotState & state_k,
                          ImuKinematicsJacobian* jacobian)
{
  // get the time delta
  const double dt = double(z_k.timestampMicroseconds - z_k_minus_1.timestampMicroseconds) * 1.0e-6;
  if (dt < 1.0e-12 || dt > 0.05) {
    // for safety, we assign reasonable values here.
    state_k = state_k_minus_1;
    if(jacobian) {
      jacobian->setIdentity();
    }
    return false;  // negative, no or too large time increments not permitted
  }

  //// TRAPEZOIDAL INTEGRATION ////
  // Create gravity vector
  Eigen::Vector3d g_W{0, 0, -9.81};
  
  // Create the two delta_chi states
  DeltaRobotState delta_chi_1, delta_chi_2;
  
  // Chi_1
  delta_chi_1 = delta_t_fc(dt, state_k_minus_1, z_k_minus_1, g_W);
  
  // x_k-1 [+] delta_chi_1
  RobotState state_k_minus_1_plus_delta;
  state_k_minus_1_plus_delta = boxPlus(state_k_minus_1, delta_chi_1);

  // Chi_2
  delta_chi_2 = delta_t_fc(dt, state_k_minus_1_plus_delta, z_k, g_W);

  // delta_step = 1/2 * (Chi_1 + Chi_2)
  DeltaRobotState delta_step;
  delta_step.delta_t_WS = 0.5*(delta_chi_1.delta_t_WS + delta_chi_2.delta_t_WS);
  delta_step.delta_alpha_WS = 0.5*(delta_chi_1.delta_alpha_WS + delta_chi_2.delta_alpha_WS);
  delta_step.delta_v_W = 0.5*(delta_chi_1.delta_v_W + delta_chi_2.delta_v_W);
  delta_step.delta_b_g = 0.5*(delta_chi_1.delta_b_g + delta_chi_2.delta_b_g);
  delta_step.delta_b_a = 0.5*(delta_chi_1.delta_b_a + delta_chi_2.delta_b_a);

  // x_k = x_k-1 [+] delta_step
  state_k = boxPlus(state_k_minus_1, delta_step);
  
  if (jacobian) {
    // Get the rotation matrices R_WS_k_minus_1
    Eigen::Matrix3d R_WS_k = state_k.q_WS.matrix();
    Eigen::Matrix3d R_WS_k_minus_1 = state_k_minus_1.q_WS.matrix();
  
    ImuKinematicsJacobian Fc_xkMinus1_tkMinus1;
    Fc_xkMinus1_tkMinus1 = Fc(state_k_minus_1, z_k_minus_1);
    
    ImuKinematicsJacobian Fc_xkMinus1_plus_delta_x1_tkMinus1;
    Fc_xkMinus1_plus_delta_x1_tkMinus1 = Fc(state_k_minus_1_plus_delta, z_k);
    
    ImuKinematicsJacobian I_15;
    I_15.setIdentity();
    *jacobian = I_15 + 0.5*dt*Fc_xkMinus1_tkMinus1 + 0.5*dt*Fc_xkMinus1_plus_delta_x1_tkMinus1*(I_15 + dt*Fc_xkMinus1_tkMinus1);
    jacobian->block(3,3,3,3).setIdentity();
    jacobian->block(3,9,3,3) = -0.5*dt*(R_WS_k_minus_1 + R_WS_k);   // are we allowed to use the components of state_k here??? 
    jacobian->block(6,3,3,3) = -0.5*dt*crossMx(R_WS_k_minus_1*(z_k_minus_1.acc_S - state_k_minus_1.b_a) + R_WS_k*(z_k.acc_S - state_k.b_a));  // maybe need to get state_k.q_WS.matrix() from somewhere else. Can we use "z_k.acc_S" and "state_k.b_a" here, since we get them from the non-linear computation. 
  }
  return true;
}

}
}  // namespace arp

