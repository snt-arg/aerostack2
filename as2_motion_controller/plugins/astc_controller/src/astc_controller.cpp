/*!*******************************************************************************************
 *  \file       astc_controller_plugin.hpp
 *  \brief      Adaptive Super Twisting Controller plugin for the Aerostack framework.
 *  \authors    Paul Kremer
 *
 *  \copyright  Copyright (c) 2023 University of Luxembourg
 *              All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************************/

#include "astc_controller.hpp"
#include "astc_controller/utils.hpp"

namespace astc_controller {

#define DECLARE_PARAM(param, param_name, param_var)                                            \
  if (param.get_name() == param_name) {                                                        \
    printf("'%s' %f -> %f\n", param.get_name().c_str(), param_var, param.get_value<double>()); \
    param_var = param.get_value<double>();                                                     \
  }

Plugin::Plugin() : controller_(node_ptr_->get_logger()) {}

void Plugin::ownInitialize() {
  tf_handler_ = std::make_shared<as2::tf::TfHandler>(node_ptr_);

  enu_frame_id_ = as2::tf::generateTfName(node_ptr_, enu_frame_id_);
  flu_frame_id_ = as2::tf::generateTfName(node_ptr_, flu_frame_id_);

  input_pose_frame_id_  = as2::tf::generateTfName(node_ptr_, input_pose_frame_id_);
  input_twist_frame_id_ = as2::tf::generateTfName(node_ptr_, input_twist_frame_id_);

  output_twist_frame_id_ = as2::tf::generateTfName(node_ptr_, output_twist_frame_id_);
  output_pose_frame_id_  = as2::tf::generateTfName(node_ptr_, output_pose_frame_id_);

  debug_pub_ = node_ptr_->create_publisher<as2_msgs::msg::ControllerDebug>("controller_debug",
                                                                           rclcpp::SensorDataQoS());

  reset();
};

bool Plugin::updateParams(const std::vector<rclcpp::Parameter> &_params_list) {
  auto result = parametersCallback(_params_list);
  return result.successful;
};

void Plugin::reset() {
  resetReferences();
  resetState();
  resetCommands();
  controller_.reset();
}

void Plugin::resetState() { uav_state_ = UAV_state(); }

void Plugin::resetReferences() {
  control_ref_.position = uav_state_.position;
  control_ref_.lin_vel  = Eigen::Vector3d::Zero();

  Eigen::Vector3d eul_uav = utils::quatToYPR(uav_state_.orientation);
  control_ref_.yaw        = eul_uav[0];
}

void Plugin::resetCommands() { controller_.reset(); }

void Plugin::updateState(const geometry_msgs::msg::PoseStamped &pose_msg,
                         const geometry_msgs::msg::TwistStamped &twist_msg) {
  if (pose_msg.header.frame_id != input_pose_frame_id_ &&
      twist_msg.header.frame_id != input_twist_frame_id_) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Pose and Twist frame_id are not desired ones");
    RCLCPP_ERROR(node_ptr_->get_logger(), "Recived: %s, %s", pose_msg.header.frame_id.c_str(),
                 twist_msg.header.frame_id.c_str());
    RCLCPP_ERROR(node_ptr_->get_logger(), "Desired: %s, %s", input_pose_frame_id_.c_str(),
                 input_twist_frame_id_.c_str());
  }

  uav_state_.position = {pose_msg.pose.position.x, pose_msg.pose.position.y,
                         pose_msg.pose.position.z};

  uav_state_.orientation = {pose_msg.pose.orientation.w, pose_msg.pose.orientation.x,
                            pose_msg.pose.orientation.y, pose_msg.pose.orientation.z};

  uav_state_.lin_vel = {twist_msg.twist.linear.x, twist_msg.twist.linear.y,
                        twist_msg.twist.linear.z};

  uav_state_.ang_vel = {twist_msg.twist.angular.x, twist_msg.twist.angular.y,
                        twist_msg.twist.angular.z};

  flags_.state_received = true;
};

void Plugin::updateReference(const geometry_msgs::msg::PoseStamped &pose_msg) {
  control_ref_.position = {pose_msg.pose.position.x, pose_msg.pose.position.y,
                           pose_msg.pose.position.z};

  Eigen::Vector3d eul = utils::quatToYPR(
      Eigen::Quaterniond{pose_msg.pose.orientation.w, pose_msg.pose.orientation.x,
                         pose_msg.pose.orientation.y, pose_msg.pose.orientation.z});
  control_ref_.yaw = eul[0];

  flags_.yaw_speed    = false;
  flags_.ref_received = true;
  printf("pose received\n");
};

void Plugin::updateReference(const geometry_msgs::msg::TwistStamped &twist_msg) {
  control_ref_.lin_vel   = {twist_msg.twist.linear.x, twist_msg.twist.linear.y,
                            twist_msg.twist.linear.z};
  control_ref_.yaw_speed = twist_msg.twist.angular.z;

  flags_.yaw_speed    = true;
  flags_.ref_received = true;
  printf("twist received [%f, %f, %f]\n", control_ref_.lin_vel.x(), control_ref_.lin_vel.y(),
         control_ref_.lin_vel.z());
};

void Plugin::updateReference(const as2_msgs::msg::TrajectoryPoint &traj_msg) {
  printf("traj received\n");

  assert(traj_msg.header.frame_id == enu_frame_id_);

  control_ref_.position = {traj_msg.position.x, traj_msg.position.y, traj_msg.position.z};
  control_ref_.lin_vel  = {traj_msg.twist.x, traj_msg.twist.y, traj_msg.twist.z};

  if (flags_.hover_flag) {
    control_ref_.lin_vel   = Eigen::Vector3d::Zero();
    control_ref_.yaw_speed = 0.0;
  }

  switch (control_mode_in_.yaw_mode) {
    case as2_msgs::msg::ControlMode::YAW_ANGLE:
      control_ref_.yaw       = traj_msg.yaw_angle;
      control_ref_.yaw_speed = 0.0;
      flags_.yaw_speed       = false;
      printf("Yaw angle: %.2f\n", traj_msg.yaw_angle);
      break;
    case as2_msgs::msg::ControlMode::YAW_SPEED:
      control_ref_.yaw_speed = traj_msg.yaw_angle;
      control_ref_.yaw       = 0.0;
      flags_.yaw_speed       = true;
      printf("Yaw angle speed: %.2f\n", traj_msg.yaw_angle);
      break;
  }

  flags_.ref_received = true;
  // std::cout << "Yaw " << (traj_msg.yaw_angle * 180.0 / M_PI) << std::endl;
};

bool Plugin::setMode(const as2_msgs::msg::ControlMode &in_mode,
                     const as2_msgs::msg::ControlMode &out_mode) {
  bool accepted = false;

  if ((in_mode.control_mode == as2_msgs::msg::ControlMode::TRAJECTORY ||
       in_mode.control_mode == as2_msgs::msg::ControlMode::HOVER ||
       in_mode.control_mode == as2_msgs::msg::ControlMode::POSITION ||
       in_mode.control_mode == as2_msgs::msg::ControlMode::SPEED) &&
      out_mode.control_mode == as2_msgs::msg::ControlMode::ATTITUDE) {
    control_mode_in_  = in_mode;
    control_mode_out_ = out_mode;
    accepted          = true;

    flags_.control_mode_set   = true;
    flags_.ignore_position_sp = (in_mode.control_mode == as2_msgs::msg::ControlMode::SPEED);
    flags_.ignore_vel_sp      = (in_mode.control_mode == as2_msgs::msg::ControlMode::POSITION);
    flags_.hover_flag         = control_mode_in_.control_mode == as2_msgs::msg::ControlMode::HOVER;

    if (flags_.hover_flag) {
      control_mode_in_.yaw_mode        = as2_msgs::msg::ControlMode::YAW_ANGLE;
      control_mode_in_.reference_frame = as2_msgs::msg::ControlMode::LOCAL_ENU_FRAME;
    }

    RCLCPP_INFO(node_ptr_->get_logger(), "Changed control mode to %i",
                control_mode_in_.control_mode);
  }

  return accepted;
};

void Plugin::checkParamList(const std::string &param,
                            std::vector<std::string> &_params_list,
                            bool &_all_params_read) {
  if (find(_params_list.begin(), _params_list.end(), param) != _params_list.end()) {
    // Remove the parameter from the list of parameters to be read
    _params_list.erase(std::remove(_params_list.begin(), _params_list.end(), param),
                       _params_list.end());
  };
  if (_params_list.size() == 0) {
    _all_params_read = true;
  }
};

rcl_interfaces::msg::SetParametersResult Plugin::parametersCallback(
    const std::vector<rclcpp::Parameter> &parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason     = "success";

  for (const auto &param : parameters) {
    DECLARE_PARAM(param, "astc.k2.x", controller_.k2_x);
    DECLARE_PARAM(param, "astc.k2.y", controller_.k2_y);
    DECLARE_PARAM(param, "astc.k2.z", controller_.k2_z);
    DECLARE_PARAM(param, "astc.rt.x", controller_.rt_x);
    DECLARE_PARAM(param, "astc.rt.y", controller_.rt_y);
    DECLARE_PARAM(param, "astc.rt.z", controller_.rt_z);
    DECLARE_PARAM(param, "astc.alpha.x", controller_.alpha_x);
    DECLARE_PARAM(param, "astc.alpha.y", controller_.alpha_y);
    DECLARE_PARAM(param, "astc.alpha.z", controller_.alpha_z);
    DECLARE_PARAM(param, "astc.gamma.x", controller_.gamma_x);
    DECLARE_PARAM(param, "astc.gamma.y", controller_.gamma_y);
    DECLARE_PARAM(param, "astc.gamma.z", controller_.gamma_z);
    DECLARE_PARAM(param, "astc.r0.x", controller_.r0_x);
    DECLARE_PARAM(param, "astc.r0.y", controller_.r0_y);
    DECLARE_PARAM(param, "astc.r0.z", controller_.r0_z);
    DECLARE_PARAM(param, "astc.eps.x", controller_.eps_x);
    DECLARE_PARAM(param, "astc.eps.y", controller_.eps_y);
    DECLARE_PARAM(param, "astc.eps.z", controller_.eps_z);
    DECLARE_PARAM(param, "astc.q.x", controller_.q_x);
    DECLARE_PARAM(param, "astc.q.y", controller_.q_y);
    DECLARE_PARAM(param, "astc.q.z", controller_.q_z);
    DECLARE_PARAM(param, "astc.tau.lps", controller_.tau_lpf);
    DECLARE_PARAM(param, "astc.lambda.xy", controller_.lambda_xy);
    DECLARE_PARAM(param, "astc.lambda.z", controller_.lambda_z);
    DECLARE_PARAM(param, "astc.lambda.omega", controller_.lambda_omega);
    DECLARE_PARAM(param, "astc.delta.x", controller_.delta_x);
    DECLARE_PARAM(param, "astc.delta.y", controller_.delta_y);
    DECLARE_PARAM(param, "astc.delta.z", controller_.delta_z);
    DECLARE_PARAM(param, "astc.eps.smc", controller_.eps_smc);
    DECLARE_PARAM(param, "astc.mass", controller_.mass);
  }
  return result;
}

bool Plugin::computeOutput(double dt,
                           geometry_msgs::msg::PoseStamped &pose,
                           geometry_msgs::msg::TwistStamped &twist,
                           as2_msgs::msg::Thrust &thrust) {
  if (!flags_.state_received) {
    auto &clk = *node_ptr_->get_clock();
    RCLCPP_WARN_THROTTLE(node_ptr_->get_logger(), clk, 5000, "State not received yet");
    return false;
  }

  if (!flags_.ref_received) {
    auto &clk = *node_ptr_->get_clock();
    RCLCPP_WARN_THROTTLE(node_ptr_->get_logger(), clk, 5000,
                         "State changed, but ref not received yet");
    return false;
  }

  controller_.setCurrentState(uav_state_);

  Eigen::Vector4d pos_ref = {control_ref_.position.x(), control_ref_.position.y(),
                             control_ref_.position.z(), control_ref_.yaw};

  Eigen::Vector4d twist_ref = Eigen::Vector4d{control_ref_.lin_vel.x(), control_ref_.lin_vel.y(),
                                              control_ref_.lin_vel.z(), control_ref_.yaw_speed};

  as2_msgs::msg::ControllerDebug dbg_msg;
  auto smc_out = controller_.update(dt, pos_ref, twist_ref, flags_, dbg_msg);

  debug_pub_->publish(dbg_msg);

  auto now = node_ptr_->now();

  // output
  twist.header.stamp    = now;
  twist.header.frame_id = output_twist_frame_id_;
  twist.twist.angular.x = smc_out.body_rates.x();
  twist.twist.angular.y = smc_out.body_rates.y();
  twist.twist.angular.z = smc_out.body_rates.z();

  pose.header.stamp       = now;
  pose.header.frame_id    = output_pose_frame_id_;
  pose.pose.orientation.w = smc_out.orientation.w();
  pose.pose.orientation.x = smc_out.orientation.x();
  pose.pose.orientation.y = smc_out.orientation.y();
  pose.pose.orientation.z = smc_out.orientation.z();

  thrust.header.stamp    = now;
  thrust.header.frame_id = output_pose_frame_id_;
  thrust.thrust          = smc_out.thrust;

  // printf("control mode %i, ignore pos %i, hover %i\n", control_mode_in_.control_mode,
  //        flags_.ignore_position, hover_flag_);

  // printf("ComputeOutput %.2fN\n", thrust.thrust);

  return true;
}

void AdaptiveSuperTwistingController::reset() {
  RCLCPP_INFO(logger_, "Reset");

  // reset adaptive gains to default
  rt_xyz = {rt_x, rt_y, rt_z};
  k2_xyz = {k2_x, k2_y, k2_z};

  // reset state
  a_xyz_stc     = Eigen::Vector3d::Zero();
  a_xyz_stc_dot = Eigen::Vector3d::Zero();
  a_xyz_eq      = Eigen::Vector3d::Zero();
  last_smc_out_ = ControllerOut();
}

void AdaptiveSuperTwistingController::setCurrentState(const UAV_state &state_) { state = state_; }

ControllerOut AdaptiveSuperTwistingController::update(double dt,
                                                      const Eigen::Vector4d &pos_sp,
                                                      const Eigen::Vector4d &vel_sp_,
                                                      const Control_flags &flags,
                                                      as2_msgs::msg::ControllerDebug &dbg_msg) {
  // guard against too small timesteps
  if (dt < 1e-6) {
    return last_smc_out_;
  }

  Eigen::Vector3d a_xyz_des = {0.0, 0.0, 9.81};
  Eigen::Quaterniond q_uav  = state.orientation;

  Eigen::Vector3d eul_uav = utils::quatToYPR(q_uav);
  double psi_uav          = eul_uav[0];  // yaw
  double theta_uav        = eul_uav[1];  // pitch
  double phi_uav          = eul_uav[2];  // roll

  Eigen::Vector3d xyz_uav = state.position;  // (ENU)
  Eigen::Vector3d uvw_uav = state.lin_vel;   // (ENU)

  double k1_x = 1.5 * std::sqrt(k2_x / 1.0);
  double k1_y = 1.5 * std::sqrt(k2_y / 1.0);
  double k1_z = 1.5 * std::sqrt(k2_z / 1.0);

  Eigen::Vector4d vel_sp = vel_sp_;
  if (flags.ignore_vel_sp) {
    vel_sp = Eigen::Vector4d::Zero();
  }

  Eigen::Vector3d err_xyz = xyz_uav - pos_sp.head<3>();
  Eigen::Vector3d err_uvw = uvw_uav - vel_sp.head<3>();

  if (flags.ignore_position_sp) {
    err_xyz = Eigen::Vector3d::Zero();
  }

  // printf("rt_xyz [%.2f, %.2f, %.2f], k2_xyz [%.2f, %.2f, %.2f]", rt_xyz.x(), rt_xyz.y(),
  // rt_xyz.z(),
  //        k2_xyz.x(), k2_xyz.y(), k2_xyz.z());

  // printf("a_xyz_stc [%.2f, %.2f, %.2f]\n", a_xyz_stc.x(), a_xyz_stc.y(), a_xyz_stc.z());

  // printf("\t xyz ref [%.2f, %.2f, %.2f], uvw ref [%.2f, %.2f, %.2f]\n", pos_sp.x(), pos_sp.y(),
  //        pos_sp.z(), vel_sp.x(), vel_sp.y(), vel_sp.z());

  // printf("\t rt_xyz [%.2f, %.2f, %.2f], k2_xyz [%.2f, %.2f, %.2f]\n", rt_xyz.x(), rt_xyz.y(),
  //        rt_xyz.z(), k2_xyz.x(), k2_xyz.y(), k2_xyz.z());

  // in xyz
  Eigen::Vector3d lambda    = {lambda_xy, lambda_xy, lambda_z};
  Eigen::Vector3d k1_xyz    = {k1_x, k1_y, k1_z};
  Eigen::Vector3d alpha_xyz = {alpha_x, alpha_y, alpha_z};
  Eigen::Vector3d r0_xyz    = {r0_x, r0_y, r0_z};
  Eigen::Vector3d eps_xyz   = {eps_x, eps_y, eps_z};
  Eigen::Vector3d gamma_xyz = {gamma_x, gamma_y, gamma_z};
  Eigen::Vector3d q_xyz     = {q_x, q_y, q_z};
  Eigen::Vector3d s_xyz     = err_uvw + lambda.cwiseProduct(err_xyz);  // (5)

  Eigen::Vector3d sw_fn = s_xyz.cwiseQuotient(
      s_xyz.cwiseAbs() + Eigen::Vector3d{eps_smc, eps_smc, eps_smc});  // switching function

  Eigen::Vector3d a_xyz = -lambda.cwiseProduct(err_uvw) -
                          k1_xyz.cwiseProduct(s_xyz.cwiseAbs().cwiseSqrt()).cwiseProduct(sw_fn) +
                          a_xyz_stc + a_xyz_des;  // (6a)
  a_xyz_stc_dot = -k2_xyz.cwiseProduct(sw_fn);    // (6b)
  a_xyz_eq      = (1.0 - dt / tau_lpf) * a_xyz_eq - (dt / tau_lpf) * a_xyz_stc_dot;
  Eigen::Vector3d deltat_xyz =
      k2_xyz - a_xyz_eq.cwiseAbs().cwiseQuotient(alpha_xyz) - eps_xyz;  // (9a)
  Eigen::Vector3d e_xyz = q_xyz.cwiseQuotient(alpha_xyz) - rt_xyz;      // (9b)
  Eigen::Vector3d rt_xyz_dot =
      gamma_xyz.cwiseProduct(deltat_xyz.cwiseAbs()) +
      r0_xyz.cwiseProduct(gamma_xyz.cwiseSqrt()).cwiseProduct(e_xyz.cwiseSign());  // (8b)

  // printf("dt=%.2f err xyz [%.2f, %.2f, %.2f], uvw [%.2f, %.2f, %.2f] axyz [%.2f, %.2f, %.2f]\n",
  // dt,
  //        err_xyz.x(), err_xyz.y(), err_xyz.z(), err_uvw.x(), err_uvw.y(), err_uvw.z(), a_xyz.x(),
  //        a_xyz.y(), a_xyz.z());

  rt_xyz += dt * rt_xyz_dot;
  k2_xyz += dt * -(r0_xyz + rt_xyz).cwiseProduct(deltat_xyz.cwiseSign());
  a_xyz_stc += dt * a_xyz_stc_dot;

  k2_xyz = k2_xyz.cwiseMax(0.0).cwiseMin(2.0);
  rt_xyz = rt_xyz.cwiseMax(0.0).cwiseMin(2.0);

  // printf("k2_xyz [%.2f, %.2f, %.2f]\n", k2_xyz.x(), k2_xyz.y(), k2_xyz.z());
  // printf("a_xyz_stc_dot [%.2f, %.2f, %.2f], a_xyz_eq [%.4f, %.4f, %.4f]\n",
  // a_xyz_stc_dot.x(),
  //        a_xyz_stc_dot.y(), a_xyz_stc_dot.z(), a_xyz_eq.x(), a_xyz_eq.y(), a_xyz_eq.z());

  // printf("rt x: %.2f, k2 x %.2f\n", rt_xyz.x(), k2_xyz.x());
  // printf("rt y: %.2f, k2 y %.2f\n", rt_xyz.y(), k2_xyz.y());
  // printf("rt z: %.2f, k2 z %.2f\n", rt_xyz.z(), k2_xyz.z());

  double f = std::max(a_xyz.norm(), 1e-6);  // avoid division by 0

  // in psi
  double psi_des   = flags.yaw_speed ? psi_uav + dt * vel_sp.w() : pos_sp.w();
  double ax2       = a_xyz.x() * std::cos(psi_uav) + a_xyz.y() * std::sin(psi_uav);
  double ay2       = -a_xyz.x() * std::sin(psi_uav) + a_xyz.y() * std::cos(psi_uav);
  double phi_des   = std::asin(-ay2 / f);
  double theta_des = std::asin(ax2 / (f * std::cos(phi_uav)));

  // printf("phi_des %.2f, theta_des %.2f, psi_des %.2f\n", phi_des * 180 / 3.1415,
  //        theta_des * 180 / 3.1415, psi_des * 180 / 3.1415);

  phi_des   = std::clamp(phi_des * lambda_omega, -0.8, 0.8);
  theta_des = std::clamp(theta_des * lambda_omega, -0.8, 0.8);

  double delta_psi = flags.yaw_speed ? vel_sp.w() : psi_uav - psi_des;

  // wrap between -pi and pi
  if (delta_psi > M_PI) {
    delta_psi -= M_2_PI * std::ceil(std::abs(delta_psi) / M_2_PI);
  } else if (delta_psi < -M_PI) {
    delta_psi += M_2_PI * std::ceil(std::abs(delta_psi) / M_2_PI);
  }

  Eigen::Vector3d e_eul =
      -lambda_omega * Eigen::Vector3d{phi_uav - phi_des, theta_uav - theta_des, delta_psi};
  Eigen::Vector3d omega_des = utils::eulerRatesToBodyRates(eul_uav, e_eul);  // body rates

  // printf("e_eul_xyz [%.2f, %.2f, %.2f]\n", e_eul.x(), e_eul.y(), e_eul.z());

  // desired orientation (quaternion)
  Eigen::Quaterniond q_des = utils::eulerToQuat({psi_des, theta_des, phi_des});

  double az2    = (q_uav.toRotationMatrix().transpose() * a_xyz).z();
  double thrust = az2 * mass;

  auto smc_out = ControllerOut{.body_rates  = {omega_des.x(), omega_des.y(), omega_des.z()},
                               .orientation = q_des,
                               .thrust      = thrust};

  last_smc_out_ = smc_out;

  dbg_msg.pos_err.x = err_xyz.x();
  dbg_msg.pos_err.y = err_xyz.y();
  dbg_msg.pos_err.z = err_xyz.z();

  dbg_msg.twist_err.x = err_uvw.x();
  dbg_msg.twist_err.y = err_uvw.y();
  dbg_msg.twist_err.z = err_uvw.z();

  dbg_msg.a_xyz.x = ax2;
  dbg_msg.a_xyz.y = ay2;
  dbg_msg.a_xyz.z = a_xyz.z();

  dbg_msg.sw_fn.x = sw_fn.x();
  dbg_msg.sw_fn.y = sw_fn.y();
  dbg_msg.sw_fn.z = sw_fn.z();

  dbg_msg.pos_ref.x = pos_sp.x();
  dbg_msg.pos_ref.y = pos_sp.y();
  dbg_msg.pos_ref.z = pos_sp.z();

  dbg_msg.vel_ref.x = vel_sp.x();
  dbg_msg.vel_ref.y = vel_sp.y();
  dbg_msg.vel_ref.z = vel_sp.z();

  return smc_out;
}

}  // namespace astc_controller

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(astc_controller::Plugin, as2_motion_controller_plugin_base::ControllerBase)
