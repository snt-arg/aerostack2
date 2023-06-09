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

#ifndef __SP_PLUGIN_H__
#define __SP_PLUGIN_H__

#include <chrono>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "as2_core/utils/control_mode_utils.hpp"
#include "as2_core/utils/frame_utils.hpp"
#include "as2_core/utils/tf_utils.hpp"
#include "as2_motion_controller/controller_base.hpp"
#include "as2_msgs/msg/thrust.hpp"
#include "as2_msgs/msg/trajectory_point.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include "as2_msgs/msg/controller_debug.hpp"

namespace astc_controller {

struct Control_flags {
  bool state_received     = false;
  bool ref_received       = false;
  bool ignore_position_sp = false;
  bool ignore_vel_sp      = false;
  bool control_mode_set   = false;
  bool hover_flag         = false;
  bool yaw_speed          = false;
};

struct UAV_state {
  Eigen::Vector3d position       = Eigen::Vector3d::Zero();
  Eigen::Vector3d lin_vel        = Eigen::Vector3d::Zero();
  Eigen::Vector3d ang_vel        = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
};

struct UAV_controlRef {
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d lin_vel  = Eigen::Vector3d::Zero();
  double yaw               = 0.0;
  double yaw_speed         = 0.0;
};

struct ControllerOut {
  Eigen::Vector3d body_rates     = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
  double thrust                  = 0.0;
};

/// @brief Adaptive Super-Twisting Control (ASTC) based on:
///
/// D. M. K. K. V. Rao, H. Habibi, J. L. Sanchez-Lopez, P. P. Menon, C. Edwards, and H. Voos,
/// “Adaptive Super-Twisting Controller Design for Accurate Trajectory Tracking Performance of
/// Unmanned Aerial Vehicles,” arXiv Prepr., pp. 1–10, Mar. 2023,
/// Available: http://arxiv.org/abs/2303.11770.
///
class AdaptiveSuperTwistingController {
private:
  // logger
  rclcpp::Logger logger_;

public:
  AdaptiveSuperTwistingController(const rclcpp::Logger &logger) : logger_(logger) {}
  void reset();
  void setParamsFromRos();
  void setCurrentState(const UAV_state &state);
  ControllerOut update(double dt,
                       const Eigen::Vector4d &pos_sp,
                       const Eigen::Vector4d &vel_sp,
                       const Control_flags &flags,
                       as2_msgs::msg::ControllerDebug &dbg_msg);

  // controller state
  Eigen::Vector3d a_xyz_stc = Eigen::Vector3d::Zero();
  Eigen::Vector3d a_xyz_eq  = Eigen::Vector3d::Zero();

  ControllerOut last_smc_out_;

  // adaptive gains
  Eigen::Vector3d rt_xyz = {rt_x, rt_y, rt_z};
  Eigen::Vector3d k2_xyz = {k2_x, k2_y, k2_z};

  // uav state
  UAV_state state;

  // params/gains
  // design parameters should be chosen according to the criteria stated in Remark 3.
  // parameters worth tuning are lambda_xy|z, gamma_x|y|z, lpf
  // tune q_x|y|z, alpha for better disturbance rejection
  // initial adaptive gains (k2_x|y|z and rt_x|y|z) can be set to zero
  double k2_x    = 0.0;
  double rt_x    = 0.0;
  double alpha_x = 0.9;  // between (0;1), disturbance
  double eps_x   = 0.001;
  double gamma_x = 2.0;  // convergence rate
  double r0_x    = 0.0001;
  double q_x     = 0.04;  // disturbance

  double k2_y    = 0.0;
  double rt_y    = 0.0;
  double alpha_y = 0.9;
  double eps_y   = 0.001;
  double gamma_y = 2.0;
  double r0_y    = 0.0001;
  double q_y     = 0.04;

  double k2_z    = 0.0;
  double rt_z    = 0.0;
  double alpha_z = 1.2;
  double eps_z   = 0.01;
  double gamma_z = 2.0;
  double r0_z    = 0.0001;
  double q_z     = 0.01;

  double tau_lpf      = 0.03;  // chattering
  double lambda_xy    = 2.25;  // tracking, may cause chattering
  double lambda_z     = 2.0;
  double lambda_omega = 2.0;
  double eps_smc      = 0.1;  // chattering
  double mass         = 1.5;
};

class Plugin : public as2_motion_controller_plugin_base::ControllerBase {
public:
  Plugin(){};
  ~Plugin(){};

public:
  void ownInitialize() override;
  void updateState(const geometry_msgs::msg::PoseStamped &pose_msg,
                   const geometry_msgs::msg::TwistStamped &twist_msg) override;

  void updateReference(const geometry_msgs::msg::PoseStamped &ref) override;
  void updateReference(const geometry_msgs::msg::TwistStamped &ref) override;
  void updateReference(const as2_msgs::msg::TrajectoryPoint &ref) override;

  bool updateParams(const std::vector<rclcpp::Parameter> &_params_list) override;
  rcl_interfaces::msg::SetParametersResult parametersCallback(
      const std::vector<rclcpp::Parameter> &parameters);

  bool setMode(const as2_msgs::msg::ControlMode &mode_in,
               const as2_msgs::msg::ControlMode &mode_out) override;

  // IMPORTANT: this is the frame_id of the desired pose and twist
  std::string getDesiredPoseFrameId() override { return input_pose_frame_id_; }
  std::string getDesiredTwistFrameId() override { return input_twist_frame_id_; }

  bool computeOutput(double dt,
                     geometry_msgs::msg::PoseStamped &pose,
                     geometry_msgs::msg::TwistStamped &twist,
                     as2_msgs::msg::Thrust &thrust) override;

  void reset() override;

private:
  as2_msgs::msg::ControlMode control_mode_in_;
  as2_msgs::msg::ControlMode control_mode_out_;

  std::unique_ptr<AdaptiveSuperTwistingController> controller_;

  Control_flags flags_;

  std::shared_ptr<as2::tf::TfHandler> tf_handler_;

  UAV_state uav_state_;
  UAV_controlRef control_ref_;

  std::string enu_frame_id_ = "odom";
  std::string flu_frame_id_ = "base_link";

  std::string input_pose_frame_id_  = enu_frame_id_;
  std::string input_twist_frame_id_ = enu_frame_id_;

  std::string output_twist_frame_id_ = enu_frame_id_;
  std::string output_pose_frame_id_  = enu_frame_id_;

  rclcpp::Publisher<as2_msgs::msg::ControllerDebug>::SharedPtr debug_pub_;

private:
  void checkParamList(const std::string &param,
                      std::vector<std::string> &_params_list,
                      bool &_all_params_read);

  void resetState();
  void resetReferences();
  void resetCommands();
};

};  // namespace astc_controller

#endif
