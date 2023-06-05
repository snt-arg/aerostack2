/*!*******************************************************************************************
 *  \file       ug_grasp_approach_behavior.cpp
 *  \brief
 *  \authors    Paul Kremer
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

#include "ug_grasp_approach_behavior.hpp"

#include <chrono>

using namespace std::chrono_literals;

UgGraspApproachBehavior::UgGraspApproachBehavior()
    : as2_behavior::BehaviorServer<as2_msgs::action::UgGraspApproach>(
          "ug_grasp_approach_behavior") {
  loadParameters();

  // tf handler
  base_link_frame_id_ = as2::tf::generateTfName(this, "base_link");
  odom_frame_id_      = as2::tf::generateTfName(this, "odom");
  tf_handler_         = std::make_shared<as2::tf::TfHandler>(this);

  // motion handlers
  speed_motion_handler_ = std::make_shared<as2::motionReferenceHandlers::SpeedMotion>(this);
  trajectory_motion_handler_ =
      std::make_shared<as2::motionReferenceHandlers::TrajectoryMotion>(this);

  // pubs
  mpc_feedback_pub_ = this->create_publisher<as2_msgs::msg::UgMpcFeedback>(
      this->generate_local_name("ug_mpc_feedback"), rclcpp::SensorDataQoS());

  // subs
  twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      as2_names::topics::self_localization::twist, as2_names::topics::self_localization::qos,
      std::bind(&UgGraspApproachBehavior::state_callback, this, std::placeholders::_1));

  // timer
  RCLCPP_INFO(get_logger(), "Setup done");
}

void UgGraspApproachBehavior::loadParameters() {
  // declare parameters etc
}

//** AS2 Behavior methods **//
bool UgGraspApproachBehavior::on_activate(
    std::shared_ptr<const as2_msgs::action::UgGraspApproach::Goal> goal) {
  pos_goal_ = Eigen::Vector3d{goal->pos_goal.x, goal->pos_goal.y, goal->pos_goal.z};
  pos_poi_  = Eigen::Vector3d{goal->pos_poi.x, goal->pos_poi.y, goal->pos_poi.z};

  RCLCPP_INFO(get_logger(), "Goal accepted");
  return true;
}

bool UgGraspApproachBehavior::on_modify(
    std::shared_ptr<const as2_msgs::action::UgGraspApproach::Goal> goal) {
  pos_goal_ = Eigen::Vector3d{goal->pos_goal.x, goal->pos_goal.y, goal->pos_goal.z};
  pos_poi_  = Eigen::Vector3d{goal->pos_poi.x, goal->pos_poi.y, goal->pos_poi.z};

  RCLCPP_INFO(get_logger(), "Goal modified");
  return true;
}

bool UgGraspApproachBehavior::on_deactivate(const std::shared_ptr<std::string> &message) {
  RCLCPP_INFO(get_logger(), "UgGraspApproachBehavior cancelled");
  return true;
}

bool UgGraspApproachBehavior::on_pause(const std::shared_ptr<std::string> &message) {
  RCLCPP_INFO(get_logger(), "UgGraspApproachBehavior paused");
  return true;
}

bool UgGraspApproachBehavior::on_resume(const std::shared_ptr<std::string> &message) {
  RCLCPP_INFO(get_logger(), "UgGraspApproachBehavior resumed");
  return true;
}

as2_behavior::ExecutionStatus UgGraspApproachBehavior::on_run(
    const std::shared_ptr<const as2_msgs::action::UgGraspApproach::Goal> &goal,
    std::shared_ptr<as2_msgs::action::UgGraspApproach::Feedback> &feedback_msg,
    std::shared_ptr<as2_msgs::action::UgGraspApproach::Result> &result_msg) {
  RCLCPP_INFO(get_logger(), "Loop...");

  mpc_.setBodyRadius(0.5);
  // mpc_.setRefPos(pos_goal_, 0.0);
  // mpc_.dumpParams();

  auto mpc_status = mpc_.solve();

  RCLCPP_INFO(get_logger(), "MPC solved");
  RCLCPP_INFO(get_logger(), "solve time %f, success %i, status %i", mpc_status.solve_time,
              mpc_status.success, mpc_status.status);

  if (mpc_status.success) {
    RCLCPP_INFO(get_logger(), "Success...");
    Eigen::VectorXd u = mpc_.inputForStage(0);

    speed_motion_handler_->sendSpeedCommandWithYawSpeed("earth", u[0], u[1], u[2], u[3]);
  }

  return as2_behavior::ExecutionStatus::RUNNING;
}

void UgGraspApproachBehavior::on_execution_end(const as2_behavior::ExecutionStatus &status) {
  RCLCPP_INFO(get_logger(), "UgGraspApproachBehavior execution ended");
  return;
}

void UgGraspApproachBehavior::state_callback(
    const geometry_msgs::msg::TwistStamped::SharedPtr _twist_msg) {
  try {
    auto [pose_msg, twist_msg] =
        tf_handler_->getState(*_twist_msg, "earth", "earth", base_link_frame_id_);

    pose_uav_  = pose_msg;
    twist_uav_ = twist_msg;

    updateMpcState();

    // printf("MPC state updated pos [%.2f, %.2f, %.2f] twist [%.2f, %.2f, %.2f]\n",
    //        pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z,
    //        twist_uav_.twist.linear.x, twist_uav_.twist.linear.y, twist_uav_.twist.linear.z);

  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
  }
  return;
}

void UgGraspApproachBehavior::updateMpcState() { mpc_.setCurrentState(pose_uav_, twist_uav_); }
