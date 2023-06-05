/*!*******************************************************************************************
 *  \file       ug_grasp_approach_behavior.hpp
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

#pragma once

#include <rclcpp/rclcpp.hpp>
#include "as2_behavior/behavior_server.hpp"
#include "as2_core/names/topics.hpp"
#include "as2_core/node.hpp"
#include "as2_core/sensor.hpp"
#include "as2_core/utils/frame_utils.hpp"
#include "as2_core/utils/tf_utils.hpp"
#include "as2_motion_reference_handlers/speed_motion.hpp"
#include "as2_motion_reference_handlers/trajectory_motion.hpp"
#include "as2_msgs/action/ug_grasp_approach.hpp"
#include "as2_msgs/msg/ug_mpc_feedback.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "ug_mpc/ug_mpc.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Dense>
#include <image_transport/image_transport.hpp>
#include <memory>
#include <vector>

class UgGraspApproachBehavior
    : public as2_behavior::BehaviorServer<as2_msgs::action::UgGraspApproach> {
public:
  /**
   * @brief Construct a new Aruco Detector object
   */
  UgGraspApproachBehavior();

  /**
   * @brief Destroy the Aruco Detector object
   */
  ~UgGraspApproachBehavior(){};

private:
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<as2::tf::TfHandler> tf_handler_;
  std::shared_ptr<as2::motionReferenceHandlers::SpeedMotion> speed_motion_handler_;
  std::shared_ptr<as2::motionReferenceHandlers::TrajectoryMotion> trajectory_motion_handler_;
  rclcpp::Publisher<as2_msgs::msg::UgMpcFeedback>::SharedPtr mpc_feedback_pub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;

  std::string base_link_frame_id_ = "base_link";
  std::string odom_frame_id_      = "odom";

  Mpc mpc_;

  Eigen::Vector3d pos_goal_;
  Eigen::Vector3d pos_poi_;

  geometry_msgs::msg::PoseStamped pose_uav_;
  geometry_msgs::msg::TwistStamped twist_uav_;

  void updateMpcState();

  void loadParameters();
  void setCameraParameters(const sensor_msgs::msg::CameraInfo &_camera_info);
  bool checkIdIsTarget(const int _id);

private:
  /** As2 Behavior methods **/
  bool on_activate(std::shared_ptr<const as2_msgs::action::UgGraspApproach::Goal> goal) override;

  bool on_modify(std::shared_ptr<const as2_msgs::action::UgGraspApproach::Goal> goal) override;

  bool on_deactivate(const std::shared_ptr<std::string> &message) override;

  bool on_pause(const std::shared_ptr<std::string> &message) override;

  bool on_resume(const std::shared_ptr<std::string> &message) override;

  as2_behavior::ExecutionStatus on_run(
      const std::shared_ptr<const as2_msgs::action::UgGraspApproach::Goal> &goal,
      std::shared_ptr<as2_msgs::action::UgGraspApproach::Feedback> &feedback_msg,
      std::shared_ptr<as2_msgs::action::UgGraspApproach::Result> &result_msg) override;

  void on_execution_end(const as2_behavior::ExecutionStatus &state) override;

  void state_callback(const geometry_msgs::msg::TwistStamped::SharedPtr _twist_msg);
};
