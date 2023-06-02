/*!*******************************************************************************************
 *  \file       utils.hpp
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

#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <eigen3/unsupported/Eigen/EulerAngles>

namespace utils {

Eigen::Matrix3d matEulRatesToBodyRates(double phi, double theta) {
  Eigen::Matrix3d M;
  // clang-format off
  M <<  1.0, 0.0,           -std::sin(theta), 
        0.0, std::cos(phi),  std::sin(phi) * std::cos(theta), 
        0.0, -std::sin(phi), std::cos(phi) * std::cos(theta);
  // clang-format on

  return M;  // [p, q, r]
}

Eigen::Matrix3d matBodyRatesToEulerRates(double phi, double theta) {
  Eigen::Matrix3d M;
  // clang-format off
  M <<  1.0, std::sin(phi) * std::tan(theta),  std::cos(phi)*std::tan(theta), 
        0.0, std::cos(phi),                   -std::sin(phi), 
        0.0, std::sin(phi) / std::cos(theta),  std::cos(phi) / std::cos(theta);
  // clang-format on

  return M;  // [dphi, dtheta, dpsi]
}

/// @brief converts a quaternion to euler angles
/// @param q the quaternion
/// @return euler angles in yaw, pitch, roll order (resp. psi, theta, phi)
Eigen::Vector3d quatToYPR(const Eigen::Quaterniond& q) {
  Eigen::Vector3d ypr = Eigen::EulerAnglesZYXd(q.toRotationMatrix()).angles();
  return ypr;  // order: yaw, pitch, roll (resp. psi, theta, phi)
}

/// @brief converts euler angles to a quaternion
/// @param ypr euler angles in yaw, pitch, roll order (resp. psi, theta, phi)
/// @return a quaternion
Eigen::Quaterniond eulerToQuat(const Eigen::Vector3d& ypr) {
  Eigen::Quaterniond q = Eigen::Quaterniond(
      Eigen::EulerAnglesZYXd(ypr[0], ypr[1], ypr[2]).toRotationMatrix());  // yaw, pitch, roll

  return q;
}

/// @brief converts body rates to euler rates
/// @param ypr euler angles in yaw, pitch roll order
/// @param pqr body rates
/// @return euler rates in dphi, dtheta, dpsi order
Eigen::Vector3d bodyRatesToEulerRates(const Eigen::Vector3d& ypr, const Eigen::Vector3d& pqr) {
  return matBodyRatesToEulerRates(ypr[2], ypr[1]) * pqr;  // [dphi, dtheta, dpsi]
}

/// @brief converts euler rates to body rates
/// @param ypr euler angles in yaw, pitch roll order
/// @param euler_rates in dphi, dtheta, dpsi order
/// @return body rates pqr
Eigen::Vector3d eulerRatesToBodyRates(const Eigen::Vector3d& ypr,
                                      const Eigen::Vector3d& euler_rates) {
  return matEulRatesToBodyRates(ypr[2], ypr[1]) * euler_rates;  // [p, q, r]
}

}  // namespace utils