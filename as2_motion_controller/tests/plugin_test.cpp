#include "gtest/gtest.h"

#include <eigen3/Eigen/Dense>
#include <eigen3/unsupported/Eigen/EulerAngles>

// colcon build --symlink-install && colcon test --packages-select as2_motion_controller
// --event-handlers console_direct+

TEST(as2_motion_controller_plugin_test, first_test) {
  Eigen::Quaterniond q_uav = {0.9753, -0.1550, 0.0266, 0.1550};
  // Eigen::Vector3d eul_uav  = Eigen::EulerAnglesZXYd(q_uav.toRotationMatrix()).angles();
  // Eigen::Vector3d eul_uav = Eigen::EulerAnglesXYZd(q_uav.toRotationMatrix()).angles();
  Eigen::Vector3d eul_uav = q_uav.toRotationMatrix().eulerAngles(2, 1, 0);
  double psi_uav          = eul_uav[0];  // yaw
  double theta_uav        = eul_uav[1];  // pitch
  double phi_uav          = eul_uav[2];  // roll

  // euler angles should be [0.3001    0.1001   -0.3001]
  printf("quat [%.3f, %.3f, %.3f, %.3f], eul [%.3f, %.3f, %.3f]\n", q_uav.w(), q_uav.x(), q_uav.y(),
         q_uav.z(), eul_uav.x(), eul_uav.y(), eul_uav.z());

  ASSERT_NEAR(eul_uav[0], 0.3001, 0.001);
  ASSERT_NEAR(eul_uav[1], 0.1001, 0.001);
  ASSERT_NEAR(eul_uav[2], -0.3001, 0.001);
}

TEST(as2_motion_controller_plugin_test, second_test) {
  Eigen::Quaterniond q_uav = {1.0000, 0.0030, 0.0020, -0.0020};
  Eigen::Vector3d eul_uav  = Eigen::EulerAnglesZYXd(q_uav.toRotationMatrix()).angles();
  // Eigen::Vector3d eul_uav = Eigen::EulerAnglesXYZd(q_uav.toRotationMatrix()).angles();
  // Eigen::Vector3d eul_uav = q_uav.toRotationMatrix().eulerAngles(2, 1, 0);

  double psi_uav   = eul_uav[0];  // yaw
  double theta_uav = eul_uav[1];  // pitch
  double phi_uav   = eul_uav[2];  // roll

  // if (eul_uav[0] > M_PI_2) eul_uav[0] -= M_PI_2;
  // if (eul_uav[0] < -M_PI_2) eul_uav[0] += M_PI_2;

  // if (eul_uav[1] > M_PI_2) eul_uav[1] -= M_PI;
  // if (eul_uav[1] < -M_PI_2) eul_uav[1] += M_PI;

  // if (eul_uav[2] > M_PI_2) eul_uav[2] -= M_PI;
  // if (eul_uav[2] < -M_PI_2) eul_uav[2] += M_PI;

  // euler angles should be [-0.0040    0.0040    0.0060]
  printf("quat [%.3f, %.3f, %.3f, %.3f], eul [%.3f, %.3f, %.3f]\n", q_uav.w(), q_uav.x(), q_uav.y(),
         q_uav.z(), eul_uav.x(), eul_uav.y(), eul_uav.z());

  ASSERT_NEAR(eul_uav[0], -0.0040, 0.001);
  ASSERT_NEAR(eul_uav[1], 0.0040, 0.001);
  ASSERT_NEAR(eul_uav[2], 0.0060, 0.001);
}

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
