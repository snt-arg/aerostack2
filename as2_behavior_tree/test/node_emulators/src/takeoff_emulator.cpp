
#include "node_emulators/takeoff_emulator.hpp"
#include "as2_core/core_functions.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<TakeOffBehaviorEmulator>();
  node->preset_loop_frequency(30);
  as2::spinLoop(node);

  rclcpp::shutdown();
  return 0;
}