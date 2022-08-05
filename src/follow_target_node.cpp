// "Copyright [year] <Copyright Owner>"

#include "as2_core/core_functions.hpp"
#include "follow_target.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<follow_target::FollowTarget>();
  node->preset_loop_frequency(100); // Node frequency for run and callbacks
  // Node with only callbacks
  as2::spinLoop(node);
  // Node with run
  // as2::spinLoop(node, std::bind(&follow_target::FollowTarget::run, node));

  rclcpp::shutdown();
  return 0;
}
