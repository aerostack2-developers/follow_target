#ifndef FT_DYNAMIC_LAND_HPP_
#define FT_DYNAMIC_LAND_HPP_

#include "as2_core/names/topics.hpp"
#include <as2_core/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include "ft_base.hpp"
#include "ft_speed_controller.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/bool.hpp>

namespace ft_dynamic_land
{
using SpeedController = ft_speed_controller::SpeedController;

class DynamicLand : ft_base::FollowTargetBase
{
  public:
    DynamicLand(as2::Node *_node_ptr);
    ~DynamicLand(){};

    ownrun(const double dt){};
};
}; // namespace ft_dynamic_land

#endif // FT_DYNAMIC_LAND_HPP_
