#ifndef FT_BASE_HPP_
#define FT_BASE_HPP_

#include <rclcpp/rclcpp.hpp>
#include "as2_core/node.hpp"
#include "motion_reference_handlers/speed_motion.hpp"
#include "motion_reference_handlers/hover_motion.hpp"

#include "ft_speed_controller.hpp"

#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <Eigen/Dense>

namespace ft_base
{
    struct FTBaseStruct {
        as2::Node* node_ptr;
        as2::motionReferenceHandlers::SpeedMotion* motion_handler_speed;
        as2::motionReferenceHandlers::HoverMotion* motion_handler_hover;
        ft_speed_controller::SpeedController* controller_handler;
        geometry_msgs::msg::PoseStamped* sl_pose;
        geometry_msgs::msg::TwistStamped* sl_twist;
        geometry_msgs::msg::PoseStamped* target_pose;
        Eigen::Vector3d* speed_limit;
    };

    class FollowTargetBase
    {
    public:
        FollowTargetBase(FTBaseStruct tf_base_struct);
        ~FollowTargetBase(){};

    protected:
        as2::Node* node_ptr_;
        geometry_msgs::msg::PoseStamped ref_pose_;
        geometry_msgs::msg::TwistStamped ref_twist_;
    };
}; // namespace ft_base

#endif // FT_BASE_HPP_
