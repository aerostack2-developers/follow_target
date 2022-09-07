#include "ft_dynamic_follow.hpp"

namespace ft_dynamic_follow
{
DynamicFollow::DynamicFollow(ft_base::FTBaseStruct tf_base_struct) : ft_base::FollowTargetBase(tf_base_struct)
{
    return;
};

void DynamicFollow::ownDeclareParameters()
{
    for (int i = 0; i < dynamic_follow_parameters.size(); i++)
    {
        // node_ptr_->declare_parameter(pickup_parameters[i]); // TODO: WARNING on galactic and advance
        ft_utils::declareParameters(node_ptr_, dynamic_follow_parameters[i]);
    }
    return;
};

void DynamicFollow::ownUpdateParam(rclcpp::Parameter param)
{
    RCLCPP_INFO(node_ptr_->get_logger(), "DynamicFollow-UpdateParam: %s", param.get_name().c_str());
    if (param.get_name() == "dynamicFollow.height")
        height_ = param.get_value<float>();
    else if (param.get_name() == "dynamicFollow.distance")
        distance_ = param.get_value<float>();
    else if (param.get_name() == "dynamicFollow.const_height")
        const_height_ = param.get_value<bool>();
    return;
};

void DynamicFollow::ownResetState()
{
    return;
};

void DynamicFollow::ownRun(const double &dt)
{
    Eigen::Vector3d speed_limit = *speed_limit_.get();
    bool proportional_speed_limit = proportional_limitation_;
    double desired_yaw = 0.0f;
    double yaw_speed = 0.0f;

    double distance2d = ft_utils::computeDistance2D(sl_pose_->pose.position.x, sl_pose_->pose.position.y,
                                                    target_pose_->pose.position.x, target_pose_->pose.position.y);
    if (distance2d > 1.0f)
    {
        desired_yaw = getPathFacingAngle();
        yaw_speed = computeYawControl(dt, desired_yaw);
    }
    // else if (Eigen::Vector2d(target_twist_.linear.x, target_twist_.linear.y).norm() > 0.2f)
    // {
    //     desired_yaw = as2::FrameUtils::getVector2DAngle(target_twist_.linear.x, target_twist_.linear.y);
    //     yaw_speed = computeYawControl(dt, desired_yaw);
    // }
    else
    {
        desired_yaw = as2::FrameUtils::getYawFromQuaternion(sl_pose_->pose.orientation);
    }

    reference_pose_.position.x = target_pose_->pose.position.x - distance_ * cos(desired_yaw);
    reference_pose_.position.y = target_pose_->pose.position.y - distance_ * sin(desired_yaw);
    if (const_height_)
    {
        reference_pose_.position.z = target_pose_->pose.position.z + height_;
    }
    else
    {
        reference_pose_.position.z = height_;
    }

    RCLCPP_INFO(node_ptr_->get_logger(), "DynamicFollow to: %f, %f, %f", reference_pose_.position.x,
                reference_pose_.position.y, reference_pose_.position.z);

    Eigen::Vector3d motion_speed_ = computeControl(dt, speed_limit, proportional_speed_limit);
    motion_handler_speed_->sendSpeedCommandWithYawSpeed(motion_speed_.x(), motion_speed_.y(), motion_speed_.z(),
                                                        yaw_speed);
    return;
};
} // namespace ft_dynamic_follow