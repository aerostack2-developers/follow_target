#include "ft_dynamic_follow.hpp"

namespace ft_dynamic_follow
{
DynamicFollow::DynamicFollow(ft_base::FTBaseStruct tf_base_struct) : ft_base::FollowTargetBase(tf_base_struct)
{
    return;
};

void DynamicFollow::ownDeclareParameters()
{
    // for (int i = 0; i < pickup_parameters.size(); i++)
    // {
    //     // node_ptr_->declare_parameter(pickup_parameters[i]); // TODO: WARNING on galactic and advance
    //     ft_utils::declareParameters(node_ptr_, dynamic_follow_parameters[i]);
    // }
    return;
};

void DynamicFollow::ownUpdateParam(rclcpp::Parameter param)
{
    return;
};

void DynamicFollow::ownResetState()
{
    current_phase_ = 0;
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
    if (distance2d > 3.0)
    {
        desired_yaw = getPathFacingAngle();
        yaw_speed = computeYawControl(dt, desired_yaw);
    }
    else
    {
        desired_yaw = as2::FrameUtils::getYawFromQuaternion(sl_pose_->pose.orientation);
    }

    // Desired reference_pose_ is X distance from target_pose_ with desired_yaw angle
    double distance = 0.5f;
    double desired_z = 15.0f;

    reference_pose_.position.x = target_pose_->pose.position.x - distance * cos(desired_yaw);
    reference_pose_.position.y = target_pose_->pose.position.y - distance * sin(desired_yaw);
    reference_pose_.position.z = desired_z;

    Eigen::Vector3d motion_speed_ = computeControl(dt, speed_limit, proportional_speed_limit);
    motion_handler_speed_->sendSpeedCommandWithYawSpeed(motion_speed_.x(), motion_speed_.y(), motion_speed_.z(),
                                                        yaw_speed);
    return;
};
} // namespace ft_dynamic_follow