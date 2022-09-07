#include "ft_base.hpp"

namespace ft_base
{
FollowTargetBase::FollowTargetBase(FTBaseStruct tf_base_struct)
{
    node_ptr_ = tf_base_struct.node_ptr;
    motion_handler_speed_ = tf_base_struct.motion_handler_speed;
    motion_handler_hover_ = tf_base_struct.motion_handler_hover;
    controller_handler_ = tf_base_struct.controller_handler;
    sl_pose_ = tf_base_struct.sl_pose;
    sl_twist_ = tf_base_struct.sl_twist;
    target_pose_ = tf_base_struct.target_pose;
    speed_limit_ = tf_base_struct.speed_limit;
    return;
};

void FollowTargetBase::declareParameters()
{
    for (int i = 0; i < base_parameters.size(); i++)
    {
        // node_ptr_->declare_parameter(base_parameters[i]); // TODO: WARNING on galactic and advance
        ft_utils::declareParameters(node_ptr_, base_parameters[i]);
    }
    ownDeclareParameters();
    return;
};

void FollowTargetBase::updateParam(const rclcpp::Parameter param)
{
    // RCLCPP_INFO(node_ptr_->get_logger(), "Base-UpdateParam: %s", param.get_name().c_str());
    if (param.get_name() == "base.proportional_limitation")
        proportional_limitation_ = param.get_value<bool>();
    else if (param.get_name() == "base.target_twist_alpha")
        target_twist_alpha_ = param.get_value<float>();
    else if (param.get_name() == "base.target_pose_predict_factor")
        target_pose_predict_factor_ = param.get_value<float>();
    else if (param.get_name() == "base.target_height_alpha")
        target_height_alpha_ = param.get_value<float>();

    ownUpdateParam(param);
    return;
};

void FollowTargetBase::resetState()
{
    ownResetState();
    first_run = true;
    return;
};

void FollowTargetBase::computeTargetSpeed()
{
    // Linear velocity
    rclcpp::Duration duration(target_pose_->header.stamp.sec, target_pose_->header.stamp.nanosec);
    rclcpp::Duration duration_last(last_target_pose_.header.stamp.sec, last_target_pose_.header.stamp.nanosec);
    double dt = (duration - duration_last).seconds();
    if (dt <= 0.0)
    {
        return;
    }

    geometry_msgs::msg::Pose delta_pose;

    delta_pose.position.x = target_pose_->pose.position.x - last_target_pose_.pose.position.x;
    delta_pose.position.y = target_pose_->pose.position.y - last_target_pose_.pose.position.y;
    delta_pose.position.z = target_pose_->pose.position.z - last_target_pose_.pose.position.z;

    if (first_run)
    {
        last_vx = delta_pose.position.x / dt;
        last_vy = delta_pose.position.y / dt;
        last_vz = delta_pose.position.z / dt;
    }

    target_twist_.linear.x = target_twist_alpha_ * (delta_pose.position.x / dt) + (1 - target_twist_alpha_) * last_vx;
    target_twist_.linear.y = target_twist_alpha_ * (delta_pose.position.y / dt) + (1 - target_twist_alpha_) * last_vy;
    target_twist_.linear.z = target_twist_alpha_ * (delta_pose.position.z / dt) + (1 - target_twist_alpha_) * last_vz;

    last_vx = target_twist_.linear.x;
    last_vy = target_twist_.linear.y;
    last_vz = target_twist_.linear.z;
    return;
};

void FollowTargetBase::computeReference(const double &dt)
{
    
    rclcpp::Duration duration(target_pose_->header.stamp.sec, target_pose_->header.stamp.nanosec);
    double delta_time = (node_ptr_->now() - duration).seconds();

    double distance2d = ft_utils::computeDistance2D(last_target_pose_.pose.position.x, last_target_pose_.pose.position.y,
                                                    target_pose_->pose.position.x, target_pose_->pose.position.y);

    if (distance2d > 300.0)
    {
        reference_pose_ = last_target_pose_.pose;
        return;
    }
    reference_pose_ = target_pose_->pose;

    // RCLCPP_INFO(node_ptr_->get_logger(), "Base-computeReference-Last time: %f", duration.seconds());
    // RCLCPP_INFO(node_ptr_->get_logger(), "Base-computeReference-Current time: %f", node_ptr_->now().seconds());
    // RCLCPP_INFO(node_ptr_->get_logger(), "Base-computeReference-Delta time: %f", delta_time);
    // RCLCPP_INFO(node_ptr_->get_logger(), "Base-computeReference-Twist: %f %f %f", target_twist_.linear.x,
    //             target_twist_.linear.y, target_twist_.linear.z);

    // RCLCPP_INFO(node_ptr_->get_logger(), "Base-computeReference-Reference: %f %f %f", reference_pose_.position.x,
    //             reference_pose_.position.y, reference_pose_.position.z);

    // reference_pose_.position.x += target_twist_.linear.x * (delta_time + target_pose_predict_factor_ * dt);
    // reference_pose_.position.y += target_twist_.linear.y * (delta_time + target_pose_predict_factor_ * dt);
    // reference_pose_.position.z += target_twist_.linear.z * (delta_time + target_pose_predict_factor_ * dt);

    // RCLCPP_INFO(node_ptr_->get_logger(), "Base-computeReference-delta_time %f", delta_time);
    // RCLCPP_INFO(node_ptr_->get_logger(), "Base-computeReference-Twist: %f %f %f", target_twist_.linear.x,
    //             target_twist_.linear.y, target_twist_.linear.z);
    // RCLCPP_INFO(node_ptr_->get_logger(), "Base-computeReference-Reference \n: %f %f %f", reference_pose_.position.x,
    //             reference_pose_.position.y, reference_pose_.position.z);

    // reference_pose_.position.x += target_twist_.linear.x * dt * target_pose_predict_factor_;
    // reference_pose_.position.y += target_twist_.linear.y * dt * target_pose_predict_factor_;
    // reference_pose_.position.z += target_twist_.linear.z * dt * target_pose_predict_factor_;
    return;
};

void FollowTargetBase::computeTargetMeanHeight()
{
    if (first_run)
    {
        last_target_height = target_pose_->pose.position.z;
    }

    target_mean_height =
        target_height_alpha_ * target_pose_->pose.position.z + (1 - target_height_alpha_) * last_target_height;
    return;
};

Eigen::Vector3d FollowTargetBase::computeControl(const double &dt,
                                                 const Eigen::Vector3d &speed_limit = Eigen::Vector3d::Zero(),
                                                 const bool &proportional_limitation = false)
{
    ft_speed_controller::UAV_state state;
    state.pos = Eigen::Vector3d(sl_pose_->pose.position.x, sl_pose_->pose.position.y, sl_pose_->pose.position.z);

    ft_speed_controller::Control_ref ref;
    ref.pos = Eigen::Vector3d(reference_pose_.position.x, reference_pose_.position.y, reference_pose_.position.z);

    Eigen::Vector3d control_cmd = controller_handler_->computePositionControl(state, ref, dt);
    Eigen::Vector3d motion_speed = controller_handler_->limitSpeed(control_cmd, speed_limit, proportional_limitation);

    // RCLCPP_INFO(node_ptr_->get_logger(), "Base-Control-dt   : %f", dt);
    // RCLCPP_INFO(node_ptr_->get_logger(), "Base-Control-State: %f %f %f", state.pos(0), state.pos(1), state.pos(2));
    // RCLCPP_INFO(node_ptr_->get_logger(), "Base-Control-Ref  : %f %f %f", ref.pos(0), ref.pos(1), ref.pos(2));
    // RCLCPP_INFO(node_ptr_->get_logger(), "Base-Control-Cmd  : %f %f %f", control_cmd(0), control_cmd(1),
    //             control_cmd(2));
    // RCLCPP_INFO(node_ptr_->get_logger(), "Base-Control-Speed: %f %f %f", motion_speed(0), motion_speed(1),
    //             motion_speed(2));
    // Eigen::Vector3d error = (ref.pos - state.pos);
    // RCLCPP_INFO(node_ptr_->get_logger(), "Base-Control-Error: %f %f %f", error(0), error(1), error(2));
    // RCLCPP_INFO(node_ptr_->get_logger(), "Base-Control-Error: %f\n", error.norm());

    return motion_speed;

    // static Eigen::Vector3d last_motion_speed = motion_speed;
    // double motion_speed_alpha_ = 0.4;
    // motion_speed.x() = motion_speed_alpha_ * motion_speed.x() + (1 - motion_speed_alpha_) * last_motion_speed.x();
    // motion_speed.y() = motion_speed_alpha_ * motion_speed.y() + (1 - motion_speed_alpha_) * last_motion_speed.y();
    // motion_speed.z() = motion_speed_alpha_ * motion_speed.z() + (1 - motion_speed_alpha_) * last_motion_speed.z();

    // return control_cmd;
};

double FollowTargetBase::getPathFacingAngle()
{
    double x_dif = target_pose_->pose.position.x - sl_pose_->pose.position.x;
    double y_dif = target_pose_->pose.position.y - sl_pose_->pose.position.y;
    return as2::FrameUtils::getVector2DAngle(x_dif, y_dif);
};

double FollowTargetBase::computeYawControl(const double &dt, const double &desired_yaw)
{
    double current_yaw = as2::FrameUtils::getYawFromQuaternion(sl_pose_->pose.orientation);
    return controller_handler_->computeYawSpeed(current_yaw, desired_yaw, dt);
};

double FollowTargetBase::computeYawDiff(const double &desired_yaw, const double &current_yaw)
{
    double yaw_diff = desired_yaw - current_yaw;
    if (yaw_diff > M_PI)
    {
        yaw_diff -= 2 * M_PI;
    }
    else if (yaw_diff < -M_PI)
    {
        yaw_diff += 2 * M_PI;
    }
    return yaw_diff;
}

Eigen::Vector2d FollowTargetBase::computeSpeedDif2d(const geometry_msgs::msg::Twist &v0,
                                                    const geometry_msgs::msg::Twist &v1)
{
    return Eigen::Vector2d((v1.linear.x - v0.linear.x), (v1.linear.y - v0.linear.y));
};

Eigen::Vector3d FollowTargetBase::computeSpeedDif3d(const geometry_msgs::msg::Twist &v0,
                                                    const geometry_msgs::msg::Twist &v1)
{
    return Eigen::Vector3d((v1.linear.x - v0.linear.x), (v1.linear.y - v0.linear.y), (v1.linear.z - v0.linear.z));
};

Eigen::Vector2d FollowTargetBase::computeRelativeSpeedTargetUav2d()
{
    return computeSpeedDif2d(target_twist_, sl_twist_->twist);
};

Eigen::Vector3d FollowTargetBase::computeRelativeSpeedTargetUav3d()
{
    return computeSpeedDif3d(target_twist_, sl_twist_->twist);
};

void FollowTargetBase::run(const double &dt)
{
    if (first_run)
    {
        RCLCPP_WARN(node_ptr_->get_logger(), "First run");
        last_target_pose_ = *target_pose_.get();
    }
    
    computeTargetSpeed();
    computeReference(dt);
    computeTargetMeanHeight();
    ownRun(dt);

    double distance2d = ft_utils::computeDistance2D(last_target_pose_.pose.position.x, last_target_pose_.pose.position.y,
                                                    target_pose_->pose.position.x, target_pose_->pose.position.y);

    if (distance2d < 300.0)
    {
        last_target_pose_ = *target_pose_.get();
        return;
    }

    first_run = false;
};

} // namespace ft_base