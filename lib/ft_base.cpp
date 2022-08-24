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
        node_ptr_->declare_parameter(base_parameters[i]); // TODO: WARNING on galactic and advance
    }
    ownDeclareParameters();
    return;
};

void FollowTargetBase::updateParam(const rclcpp::Parameter param)
{
    RCLCPP_INFO(node_ptr_->get_logger(), "Base-UpdateParam: %s", param.get_name().c_str());
    if (param.get_name() == "target_twist_alpha")
        target_twist_alpha = param.get_value<float>();
    else if (param.get_name() == "target_pose_predict_factor")
        target_pose_predict_factor_ = param.get_value<float>();
    else if (param.get_name() == "target_height_alpha")
        target_height_alpha = param.get_value<float>();

    ownUpdateParam(param);
    return;
};

void FollowTargetBase::computeTargetSpeed(const double &dt)
{
    // Linear velocity
    geometry_msgs::msg::Pose delta_pose;

    delta_pose.position.x = target_pose_->pose.position.x - last_target_pose_.position.x;
    delta_pose.position.y = target_pose_->pose.position.y - last_target_pose_.position.y;
    delta_pose.position.z = target_pose_->pose.position.z - last_target_pose_.position.z;

    static auto last_vx = delta_pose.position.x / dt;
    static auto last_vy = delta_pose.position.y / dt;
    static auto last_vz = delta_pose.position.z / dt;

    target_twist_.linear.x = target_twist_alpha * (delta_pose.position.x / dt) + (1 - target_twist_alpha) * last_vx;
    target_twist_.linear.y = target_twist_alpha * (delta_pose.position.y / dt) + (1 - target_twist_alpha) * last_vy;
    target_twist_.linear.z = target_twist_alpha * (delta_pose.position.z / dt) + (1 - target_twist_alpha) * last_vz;

    last_vx = target_twist_.linear.x;
    last_vy = target_twist_.linear.y;
    last_vz = target_twist_.linear.z;
    return;
};

void FollowTargetBase::computeReference(const double &dt)
{
    reference_pose_ = target_pose_->pose;
    reference_pose_.position.x += target_twist_.linear.x * dt * target_pose_predict_factor_;
    reference_pose_.position.y += target_twist_.linear.y * dt * target_pose_predict_factor_;
    reference_pose_.position.z += target_twist_.linear.z * dt * target_pose_predict_factor_;
    return;
};

void FollowTargetBase::computeTargetMeanHeight(const double &dt)
{
    target_mean_height =
        target_height_alpha * target_pose_->pose.position.z + (1 - target_height_alpha) * last_target_pose_.position.z;
    return;
};

Eigen::Vector3d FollowTargetBase::computeControl(const double &dt)
{
    ft_speed_controller::UAV_state state;
    state.pos = Eigen::Vector3d(sl_pose_->pose.position.x, sl_pose_->pose.position.y, sl_pose_->pose.position.z);

    ft_speed_controller::Control_ref ref;
    ref.pos = Eigen::Vector3d(reference_pose_.position.x, reference_pose_.position.y, reference_pose_.position.z);

    Eigen::Vector3d control_cmd = controller_handler_->computePositionControl(state, ref, dt);
    Eigen::Vector3d motion_speed =
        controller_handler_->limitSpeed(control_cmd, *speed_limit_.get(), proportional_limitation_);

    return control_cmd;
};

void FollowTargetBase::run(const double &dt)
{
    if (first_run)
    {
        last_target_pose_ = target_pose_->pose;
        first_run = false;
    }
    computeTargetSpeed(dt);
    computeReference(dt);
    computeTargetMeanHeight(dt);
    ownRun(dt);
    last_target_pose_ = target_pose_->pose;
};

} // namespace ft_base