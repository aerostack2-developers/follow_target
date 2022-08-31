#include "ft_pickup.hpp"

namespace ft_pickup
{
PickUp::PickUp(ft_base::FTBaseStruct tf_base_struct) : ft_base::FollowTargetBase(tf_base_struct)
{
    /* Subscribers */
    // node_ptr_->declare_parameter("pickup.gripper_state_topic", "/gripper_state_topic");
    ft_utils::declareParameters(node_ptr_, "pickup.gripper_state_topic", "/gripper_state_topic");

    std::string gripper_state_topic = node_ptr_->get_parameter("pickup.gripper_state_topic").as_string();
    gripper_sensor_sub_ = node_ptr_->create_subscription<std_msgs::msg::Bool>(
        gripper_state_topic, as2_names::topics::sensor_measurements::qos,
        std::bind(&PickUp::gripperSensorCallback, this, std::placeholders::_1));

    // node_ptr_->declare_parameter("pickup.gripper_actuator_topic", "/gripper_actuator_topic");
    ft_utils::declareParameters(node_ptr_, "pickup.gripper_actuator_topic", "/gripper_actuator_topic");
    std::string gripper_actuator_topic = node_ptr_->get_parameter("pickup.gripper_actuator_topic").as_string();
    gripper_actuator_sub_ = node_ptr_->create_subscription<std_msgs::msg::Bool>(
        gripper_actuator_topic, as2_names::topics::sensor_measurements::qos,
        std::bind(&PickUp::gripperActuatorCallback, this, std::placeholders::_1));

    /* Publishers */
    gripper_actuator_pub_ =
        node_ptr_->create_publisher<std_msgs::msg::Bool>(gripper_actuator_topic,
                                                         rclcpp::QoS(10)); // TODO: rclcpp::SensorDataQoS()
};

void PickUp::ownDeclareParameters()
{
    for (int i = 0; i < pickup_parameters.size(); i++)
    {
        // node_ptr_->declare_parameter(pickup_parameters[i]); // TODO: WARNING on galactic and advance
        ft_utils::declareParameters(node_ptr_, pickup_parameters[i]);
    }
    return;
};

void PickUp::ownUpdateParam(rclcpp::Parameter param)
{
    // RCLCPP_INFO(node_ptr_->get_logger(), "PickUp-UpdateParam: %s", param.get_name().c_str());
    if (param.get_name() == "pickup.pickup_approach_2D_threshold")
        pickup_approach_2D_threshold_ = param.get_value<float>();
    else if (param.get_name() == "pickup.pickup_approach_height_threshold")
        pickup_approach_height_threshold_ = param.get_value<float>();
    else if (param.get_name() == "pickup.pickup_approach_speed_threshold")
        pickup_approach_speed_threshold_ = param.get_value<float>();
    else if (param.get_name() == "pickup.gripper_height")
        gripper_height_ = param.get_value<float>();
    else if (param.get_name() == "pickup.vessel_height")
        vessel_height_ = param.get_value<float>();
    return;
};

void PickUp::gripperSensorCallback(const std::shared_ptr<std_msgs::msg::Bool> _msg)
{
    gripper_contact_ = _msg->data;
    return;
};

void PickUp::gripperActuatorCallback(const std::shared_ptr<std_msgs::msg::Bool> _msg)
{
    gripper_actuator_ = _msg->data;
    return;
};

void PickUp::publishGripper(const bool &_state)
{
    // if (gripper_actuator_ == _state)
    // {
    //     return;
    // }
    std_msgs::msg::Bool msg;
    msg.data = _state;
    gripper_actuator_pub_->publish(msg);
    return;
};

void PickUp::checkGripperContact()
{
    static rclcpp::Time time_0 = rclcpp::Clock().now();
    rclcpp::Time time_1 = rclcpp::Clock().now();
    object_gripped_ = false;

    if (gripper_contact_)
    {
        RCLCPP_INFO_ONCE(node_ptr_->get_logger(), "Object contact detected");
    }
    if (gripper_actuator_)
    {
        RCLCPP_INFO_ONCE(node_ptr_->get_logger(), "Gripper is enable");
    }

    if (gripper_contact_ && gripper_actuator_)
    {
        if ((time_1 - time_0).seconds() > 0.5)
        {
            RCLCPP_INFO_ONCE(node_ptr_->get_logger(), "Object grasped");
            object_gripped_ = true;
        }
        return;
    }
    time_0 = time_1;
    return;
};

void PickUp::computeGripperTransform()
{
    Eigen::Vector3d gripper_traslation = Eigen::Vector3d(0.0, 0.0, gripper_height_);
    tf2::Quaternion q = tf2::Quaternion(sl_pose_->pose.orientation.x, sl_pose_->pose.orientation.y,
                                        sl_pose_->pose.orientation.z, sl_pose_->pose.orientation.w);

    tf2::Matrix3x3 m_tf(q);
    Eigen::Matrix3d uav_rotation;
    uav_rotation << m_tf[0][0], m_tf[0][1], m_tf[0][2], m_tf[1][0], m_tf[1][1], m_tf[1][2], m_tf[2][0], m_tf[2][1],
        m_tf[2][2];

    Eigen::Vector3d gripper_end_effector_position = uav_rotation * gripper_traslation;
}

void PickUp::ownResetState()
{
    current_phase_ = 0;
    return;
};

void PickUp::ownRun(const double &dt)
{
    checkGripperContact();
    Eigen::Vector3d speed_limit = *speed_limit_.get();
    bool proportional_speed_limit = proportional_limitation_;
    double yaw_speed = 0.0;
    static rclcpp::Time pick_up_time = node_ptr_->now();

    switch (current_phase_)
    {
    case 0: {
        RCLCPP_INFO(node_ptr_->get_logger(), "Pickup phase 0: Approach to vessel");
        publishGripper(false);

        // reference_pose_.position.z = vessel_height_;
        proportional_speed_limit = false;
        reference_pose_.position.z = 5.0f;

        double distance2d = ft_utils::computeDistance2D(sl_pose_->pose.position.x, sl_pose_->pose.position.y,
                                                        target_pose_->pose.position.x, target_pose_->pose.position.y);

        double distance1d = ft_utils::computeDistance1D(sl_pose_->pose.position.z, reference_pose_.position.z);

        double relative_speed = computeRelativeSpeedTargetUav2d().norm();

        if (distance2d >= 20.0f)
        {
            yaw_speed = computeYawControl(dt, getPathFacingAngle());
        }

        if (distance2d < pickup_approach_2D_threshold_ * 10.0 && distance1d < pickup_approach_height_threshold_ * 10.0)
        {
            current_phase_++;
        }
        break;
    }
    // case 1: {
    //     RCLCPP_INFO(node_ptr_->get_logger(), "Pickup phase 1: Set angle with target speed");
    //     publishGripper(false);

    //     reference_pose_.position.z = vessel_height_;

    //     double distance2d = ft_utils::computeDistance2D(sl_pose_->pose.position.x, sl_pose_->pose.position.y,
    //                                                     target_pose_->pose.position.x, target_pose_->pose.position.y);

    //     double distance1d = ft_utils::computeDistance1D(sl_pose_->pose.position.z, reference_pose_.position.z);

    //     double relative_speed = computeRelativeSpeedTargetUav2d().norm();

    //     double yaw_diff = 0.0f;
    //     if (Eigen::Vector2d(target_twist_.linear.x, target_twist_.linear.y).norm() > 0.2f)
    //     {
    //         yaw_speed = computeYawControl(
    //             dt, as2::FrameUtils::getVector2DAngle(target_twist_.linear.x, target_twist_.linear.y));

    //         yaw_diff = computeYawDiff(as2::FrameUtils::getVector2DAngle(target_twist_.linear.x, target_twist_.linear.y),
    //                                   as2::FrameUtils::getYawFromQuaternion(sl_pose_->pose.orientation));
    //     }

    //     if (distance2d < pickup_approach_2D_threshold_ * 10.0 &&
    //         distance1d < pickup_approach_height_threshold_ * 10.0 &&
    //         relative_speed < pickup_approach_speed_threshold_ && yaw_diff < 0.1)
    //     {
    //         current_phase_++;
    //     }

    //     break;
    // }
    case 1: {
        RCLCPP_INFO(node_ptr_->get_logger(), "Pickup phase 1: Approach to target");
        publishGripper(false);

        reference_pose_.position.z = target_mean_height + 0.5 + gripper_height_;

        proportional_speed_limit = false;
        speed_limit.z() = 0.2f;

        double distance2d = ft_utils::computeDistance2D(sl_pose_->pose.position.x, sl_pose_->pose.position.y,
                                                        target_pose_->pose.position.x, target_pose_->pose.position.y);

        double distance1d = ft_utils::computeDistance1D(sl_pose_->pose.position.z, reference_pose_.position.z);

        double relative_speed = computeRelativeSpeedTargetUav2d().norm();

        if (distance2d < pickup_approach_2D_threshold_ && distance1d < pickup_approach_height_threshold_ &&
            relative_speed < pickup_approach_speed_threshold_)
        {
            current_phase_++;
            pick_up_time = node_ptr_->now();
        }

        break;
    }
    case 2: {
        RCLCPP_INFO(node_ptr_->get_logger(), "Pickup phase 2: Pickup");

        rclcpp::Time current_time = node_ptr_->now();
        double time_diff = (current_time - pick_up_time).seconds();

        publishGripper(true);

        computeGripperTransform();

        proportional_speed_limit = false;

        Eigen::Vector3d motion_speed_ = computeControl(dt, speed_limit, proportional_speed_limit);

        if (time_diff > 6.0f)
        {
            RCLCPP_WARN(node_ptr_->get_logger(), "Pickup failed. Retrying...");
            vessel_height_ = 2.0f;
            current_phase_ = 1;
            motion_handler_speed_->sendSpeedCommandWithYawSpeed(motion_speed_.x(), motion_speed_.y(), 1.0f, 0.0f);
            return;
        }

        motion_handler_speed_->sendSpeedCommandWithYawSpeed(motion_speed_.x(), motion_speed_.y(), -0.5f, yaw_speed);

        // reference_pose_.position.z -= gripper_height_ + 0.3f;
        // speed_limit.z() = 0.1f;

        if (object_gripped_)
        {
            pickup_position_ = Eigen::Vector3d(target_pose_->pose.position.x, target_pose_->pose.position.y,
                                               target_pose_->pose.position.z);
            current_phase_++;
        }
        break;
    }
    case 3: {
        if (!gripper_contact_)
        {
            current_phase_ = 0;
        }
        RCLCPP_INFO(node_ptr_->get_logger(), "Pickup phase 3: Hold object");
        publishGripper(true);

        reference_pose_.position.x = pickup_position_.x();
        reference_pose_.position.y = pickup_position_.y();
        reference_pose_.position.z = pickup_position_.z() + vessel_height_;

        // RCLCPP_INFO(node_ptr_->get_logger(), "Go to pickup position: %f, %f, %f", reference_pose_.position.x,
        //             reference_pose_.position.y, reference_pose_.position.z);

        if (!object_gripped_)
        {
            RCLCPP_WARN(node_ptr_->get_logger(), "Object not gripped");
            current_phase_ = 1;
        }
        else if (sl_pose_->pose.position.z > reference_pose_.position.z)
        {
            finished = true;
            return;
        }
        break;
    }
    default:
        RCLCPP_ERROR(node_ptr_->get_logger(), "Unknown phase: %d", current_phase_);
        break;
    }

    Eigen::Vector3d motion_speed_ = computeControl(dt, speed_limit, proportional_speed_limit);

    motion_handler_speed_->sendSpeedCommandWithYawSpeed(motion_speed_.x(), motion_speed_.y(), motion_speed_.z(),
                                                        yaw_speed);

    // RCLCPP_INFO(node_ptr_->get_logger(), "motion_speed_: %f, %f, %f \n", motion_speed_.x(), motion_speed_.y(),
    //             motion_speed_.z());

    return;
};
} // namespace ft_pickup