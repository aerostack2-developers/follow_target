#include "ft_unpick.hpp"

namespace ft_unpick
{
UnPick::UnPick(ft_base::FTBaseStruct tf_base_struct) : ft_base::FollowTargetBase(tf_base_struct)
{
    /* Subscribers */
    // node_ptr_->declare_parameter("unpick.gripper_state_topic", "/gripper_state_topic");
    ft_utils::declareParameters(node_ptr_, "unpick.gripper_state_topic", "/gripper_state_topic");
    std::string gripper_state_topic = node_ptr_->get_parameter("unpick.gripper_state_topic").as_string();
    gripper_sensor_sub_ = node_ptr_->create_subscription<std_msgs::msg::Bool>(
        gripper_state_topic, as2_names::topics::sensor_measurements::qos,
        std::bind(&UnPick::gripperSensorCallback, this, std::placeholders::_1));

    // node_ptr_->declare_parameter("unpick.gripper_actuator_topic", "/gripper_actuator_topic");
    ft_utils::declareParameters(node_ptr_, "unpick.gripper_actuator_topic", "/gripper_actuator_topic");
    std::string gripper_actuator_topic = node_ptr_->get_parameter("unpick.gripper_actuator_topic").as_string();
    gripper_actuator_sub_ = node_ptr_->create_subscription<std_msgs::msg::Bool>(
        gripper_actuator_topic, as2_names::topics::sensor_measurements::qos,
        std::bind(&UnPick::gripperActuatorCallback, this, std::placeholders::_1));

    /* Publishers */
    gripper_actuator_pub_ =
        node_ptr_->create_publisher<std_msgs::msg::Bool>(gripper_actuator_topic,
                                                         rclcpp::QoS(10)); // TODO: rclcpp::SensorDataQoS()
};

void UnPick::ownDeclareParameters()
{
    for (int i = 0; i < unpick_parameters.size(); i++)
    {
        // node_ptr_->declare_parameter(unpick_parameters[i]); // TODO: WARNING on galactic and advance
        ft_utils::declareParameters(node_ptr_, unpick_parameters[i]);
    }
    return;
};

void UnPick::ownUpdateParam(rclcpp::Parameter param)
{
    // RCLCPP_INFO(node_ptr_->get_logger(), "UnPick-UpdateParam: %s", param.get_name().c_str());
    if (param.get_name() == "unpick.unpick_approach_2D_threshold")
        unpick_approach_2D_threshold_ = param.get_value<float>();
    else if (param.get_name() == "unpick.unpick_approach_height_threshold")
        unpick_approach_height_threshold_ = param.get_value<float>();
    else if (param.get_name() == "unpick.unpick_approach_speed_threshold")
        unpick_approach_speed_threshold_ = param.get_value<float>();
    else if (param.get_name() == "unpick.gripper_height")
        gripper_height_ = param.get_value<float>();
    else if (param.get_name() == "unpick.vessel_height")
        vessel_height_ = param.get_value<float>();
    else if (param.get_name() == "unpick.object_height")
        object_height_ = param.get_value<float>();
    else if (param.get_name() == "unpick.delivery_height")
        delivery_height_ = param.get_value<float>();
    return;
};

void UnPick::gripperSensorCallback(const std::shared_ptr<std_msgs::msg::Bool> _msg)
{
    gripper_contact_ = _msg->data;
    return;
};

void UnPick::gripperActuatorCallback(const std::shared_ptr<std_msgs::msg::Bool> _msg)
{
    gripper_actuator_ = _msg->data;
    return;
};

void UnPick::publishGripper(const bool &_state)
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

void UnPick::checkGripperContact()
{
    if (object_gripped_ && gripper_contact_ && gripper_actuator_)
    {
        return;
    }

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

void UnPick::ownResetState()
{
    current_phase_ = 0;
    return;
};

void UnPick::ownRun(const double &dt)
{
    checkGripperContact();
    Eigen::Vector3d speed_limit = *speed_limit_.get();
    bool proportional_speed_limit = proportional_limitation_;
    double yaw_speed = 0.0;

    switch (current_phase_)
    {
    case 0: {
        RCLCPP_INFO(node_ptr_->get_logger(), "UnPick phase 0: Approach to USV");
        publishGripper(true);

        reference_pose_.position.z = vessel_height_;
        // RCLCPP_INFO(node_ptr_->get_logger(), "Vessel height: %f", reference_pose_.position.z);

        double distance2d = ft_utils::computeDistance2D(sl_pose_->pose.position.x, sl_pose_->pose.position.y,
                                                        target_pose_->pose.position.x, target_pose_->pose.position.y);

        double distance1d = ft_utils::computeDistance1D(sl_pose_->pose.position.z, reference_pose_.position.z);

        double relative_speed = computeRelativeSpeedTargetUav2d().norm();

        if (distance2d >= 2.5)
        {
            yaw_speed = computeYawControl(dt, getPathFacingAngle());
        }

        if (distance2d < unpick_approach_2D_threshold_ * 20.0 && distance1d < unpick_approach_height_threshold_ * 20.0)
        {
            current_phase_++;
        }
        if (!object_gripped_)
        {
            RCLCPP_INFO_ONCE(node_ptr_->get_logger(), "Object lost");
            finished = true;
        }
        break;
    }
    case 1: {
        RCLCPP_INFO(node_ptr_->get_logger(), "UnPick phase 0: Approach to USV");
        publishGripper(true);

        reference_pose_.position.z = vessel_height_;
        // RCLCPP_INFO(node_ptr_->get_logger(), "Vessel height: %f", reference_pose_.position.z);

        double distance2d = ft_utils::computeDistance2D(sl_pose_->pose.position.x, sl_pose_->pose.position.y,
                                                        target_pose_->pose.position.x, target_pose_->pose.position.y);

        double distance1d = ft_utils::computeDistance1D(sl_pose_->pose.position.z, reference_pose_.position.z);

        double relative_speed = computeRelativeSpeedTargetUav2d().norm();

        yaw_speed =
            computeYawControl(dt, as2::FrameUtils::getVector2DAngle(target_twist_.linear.x, target_twist_.linear.y));

        double yaw_diff =
            computeYawDiff(as2::FrameUtils::getVector2DAngle(target_twist_.linear.x, target_twist_.linear.y),
                           as2::FrameUtils::getYawFromQuaternion(sl_pose_->pose.orientation));

        if (distance2d < unpick_approach_2D_threshold_ * 10.0 &&
            distance1d < unpick_approach_height_threshold_ * 10.0 &&
            relative_speed < unpick_approach_speed_threshold_ && yaw_diff < 0.2)
        {
            current_phase_++;
        }
        if (!object_gripped_)
        {
            RCLCPP_INFO_ONCE(node_ptr_->get_logger(), "Object lost");
            finished = true;
        }
        break;
    }
    case 2: {
        RCLCPP_INFO(node_ptr_->get_logger(), "UnPick phase 2: Approach to USV delivery point");
        publishGripper(true);

        // reference_pose_.position.z = target_mean_height + object_height_ + gripper_height_ + delivery_height_;
        reference_pose_.position.z = 1.5f;

        proportional_speed_limit = false;
        speed_limit.z() = 0.2;

        double distance2d = ft_utils::computeDistance2D(sl_pose_->pose.position.x, sl_pose_->pose.position.y,
                                                        target_pose_->pose.position.x, target_pose_->pose.position.y);

        double distance1d = ft_utils::computeDistance1D(sl_pose_->pose.position.z, reference_pose_.position.z);

        double relative_speed = computeRelativeSpeedTargetUav2d().norm();

        if (distance2d < unpick_approach_2D_threshold_ && distance1d < unpick_approach_height_threshold_ &&
            relative_speed < unpick_approach_speed_threshold_)
        {
            current_phase_++;
        }
        if (!object_gripped_)
        {
            RCLCPP_INFO_ONCE(node_ptr_->get_logger(), "Object lost");
            finished = true;
        }
        break;
    }
    case 3: {
        RCLCPP_INFO(node_ptr_->get_logger(), "UnPick phase 3: Deliver object");
        publishGripper(false);

        reference_pose_.position.z = target_mean_height + object_height_ + gripper_height_ + delivery_height_;
        // RCLCPP_INFO(node_ptr_->get_logger(), "Delivery height: %f", reference_pose_.position.z);
        // RCLCPP_INFO(node_ptr_->get_logger(), "Object height: %f", object_height_);
        // RCLCPP_INFO(node_ptr_->get_logger(), "Gripper height: %f", gripper_height_);
        // RCLCPP_INFO(node_ptr_->get_logger(), "Delivery height: %f", delivery_height_);
        // RCLCPP_INFO(node_ptr_->get_logger(), "Target mean height: %f", target_mean_height);
        // RCLCPP_INFO(node_ptr_->get_logger(), "Self height: %f", sl_pose_->pose.position.z);

        proportional_speed_limit = false;
        speed_limit.z() = 0.2;

        if (!object_gripped_)
        {
            unpick_position_ =
                Eigen::Vector3d(sl_pose_->pose.position.x, sl_pose_->pose.position.y, sl_pose_->pose.position.z);
            current_phase_++;
        }
        break;
    }
    case 4: {
        RCLCPP_INFO(node_ptr_->get_logger(), "UnPick phase 4: Hover");
        publishGripper(true);

        reference_pose_.position.x = unpick_position_.x();
        reference_pose_.position.y = unpick_position_.y();
        reference_pose_.position.z = unpick_position_.z() + vessel_height_;

        // RCLCPP_INFO(node_ptr_->get_logger(), "Go to unpick position: %f, %f, %f", reference_pose_.position.x,
        //             reference_pose_.position.y, reference_pose_.position.z);

        if (sl_pose_->pose.position.z > reference_pose_.position.z)
        {
            finished = true;
            return;
        }
        break;
    }
    default:
        RCLCPP_ERROR(node_ptr_->get_logger(), "UnPick phase %d not implemented", current_phase_);
        break;
    }

    Eigen::Vector3d motion_speed_ = computeControl(dt, speed_limit, proportional_speed_limit);
    motion_handler_speed_->sendSpeedCommandWithYawSpeed(motion_speed_.x(), motion_speed_.y(), motion_speed_.z(),
                                                        yaw_speed);

    // RCLCPP_INFO(node_ptr_->get_logger(), "target_pose: %f, %f, %f", target_pose_->pose.position.x,
    //             target_pose_->pose.position.y, target_pose_->pose.position.z);

    // RCLCPP_INFO(node_ptr_->get_logger(), "reference_pose_: %f, %f, %f", reference_pose_.position.x,
    //             reference_pose_.position.y, reference_pose_.position.z);
    // RCLCPP_INFO(node_ptr_->get_logger(), "state: %f, %f, %f", sl_pose_->pose.position.x, sl_pose_->pose.position.y,
    //             sl_pose_->pose.position.z);
    // RCLCPP_INFO(node_ptr_->get_logger(), "motion_speed_: %f, %f, %f", motion_speed_.x(), motion_speed_.y(),
    //             motion_speed_.z());
    // RCLCPP_INFO(node_ptr_->get_logger(), "speed_limit: %f, %f, %f", speed_limit.x(), speed_limit.y(),
    // speed_limit.z()); RCLCPP_INFO(node_ptr_->get_logger(), "proportional_speed_limit: %d \n",
    // proportional_speed_limit);
    return;
};
} // namespace ft_unpick