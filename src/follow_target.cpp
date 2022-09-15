#include "follow_target.hpp"

namespace follow_target
{
FollowTarget::FollowTarget() : as2::Node("follow_target")
{
    motion_handler_speed_ = std::make_shared<as2::motionReferenceHandlers::SpeedMotion>(this);
    motion_handler_hover_ = std::make_shared<as2::motionReferenceHandlers::HoverMotion>(this);
    controller_handler_ = std::make_shared<ft_speed_controller::SpeedController>();
    sl_pose_ = std::make_shared<geometry_msgs::msg::PoseStamped>();
    sl_twist_ = std::make_shared<geometry_msgs::msg::TwistStamped>();
    target_pose_ = std::make_shared<geometry_msgs::msg::PoseStamped>();
    speed_limit_ = std::make_shared<Vector3d>();

    ft_base::FTBaseStruct base_struct;
    base_struct.node_ptr = this;
    base_struct.controller_handler = controller_handler_;
    base_struct.motion_handler_hover = motion_handler_hover_;
    base_struct.motion_handler_speed = motion_handler_speed_;
    base_struct.sl_pose = sl_pose_;
    base_struct.sl_twist = sl_twist_;
    base_struct.target_pose = target_pose_;
    base_struct.speed_limit = speed_limit_;
    pickup_handler_ = std::make_shared<ft_pickup::PickUp>(base_struct);
    unpick_handler_ = std::make_shared<ft_unpick::UnPick>(base_struct);
    dynamic_follow_handler_ = std::make_shared<ft_dynamic_follow::DynamicFollow>(base_struct);

    end_time_ = this->now();

    static auto parameters_callback_handle_ =
        this->add_on_set_parameters_callback(std::bind(&FollowTarget::parametersCallback, this, std::placeholders::_1));

    declare_parameters();

    // Timer to send command
    static auto timer_commands_ = this->create_wall_timer(std::chrono::milliseconds(100), [this]() { this->run(); });
}

CallbackReturn FollowTarget::on_configure(const rclcpp_lifecycle::State &_state)
{
    // Subscribers
    sl_pose_sub_ = std::make_shared<message_filters::Subscriber<geometry_msgs::msg::PoseStamped>>(
        this, as2_names::topics::self_localization::pose,
        as2_names::topics::self_localization::qos.get_rmw_qos_profile());
    sl_twist_sub_ = std::make_shared<message_filters::Subscriber<geometry_msgs::msg::TwistStamped>>(
        this, as2_names::topics::self_localization::twist,
        as2_names::topics::self_localization::qos.get_rmw_qos_profile());
    synchronizer_ = std::make_shared<message_filters::Synchronizer<approximate_policy>>(
        approximate_policy(5), *(sl_pose_sub_.get()), *(sl_twist_sub_.get()));
    synchronizer_->registerCallback(&FollowTarget::state_callback, this);

    std::string target_pickup_topic_ = this->get_parameter("pickup.target_topic").as_string();
    if (target_pickup_topic_ != "")
    {
        target_pickup_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            target_pickup_topic_, as2_names::topics::sensor_measurements::qos,
            std::bind(&FollowTarget::targetPickUpPoseCallback, this, std::placeholders::_1));
    }

    std::string target_unpick_topic_ = this->get_parameter("unpick.target_topic").as_string();
    if (target_unpick_topic_ != "")
    {
        target_unpick_pose_sub_ = this->create_subscription<as2_msgs::msg::PoseStampedWithID>(
            target_unpick_topic_, rclcpp::SensorDataQoS(),
            std::bind(&FollowTarget::targetUnPickPoseCallback, this, std::placeholders::_1));
    }

    std::string target_dynamic_land_topic_ = this->get_parameter("dynamicLand.target_topic").as_string();
    if (target_dynamic_land_topic_ != "")
    {
        target_dynamic_land_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            target_dynamic_land_topic_, as2_names::topics::sensor_measurements::qos,
            std::bind(&FollowTarget::targetDynamicLandPoseCallback, this, std::placeholders::_1));
    }

    std::string target_dynamic_follow_topic_ = this->get_parameter("dynamicFollow.target_topic").as_string();
    if (target_dynamic_follow_topic_ != "")
    {
        target_dynamic_follow_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            target_dynamic_follow_topic_, as2_names::topics::sensor_measurements::qos,
            std::bind(&FollowTarget::targetDynamicFollowPoseCallback, this, std::placeholders::_1));
    }

    // Publishers
    info_pub_ = this->create_publisher<as2_msgs::msg::FollowTargetInfo>(as2_names::topics::follow_target::info,
                                                                        as2_names::topics::follow_target::qos_info);

    // TF listener
    tfBuffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);

    // Services
    set_dynamic_land_srv_ = this->create_service<as2_msgs::srv::DynamicLand>(
        as2_names::services::behaviour::dynamic_land,
        std::bind(&FollowTarget::setDynamicLandSrvCall, this,
                  std::placeholders::_1, // Corresponds to the 'request'  input
                  std::placeholders::_2  // Corresponds to the 'response' input
                  ));

    set_package_pickup_srv_ = this->create_service<as2_msgs::srv::PackagePickUp>(
        as2_names::services::behaviour::package_pickup,
        std::bind(&FollowTarget::setPackagePickUpSrvCall, this,
                  std::placeholders::_1, // Corresponds to the 'request'  input
                  std::placeholders::_2  // Corresponds to the 'response' input
                  ));

    set_package_unpick_srv_ = this->create_service<as2_msgs::srv::PackageUnPick>(
        as2_names::services::behaviour::package_unpick,
        std::bind(&FollowTarget::setPackageUnPickSrvCall, this,
                  std::placeholders::_1, // Corresponds to the 'request'  input
                  std::placeholders::_2  // Corresponds to the 'response' input
                  ));

    set_dynamic_follow_srv_ = this->create_service<as2_msgs::srv::DynamicFollower>(
        as2_names::services::behaviour::dynamic_follower,
        std::bind(&FollowTarget::setDynamicFollowSrvCall, this,
                  std::placeholders::_1, // Corresponds to the 'request'  input
                  std::placeholders::_2  // Corresponds to the 'response' input
                  ));

    return CallbackReturn::SUCCESS;
};

CallbackReturn FollowTarget::on_activate(const rclcpp_lifecycle::State &_state)
{
    // Set parameters?
    manage_flags_.parameters_read = true;
    manage_flags_.state_received = false;
    manage_flags_.ref_received = false;

    controller_handler_->resetError();
    is_active_ = true;

    return CallbackReturn::SUCCESS;
};

CallbackReturn FollowTarget::on_deactivate(const rclcpp_lifecycle::State &_state)
{
    // Clean up subscriptions, publishers, services, actions, etc. here.
    is_active_ = false;
    return CallbackReturn::SUCCESS;
};

CallbackReturn FollowTarget::on_shutdown(const rclcpp_lifecycle::State &_state)
{
    // Clean other resources here.
    return CallbackReturn::SUCCESS;
};

void FollowTarget::declare_parameters()
{
    // Static parameters
    this->declare_parameter("base_frame", "");
    // ft_utils::declareParameters(this, "base_frame", "");
    std::string base_frame = this->get_parameter("base_frame").as_string();
    std::string ns = this->get_namespace();
    if (base_frame == "")
    {
        base_frame_ = ns.substr(1, ns.length());
        RCLCPP_WARN(get_logger(), "NO BASE FRAME SPECIFIED , USING DEFAULT: %s", base_frame_.c_str());
    }
    else
    {
        base_frame_ = generateTfName(ns, base_frame);
    }

    this->declare_parameter("pickup.target_topic", "");
    this->declare_parameter("unpick.target_topic", "");
    this->declare_parameter("dynamicLand.target_topic", "");
    this->declare_parameter("dynamicFollow.target_topic", "");

    // ft_utils::declareParameters(this, "pickup.target_topic", "");
    // ft_utils::declareParameters(this, "unpick.target_topic", "");
    // ft_utils::declareParameters(this, "dynamicLand.target_topic", "");
    // ft_utils::declareParameters(this, "dynamicFollow.target_topic", "");

    // Dynamic parameters
    std::vector<std::string> params_to_declare(dynamic_parameters);
    for (int i = 0; i < params_to_declare.size(); i++)
    {
        this->declare_parameter(params_to_declare[i]); // TODO: WARNING on galactic and advance
    }

    // Controller parameters
    std::vector<std::pair<std::string, double>> parameters_list = controller_handler_->getParametersList();
    for (auto &param : parameters_list)
    {
        this->declare_parameter<double>(param.first, param.second);
    }

    // PickUp parameters
    pickup_handler_->declareParameters();
    unpick_handler_->declareParameters();
    dynamic_follow_handler_->declareParameters();
}

rcl_interfaces::msg::SetParametersResult FollowTarget::parametersCallback(
    const std::vector<rclcpp::Parameter> &parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";

    for (auto &param : parameters)
    {
        if (find(dynamic_parameters.begin(), dynamic_parameters.end(), param.get_name()) != dynamic_parameters.end())
        {
            // RCLCPP_INFO(this->get_logger(), "FollowTarget-UpdateParam: %s", param.get_name().c_str());
            if (param.get_name() == "speed_limit.vx")
            {
                speed_limit_->x() = param.get_value<double>();
                speed_limit_default_.x() = param.get_value<double>();
            }
            else if (param.get_name() == "speed_limit.vy")
            {
                speed_limit_->y() = param.get_value<double>();
                speed_limit_default_.y() = param.get_value<double>();
            }
            else if (param.get_name() == "speed_limit.vz")
            {
                speed_limit_->z() = param.get_value<double>();
                speed_limit_default_.z() = param.get_value<double>();
            }
        }
        else if (controller_handler_->isParameter(param.get_name()))
        {
            // RCLCPP_INFO(this->get_logger(), "SpeedController-UpdateParam: %s", param.get_name().c_str());
            controller_handler_->setParameter(param.get_name(), param.get_value<double>());
        }
        else
        {
            pickup_handler_->updateParam(param);
            unpick_handler_->updateParam(param);
            dynamic_follow_handler_->updateParam(param);
        }
    }
    return result;
};

void FollowTarget::state_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr pose_msg,
                                  const geometry_msgs::msg::TwistStamped::ConstSharedPtr twist_msg)
{
    *sl_pose_.get() = *pose_msg.get();
    *sl_twist_.get() = *twist_msg.get();
    manage_flags_.state_received = true;
    return;
};

void FollowTarget::targetPickUpPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr _msg)
{
    if (current_state_.follow_mode == as2_msgs::msg::FollowTargetInfo::PICKUP &&
        current_state_.follow_status == as2_msgs::msg::FollowTargetInfo::RUNNING)
    {
        *target_pose_.get() = *_msg.get();
        manage_flags_.ref_received = true;
    }
    return;
};

void FollowTarget::targetUnPickPoseCallback(const as2_msgs::msg::PoseStampedWithID::SharedPtr _msg)
{
    if (current_state_.follow_mode == as2_msgs::msg::FollowTargetInfo::UNPICK &&
        current_state_.follow_status == as2_msgs::msg::FollowTargetInfo::RUNNING)
    {
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header = _msg->header;
        pose_msg.pose = _msg->pose;
        *target_pose_.get() = pose_msg;
        manage_flags_.ref_received = true;
    }
    return;
};

void FollowTarget::targetDynamicLandPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr _msg)
{
    if (current_state_.follow_mode == as2_msgs::msg::FollowTargetInfo::DYNAMIC_LAND &&
        current_state_.follow_status == as2_msgs::msg::FollowTargetInfo::RUNNING)
    {
        *target_pose_.get() = *_msg.get();
        manage_flags_.ref_received = true;
    }
    return;
};

void FollowTarget::targetDynamicFollowPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr _msg)
{
    if (current_state_.follow_mode == as2_msgs::msg::FollowTargetInfo::DYNAMIC_FOLLOWER &&
        current_state_.follow_status == as2_msgs::msg::FollowTargetInfo::RUNNING)
    {
        *target_pose_.get() = *_msg.get();
        manage_flags_.ref_received = true;
    }
    return;
};

void FollowTarget::setDynamicLandSrvCall(const std::shared_ptr<as2_msgs::srv::DynamicLand::Request> _request,
                                         std::shared_ptr<as2_msgs::srv::DynamicLand::Response> _response)
{
    RCLCPP_INFO(this->get_logger(), "DynamicLandSrvCall");
    // Disable dynamic land
    if (current_state_.follow_status == as2_msgs::msg::FollowTargetInfo::DYNAMIC_LAND && !_request->enable)
    {
        current_state_.follow_mode = as2_msgs::msg::FollowTargetInfo::WAITING;
        current_state_.follow_status = as2_msgs::msg::FollowTargetInfo::UNSET;
        _response->success = true;
        return;
    }
    // Enable dynamic land
    else if (current_state_.follow_status == as2_msgs::msg::FollowTargetInfo::WAITING && _request->enable)
    {
        current_state_.follow_mode = as2_msgs::msg::FollowTargetInfo::DYNAMIC_LAND;
        current_state_.follow_status = as2_msgs::msg::FollowTargetInfo::RUNNING;

        Eigen::Vector3d speed_limit = speed_limit_default_;
        if (_request->speed_limit.linear.x != 0)
            speed_limit.x() = _request->speed_limit.linear.x;
        if (_request->speed_limit.linear.y != 0)
            speed_limit.y() = _request->speed_limit.linear.y;
        if (_request->speed_limit.linear.z != 0)
            speed_limit.z() = _request->speed_limit.linear.z;
        *speed_limit_.get() = speed_limit;

        dynamic_follow_handler_->resetState();
        manage_flags_.ref_received = false;

        _response->success = true;
        return;
    }

    RCLCPP_WARN(this->get_logger(), "Cannot change to dynamic land %d in state %d", _request->enable,
                current_state_.follow_status);
    _response->success = false;
    return;
}

void FollowTarget::setPackagePickUpSrvCall(const std::shared_ptr<as2_msgs::srv::PackagePickUp::Request> _request,
                                           std::shared_ptr<as2_msgs::srv::PackagePickUp::Response> _response)
{
    RCLCPP_INFO(this->get_logger(), "Package pick service called");
    // Disable pick up
    if (current_state_.follow_status == as2_msgs::msg::FollowTargetInfo::PICKUP && !_request->enable)
    {
        current_state_.follow_mode = as2_msgs::msg::FollowTargetInfo::WAITING;
        current_state_.follow_status = as2_msgs::msg::FollowTargetInfo::UNSET;
        _response->success = true;
        return;
    }
    // Enable pick up
    else if ((current_state_.follow_status == as2_msgs::msg::FollowTargetInfo::WAITING && _request->enable) ||
             (current_state_.follow_mode == as2_msgs::msg::FollowTargetInfo::DYNAMIC_FOLLOWER && _request->enable))
    {
        current_state_.follow_mode = as2_msgs::msg::FollowTargetInfo::PICKUP;
        current_state_.follow_status = as2_msgs::msg::FollowTargetInfo::RUNNING;

        Eigen::Vector3d speed_limit = speed_limit_default_;
        if (_request->speed_limit.linear.x != 0)
            speed_limit.x() = _request->speed_limit.linear.x;
        if (_request->speed_limit.linear.y != 0)
            speed_limit.y() = _request->speed_limit.linear.y;
        if (_request->speed_limit.linear.z != 0)
            speed_limit.z() = _request->speed_limit.linear.z;
        *speed_limit_.get() = speed_limit;

        pickup_handler_->resetState();
        manage_flags_.ref_received = false;

        _response->success = true;
        return;
    }

    RCLCPP_WARN(this->get_logger(), "Cannot change to pick up %d in state %d", _request->enable,
                current_state_.follow_status);
    _response->success = false;
    return;
}

void FollowTarget::setPackageUnPickSrvCall(const std::shared_ptr<as2_msgs::srv::PackageUnPick::Request> _request,
                                           std::shared_ptr<as2_msgs::srv::PackageUnPick::Response> _response)
{
    RCLCPP_INFO(this->get_logger(), "Package unpick service called");
    // Disable unpick
    if (current_state_.follow_status == as2_msgs::msg::FollowTargetInfo::UNPICK && !_request->enable)
    {
        current_state_.follow_mode = as2_msgs::msg::FollowTargetInfo::WAITING;
        current_state_.follow_status = as2_msgs::msg::FollowTargetInfo::UNSET;
        _response->success = true;
        return;
    }
    // Enable unpick
    else if ((current_state_.follow_status == as2_msgs::msg::FollowTargetInfo::WAITING && _request->enable) ||
             (current_state_.follow_mode == as2_msgs::msg::FollowTargetInfo::DYNAMIC_FOLLOWER && _request->enable))
    {
        current_state_.follow_mode = as2_msgs::msg::FollowTargetInfo::UNPICK;
        current_state_.follow_status = as2_msgs::msg::FollowTargetInfo::RUNNING;

        Eigen::Vector3d speed_limit = speed_limit_default_;
        if (_request->speed_limit.linear.x != 0)
            speed_limit.x() = _request->speed_limit.linear.x;
        if (_request->speed_limit.linear.y != 0)
            speed_limit.y() = _request->speed_limit.linear.y;
        if (_request->speed_limit.linear.z != 0)
            speed_limit.z() = _request->speed_limit.linear.z;
        *speed_limit_.get() = speed_limit;

        unpick_handler_->resetState();
        manage_flags_.ref_received = false;

        _response->success = true;
        return;
    }

    RCLCPP_WARN(this->get_logger(), "Cannot change to un pick %d in state %d", _request->enable,
                current_state_.follow_status);
    _response->success = false;
    return;
};

void FollowTarget::setDynamicFollowSrvCall(const std::shared_ptr<as2_msgs::srv::DynamicFollower::Request> _request,
                                           std::shared_ptr<as2_msgs::srv::DynamicFollower::Response> _response)
{
    RCLCPP_INFO(this->get_logger(), "Dynamic follow service called");
    // Disable dynamic follow
    if (current_state_.follow_status == as2_msgs::msg::FollowTargetInfo::DYNAMIC_FOLLOWER && !_request->enable)
    {
        current_state_.follow_mode = as2_msgs::msg::FollowTargetInfo::WAITING;
        current_state_.follow_status = as2_msgs::msg::FollowTargetInfo::UNSET;
        _response->success = true;
        return;
    }
    // Enable dynamic follow
    else if (current_state_.follow_status == as2_msgs::msg::FollowTargetInfo::WAITING && _request->enable)
    {
        current_state_.follow_mode = as2_msgs::msg::FollowTargetInfo::DYNAMIC_FOLLOWER;
        current_state_.follow_status = as2_msgs::msg::FollowTargetInfo::RUNNING;

        Eigen::Vector3d speed_limit = speed_limit_default_;
        if (_request->speed_limit.linear.x != 0)
            speed_limit.x() = _request->speed_limit.linear.x;
        if (_request->speed_limit.linear.y != 0)
            speed_limit.y() = _request->speed_limit.linear.y;
        if (_request->speed_limit.linear.z != 0)
            speed_limit.z() = _request->speed_limit.linear.z;
        *speed_limit_.get() = speed_limit;

        dynamic_follow_handler_->resetState();
        manage_flags_.ref_received = false;

        _response->success = true;
        return;
    }

    RCLCPP_WARN(this->get_logger(), "Cannot change to dynamic follow %d in state %d", _request->enable,
                current_state_.follow_status);
    _response->success = false;
    return;
}

void FollowTarget::publishInfo()
{
    info_pub_->publish(current_state_);
}

void FollowTarget::resetCommand()
{
    RCLCPP_INFO(this->get_logger(), "Send hover command");
    motion_handler_hover_->sendHover();
}

void FollowTarget::resetState()
{
    RCLCPP_INFO(this->get_logger(), "Reset state");
    current_state_.follow_status = as2_msgs::msg::FollowTargetInfo::END;
    end_time_ = this->now();
    publishInfo();
    resetCommand();
    *speed_limit_.get() = Vector3d::Zero();
    pickup_handler_->resetState();
    unpick_handler_->resetState();
    dynamic_follow_handler_->resetState();
    manage_flags_.ref_received = false;
}

void FollowTarget::run()
{
    if (!is_active_)
    {
        RCLCPP_WARN_ONCE(this->get_logger(), "Follow target is not active");
        return;
    }

    publishInfo();

    rclcpp::Time current_time = this->now();
    if (current_state_.follow_status == as2_msgs::msg::FollowTargetInfo::END)
    {
        double dt_end = (current_time - end_time_).seconds();
        if (dt_end > 0.5f)
        {
            RCLCPP_INFO(this->get_logger(), "Change mode from END to WAITING");
            current_state_.follow_status = as2_msgs::msg::FollowTargetInfo::WAITING;
            current_state_.follow_mode = as2_msgs::msg::FollowTargetInfo::UNSET;
            manage_flags_.ref_received = false;
        }
    }

    if (current_state_.follow_mode == as2_msgs::msg::FollowTargetInfo::UNSET ||
        current_state_.follow_status == as2_msgs::msg::FollowTargetInfo::END ||
        current_state_.follow_status == as2_msgs::msg::FollowTargetInfo::WAITING)
    {
        RCLCPP_INFO(this->get_logger(), "No follow mode");
        return;
    }

    if (!manage_flags_.ref_received || !manage_flags_.state_received)
    {
        if (!manage_flags_.ref_received)
        {
            RCLCPP_WARN(this->get_logger(), "Target pose not received");
        }
        if (!manage_flags_.state_received)
        {
            RCLCPP_WARN(this->get_logger(), "UAV state not received");
        }
        return;
    }

    static rclcpp::Time last_time_ = this->now();
    double dt = (current_time - last_time_).seconds();
    last_time_ = current_time;
    if (dt == 0)
    {
        return;
    }

    switch (current_state_.follow_mode)
    {
    case as2_msgs::msg::FollowTargetInfo::UNSET:
        return;
        break;
    case as2_msgs::msg::FollowTargetInfo::PICKUP:
        pickup_handler_->run(dt);
        if (pickup_handler_->finished)
            resetState();
        break;
    case as2_msgs::msg::FollowTargetInfo::UNPICK:
        unpick_handler_->run(dt);
        if (unpick_handler_->finished)
            resetState();
        return;
        break;
    case as2_msgs::msg::FollowTargetInfo::DYNAMIC_LAND:
        return;
        break;
    case as2_msgs::msg::FollowTargetInfo::DYNAMIC_FOLLOWER:
        dynamic_follow_handler_->run(dt);
        return;
        break;
    default:
        RCLCPP_ERROR(this->get_logger(), "Unknown follow mode");
        break;
    }
    return;
}

}; // namespace follow_target