#include "follow_target.hpp"

namespace follow_target
{
  FollowTarget::FollowTarget() : as2::Node("follow_target")
  {
    controller_handler_ = std::make_shared<ft_speed_controller::SpeedController>();

    static auto parameters_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&FollowTarget::parametersCallback, this, std::placeholders::_1));

    declare_parameters();

    RCLCPP_INFO(this->get_logger(), "PickUp handler");
    ft_base::FTBaseStruct base_struct;
    base_struct.node_ptr = this;
    pickup_handler_ = std::make_shared<ft_pickup::PickUp>(base_struct);

    // Timer to send command
    static auto timer_commands_ =
        this->create_wall_timer(
            std::chrono::milliseconds(100),
            [this]()
            { this->run(); });
  }

  CallbackReturn FollowTarget::on_configure(const rclcpp_lifecycle::State &_state)
  {
    // Set subscriptions, publishers, services, actions, etc. here.
    setupNode();

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
    cleanupNode();
    is_active_ = false;
    return CallbackReturn::SUCCESS;
  };

  CallbackReturn FollowTarget::on_shutdown(const rclcpp_lifecycle::State &_state)
  {
    // Clean other resources here.

    return CallbackReturn::SUCCESS;
  };

  void FollowTarget::setupNode()
  {
    motion_handler_speed_ = std::make_shared<as2::motionReferenceHandlers::SpeedMotion>(this);
    motion_handler_hover_ = std::make_shared<as2::motionReferenceHandlers::HoverMotion>(this);

    // Subscribers
    sl_pose_sub_ = std::make_shared<message_filters::Subscriber<geometry_msgs::msg::PoseStamped>>(this, as2_names::topics::self_localization::pose, as2_names::topics::self_localization::qos.get_rmw_qos_profile());
    sl_twist_sub_ = std::make_shared<message_filters::Subscriber<geometry_msgs::msg::TwistStamped>>(this, as2_names::topics::self_localization::twist, as2_names::topics::self_localization::qos.get_rmw_qos_profile());
    synchronizer_ = std::make_shared<message_filters::Synchronizer<approximate_policy>>(approximate_policy(5), *(sl_pose_sub_.get()), *(sl_twist_sub_.get()));
    synchronizer_->registerCallback(&FollowTarget::state_callback, this);

    target_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        target_topic_,
        as2_names::topics::sensor_measurements::qos,
        std::bind(&FollowTarget::targetPoseCallback, this, std::placeholders::_1));

    // Publishers
    info_pub_ = this->create_publisher<as2_msgs::msg::FollowTargetInfo>(
        as2_names::topics::follow_target::info,
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
  }

  void FollowTarget::cleanupNode(){
      // TODO: CLeanup Node
  };

  void FollowTarget::declare_parameters()
  {
    // Static parameters
    this->declare_parameter("base_frame", "");
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

    this->declare_parameter("target_topic", "/target_pose");
    target_topic_ = this->get_parameter("target_topic").as_string();

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
  }

  rcl_interfaces::msg::SetParametersResult FollowTarget::parametersCallback(const std::vector<rclcpp::Parameter> &parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";

    for (auto &param : parameters)
    {
      if (find(dynamic_parameters.begin(), dynamic_parameters.end(), param.get_name()) != dynamic_parameters.end())
      {
        if (param.get_name() == "proportional_limitation")
          proportional_limitation_ = param.get_value<bool>();
        else if (param.get_name() == "unpick_threshold")
          unpick_threshold_ = param.get_value<double>();
        else if (param.get_name() == "predict_factor")
          predict_factor_ = param.get_value<double>();
        else if (param.get_name() == "speed_limit.vx")
          speed_limit_.x() = param.get_value<double>();
        else if (param.get_name() == "speed_limit.vy")
          speed_limit_.y() = param.get_value<double>();
        else if (param.get_name() == "speed_limit.vz")
          speed_limit_.z() = param.get_value<double>();
      }
      else if (controller_handler_->isParameter(param.get_name()))
      {
        controller_handler_->setParameter(param.get_name(), param.get_value<double>());
      }
      // else
      // {
      //   RCLCPP_WARN(this->get_logger(), "Parameter %s not expected to change dynamically", param.get_name().c_str());
      // }
    }
    return result;
  };

  void FollowTarget::state_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr pose_msg,
                                    const geometry_msgs::msg::TwistStamped::ConstSharedPtr twist_msg)
  {
    sl_pose_ = *pose_msg;
    sl_twist_ = *twist_msg;
    manage_flags_.state_received = true;
    return;
  }

  void FollowTarget::targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr _msg)
  {
    target_pose_ = *_msg;

    // Derive to get the velocity
    static auto last_time = rclcpp::Clock().now();
    auto current_time = rclcpp::Clock().now();
    auto dt = (current_time - last_time).seconds();
    last_time = current_time;

    geometry_msgs::msg::Twist twist_msg;
    static auto last_pose_msg = target_pose_.pose;
    geometry_msgs::msg::Pose delta_pose;
    const double alpha = 0.1f;

    // Linear velocity
    delta_pose.position.x = target_pose_.pose.position.x - last_pose_msg.position.x;
    delta_pose.position.y = target_pose_.pose.position.y - last_pose_msg.position.y;
    delta_pose.position.z = target_pose_.pose.position.z - last_pose_msg.position.z;

    static auto last_vx = delta_pose.position.x / dt;
    static auto last_vy = delta_pose.position.y / dt;
    static auto last_vz = delta_pose.position.z / dt;

    twist_msg.linear.x = alpha * (delta_pose.position.x / dt) + (1 - alpha) * last_vx;
    twist_msg.linear.y = alpha * (delta_pose.position.y / dt) + (1 - alpha) * last_vy;
    twist_msg.linear.z = alpha * (delta_pose.position.z / dt) + (1 - alpha) * last_vz;

    last_vx = twist_msg.linear.x;
    last_vy = twist_msg.linear.y;
    last_vz = twist_msg.linear.z;

    last_pose_msg = target_pose_.pose;

    target_twist_.header = _msg->header;
    target_twist_.header.frame_id = base_frame_;
    target_twist_.twist = twist_msg;

    manage_flags_.ref_received = true;

    /*
    try
    {
      // geometry_msgs::msg::TransformStamped tf;
      // tf = tfBuffer_->lookupTransform(base_frame_, _msg->header.frame_id, _msg->header.stamp,
      //                                 rclcpp::Duration::from_nanoseconds(100000));
      // target_pose_.header = _msg->header;
      // target_pose_.header.frame_id = base_frame_;
      // target_pose_.pose.position.x = tf.transform.translation.x + _msg->pose.position.x;
      // target_pose_.pose.position.y = tf.transform.translation.y + _msg->pose.position.y;
      // target_pose_.pose.position.z = tf.transform.translation.z + _msg->pose.position.z;

      // tf2::Quaternion q_pose(_msg->pose.orientation.x, _msg->pose.orientation.y, _msg->pose.orientation.z, _msg->pose.orientation.w);
      // tf2::Quaternion q_tf(tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w);
      // tf2::Quaternion q_final = q_tf * q_pose;
      // target_pose_.pose.orientation.x = q_final.x();
      // target_pose_.pose.orientation.y = q_final.y();
      // target_pose_.pose.orientation.z = q_final.z();
      // target_pose_.pose.orientation.w = q_final.w();

      // Derive to get the velocity
      static auto last_time = rclcpp::Clock().now();
      auto current_time = rclcpp::Clock().now();
      auto dt = (current_time - last_time).seconds();
      last_time = current_time;

      geometry_msgs::msg::Twist twist_msg;
      static auto last_pose_msg = target_pose_.pose;
      geometry_msgs::msg::Pose delta_pose;
      const double alpha = 0.1f;

      // Linear velocity
      delta_pose.position.x = target_pose_.pose.position.x - last_pose_msg.position.x;
      delta_pose.position.y = target_pose_.pose.position.y - last_pose_msg.position.y;
      delta_pose.position.z = target_pose_.pose.position.z - last_pose_msg.position.z;

      static auto last_vx = delta_pose.position.x / dt;
      static auto last_vy = delta_pose.position.y / dt;
      static auto last_vz = delta_pose.position.z / dt;

      twist_msg.linear.x = alpha * (delta_pose.position.x / dt) + (1 - alpha) * last_vx;
      twist_msg.linear.y = alpha * (delta_pose.position.y / dt) + (1 - alpha) * last_vy;
      twist_msg.linear.z = alpha * (delta_pose.position.z / dt) + (1 - alpha) * last_vz;

      last_vx = twist_msg.linear.x;
      last_vy = twist_msg.linear.y;
      last_vz = twist_msg.linear.z;

      // Angular velocity
      tf2::Quaternion q_target(_msg->pose.orientation.x, _msg->pose.orientation.y, _msg->pose.orientation.z, _msg->pose.orientation.w);
      double roll, pitch, yaw;
      tf2::Matrix3x3(q_target).getRPY(roll, pitch, yaw);

      tf2::Quaternion q_last(last_pose_msg.orientation.x, last_pose_msg.orientation.y, last_pose_msg.orientation.z, last_pose_msg.orientation.w);
      double roll_last, pitch_last, yaw_last;
      tf2::Matrix3x3(q_last).getRPY(roll_last, pitch_last, yaw_last);

      double delta_roll = roll - roll_last;
      double delta_pitch = pitch - pitch_last;
      double delta_yaw = yaw - yaw_last;

      if (delta_roll > M_PI)
        delta_roll -= 2 * M_PI;
      else if (delta_roll < -M_PI)
        delta_roll += 2 * M_PI;

      if (delta_pitch > M_PI)
        delta_pitch -= 2 * M_PI;
      else if (delta_pitch < -M_PI)
        delta_pitch += 2 * M_PI;

      if (delta_yaw > M_PI)
        delta_yaw -= 2 * M_PI;
      else if (delta_yaw < -M_PI)
        delta_yaw += 2 * M_PI;

      static auto last_wx = delta_roll / dt;
      static auto last_wy = delta_pitch / dt;
      static auto last_wz = delta_yaw / dt;

      twist_msg.angular.x = alpha * (delta_roll / dt) + (1 - alpha) * last_wx;
      twist_msg.angular.y = alpha * (delta_pitch / dt) + (1 - alpha) * last_wy;
      twist_msg.angular.z = alpha * (delta_yaw / dt) + (1 - alpha) * last_wz;

      last_wx = twist_msg.angular.x;
      last_wy = twist_msg.angular.y;
      last_wz = twist_msg.angular.z;

      last_pose_msg = sl_pose_.pose;

      target_twist_.header = _msg->header;
      target_twist_.header.frame_id = base_frame_;
      target_twist_.twist = twist_msg;

      manage_flags_.ref_received = true;
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_ERROR(this->get_logger(), "Could not transform %s to %s: %s",
                   base_frame_.c_str(), _msg->header.frame_id.c_str(), ex.what());
      return;
    }

    // RCLCPP_INFO(this->get_logger(), "Target pose: %f %f %f", _msg->pose.position.x, _msg->pose.position.y, _msg->pose.position.z);
    // RCLCPP_INFO(this->get_logger(), "Target tf  : %f %f %f", target_twist_.twist.linear.x, target_twist_.twist.linear.y, target_twist_.twist.linear.z);
    */
    return;
  }

  void FollowTarget::setDynamicLandSrvCall(
      const std::shared_ptr<as2_msgs::srv::DynamicLand::Request> _request,
      std::shared_ptr<as2_msgs::srv::DynamicLand::Response> _response)
  {
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
      speed_limit_ = Vector3d(_request->speed_limit.linear.x, _request->speed_limit.linear.y, _request->speed_limit.linear.z);
      _response->success = true;
      return;
    }

    RCLCPP_WARN(this->get_logger(), "Cannot change to dynamic land %d in state %d", _request->enable, current_state_.follow_status);
    _response->success = false;
    return;
  }

  void FollowTarget::setPackagePickUpSrvCall(
      const std::shared_ptr<as2_msgs::srv::PackagePickUp::Request> _request,
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
    else if (current_state_.follow_status == as2_msgs::msg::FollowTargetInfo::WAITING && _request->enable)
    {
      current_state_.follow_mode = as2_msgs::msg::FollowTargetInfo::PICKUP;
      current_state_.follow_status = as2_msgs::msg::FollowTargetInfo::RUNNING;
      speed_limit_ = Vector3d(_request->speed_limit.linear.x, _request->speed_limit.linear.y, _request->speed_limit.linear.z);
      _response->success = true;
      return;
    }

    RCLCPP_WARN(this->get_logger(), "Cannot change to pick up %d in state %d", _request->enable, current_state_.follow_status);
    _response->success = false;
    return;
  }

  void FollowTarget::setPackageUnPickSrvCall(
      const std::shared_ptr<as2_msgs::srv::PackageUnPick::Request> _request,
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
    else if (current_state_.follow_status == as2_msgs::msg::FollowTargetInfo::WAITING && _request->enable)
    {
      current_state_.follow_mode = as2_msgs::msg::FollowTargetInfo::UNPICK;
      current_state_.follow_status = as2_msgs::msg::FollowTargetInfo::RUNNING;
      speed_limit_ = Vector3d(_request->speed_limit.linear.x, _request->speed_limit.linear.y, _request->speed_limit.linear.z);
      _response->success = true;
      return;
    }

    RCLCPP_WARN(this->get_logger(), "Cannot change to un pick %d in state %d", _request->enable, current_state_.follow_status);
    _response->success = false;
    return;
  }

  void FollowTarget::publishInfo()
  {
    info_pub_->publish(current_state_);
  }

  void FollowTarget::computeRefPose()
  {
    static rclcpp::Time last_time_dt = rclcpp::Clock().now();
    auto current_time = rclcpp::Clock().now();
    auto dt = (current_time - last_time_dt).seconds();
    last_time_dt = current_time;

    // RCLCPP_INFO(this->get_logger(), "Predict factor: %f", predict_factor_);

    ref_pose_ = target_pose_;
    // ref_pose_.pose.position.x += target_twist_.twist.linear.x * dt * predict_factor_;
    // ref_pose_.pose.position.y += target_twist_.twist.linear.y * dt * predict_factor_;
    // ref_pose_.pose.position.z += target_twist_.twist.linear.z * dt * predict_factor_;

    return;
  }

  Vector3d FollowTarget::computeControl(const double &dt)
  {
    // ft_speed_controller::UAV_state state;
    // state.pos = Vector3d(target_pose_.pose.position.x, target_pose_.pose.position.y, target_pose_.pose.position.z);
    // state.vel = Vector3d(target_pose_.twist.linear.x, target_pose_.twist.linear.y, target_pose_.twist.linear.z);
    // state.rot = tf2::Quaternion(target_pose_.pose.orientation.x, target_pose_.pose.orientation.y, target_pose_.pose.orientation.z, target_pose_.pose.orientation.w).getRPY();

    // ft_speed_controller::Control_ref ref;
    // ref.pos = Vector3d(ref_pose_.pose.position.x, ref_pose_.pose.position.y, ref_pose_.pose.position.z);
    // ref.vel = Vector3d(ref_twist_.twist.linear.x, ref_twist_.twist.linear.y, ref_twist_.twist.linear.z);
    // ref.yaw = tf2::Quaternion(ref_pose_.pose.orientation.x, ref_pose_.pose.orientation.y, ref_pose_.pose.orientation.z, ref_pose_.pose.orientation.w).getYaw();

    // Speed controller from base_link to target_pose_:
    // reference = desired target position
    // state = current target position
    // Output = desired speed of target relative to base_link
    // -1.0f * output = desired speed of base_link relative to target_pose_
    // Vector3d control_cmd = -controller_handler_->computePositionControl(state, ref, dt);
    // Vector3d motion_speed = controller_handler_->limitSpeed(control_cmd, speed_limit_, proportional_limitation_);

    ft_speed_controller::UAV_state state;
    state.pos = Vector3d(sl_pose_.pose.position.x, sl_pose_.pose.position.y, sl_pose_.pose.position.z);

    ft_speed_controller::Control_ref ref;
    ref.pos = Vector3d(ref_pose_.pose.position.x, ref_pose_.pose.position.y, ref_pose_.pose.position.z);

    Vector3d control_cmd = controller_handler_->computePositionControl(state, ref, dt);
    Vector3d motion_speed = controller_handler_->limitSpeed(control_cmd, speed_limit_, proportional_limitation_);

    return motion_speed;
  }

  void FollowTarget::resetCommand()
  {
    RCLCPP_INFO(this->get_logger(), "Send hover command");
    motion_handler_hover_->sendHover();
  }

  void FollowTarget::resetState()
  {
    RCLCPP_INFO(this->get_logger(), "Reset state");
    current_state_.follow_status = as2_msgs::msg::FollowTargetInfo::WAITING;
    current_state_.follow_mode = as2_msgs::msg::FollowTargetInfo::UNSET;
    resetCommand();
    speed_limit_ = Vector3d(0.0, 0.0, 0.0);
  }

  void FollowTarget::run()
  {
    if (!is_active_)
    {
      RCLCPP_WARN_ONCE(this->get_logger(), "Follow target is not active");
      return;
    }

    pickup_handler_->test();
    return;

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

    static rclcpp::Time last_time_ = rclcpp::Clock().now();
    rclcpp::Time current_time = rclcpp::Clock().now();
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
    {
      // pickup_handler_->run(dt);
      // pickup_handler_->test();
      return;
      break;
    }
    case as2_msgs::msg::FollowTargetInfo::UNPICK:
      RCLCPP_INFO(this->get_logger(), "UnPick");
      return;
      break;
    case as2_msgs::msg::FollowTargetInfo::DYNAMIC_LAND:
      RCLCPP_INFO(this->get_logger(), "DYNAMIC_LAND");
      return;
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown follow mode");
      break;
    }

    // computeRefPose();
    // ref_pose_.pose.position.z = 2.0f;
    // Vector3d motion_speed = computeControl(dt);
    // motion_handler_speed_->sendSpeedCommandWithYawSpeed(motion_speed.x(), motion_speed.y(), motion_speed.z(), 0.0);
    // RCLCPP_INFO(this->get_logger(), "Dt: %f", dt);
    // RCLCPP_INFO(this->get_logger(), "Tar pose:  %f, %f, %f", target_pose_.pose.position.x, target_pose_.pose.position.y, target_pose_.pose.position.z);
    // RCLCPP_INFO(this->get_logger(), "Tar speed: %f, %f, %f", target_twist_.twist.linear.x, target_twist_.twist.linear.y, target_twist_.twist.linear.z);
    // RCLCPP_INFO(this->get_logger(), "Ref pose:  %f, %f, %f", ref_pose_.pose.position.x, ref_pose_.pose.position.y, ref_pose_.pose.position.z);
    // RCLCPP_INFO(this->get_logger(), "UAV State: %f, %f, %f", sl_pose_.pose.position.x, sl_pose_.pose.position.y, sl_pose_.pose.position.z);
    // double error = computeDistance2D(ref_pose_.pose.position.x, ref_pose_.pose.position.y, sl_pose_.pose.position.x, sl_pose_.pose.position.y);
    // RCLCPP_INFO(this->get_logger(), "Error: %f \n", error);
    // return;

    /*
    checkGripperContact();

    static double predict_factor_acum_error = 0.0;

    switch (current_state_.follow_mode)
    {
    case as2_msgs::msg::FollowTargetInfo::UNSET:
      return;
      break;
    case as2_msgs::msg::FollowTargetInfo::PICKUP:
    {
      static Vector3d pickup_position = Vector3d(target_pose_.pose.position.x, target_pose_.pose.position.y, target_pose_.pose.position.z);
      switch (current_phase_)
      {
      case 0:
      {
        RCLCPP_INFO(this->get_logger(), "Pickup phase 0: Approach to target");
        publishGripper(false);

        computeRefPose();
        ref_pose_.pose.position.z += 0.2;
        Vector3d motion_speed = computeControl(dt);
        motion_handler_speed_->sendSpeedCommandWithYawSpeed(motion_speed.x(), motion_speed.y(), motion_speed.z(), 0.0);

        // TODO: Speed should be relative to the target_pose_
        float threshold = 0.1f;
        if (ft_utils::computeDistance2D(sl_pose_.pose.position.x, sl_pose_.pose.position.y, target_pose_.pose.position.x, target_pose_.pose.position.y) < threshold &&
            ft_utils::computeDistance1D(sl_pose_.pose.position.z, ref_pose_.pose.position.z) < threshold &&
            ft_utils::computeModule(motion_speed) < 0.75f)
        {
          current_phase_++;
          motion_handler_speed_->sendSpeedCommandWithYawSpeed(motion_speed.x(), motion_speed.y(), -0.5f, 0.0);
        }
        break;
      }
      case 1:
      {
        RCLCPP_INFO(this->get_logger(), "Pickup phase 1: Pickup");
        publishGripper(true);
        // ref_pose_ = target_pose_;
        computeRefPose();
        ref_pose_.pose.position.z = target_pose_.pose.position.z;
        Vector3d motion_speed = computeControl(dt);
        motion_handler_speed_->sendSpeedCommandWithYawSpeed(motion_speed.x(), motion_speed.y(), -0.2f, 0.0);

        if (object_gripped_)
        {
          pickup_position = Vector3d(target_pose_.pose.position.x, target_pose_.pose.position.y, target_pose_.pose.position.z);
          current_phase_++;
        }
        break;
      }
      case 2:
      {
        if (!gripper_contact_)
        {
          current_phase_ = 0;
        }
        RCLCPP_INFO(this->get_logger(), "Pickup phase 2: Hold object");
        publishGripper(true);
        ref_pose_.pose.position.x = pickup_position.x();
        ref_pose_.pose.position.y = pickup_position.y();
        ref_pose_.pose.position.z = pickup_position.z() + 1.0;
        // RCLCPP_INFO(this->get_logger(), "Go to pickup position: %f, %f, %f", ref_pose_.pose.position.x, ref_pose_.pose.position.y, ref_pose_.pose.position.z);
        Vector3d motion_speed = computeControl(dt);
        // motion_handler_speed_->sendSpeedCommandWithYawSpeed(motion_speed.x(), motion_speed.y(), motion_speed.z(), 0.0);
        // if (computeDistance1D(sl_pose_.pose.position.z, ref_pose_.pose.position.z) < 0.1)
        motion_handler_speed_->sendSpeedCommandWithYawSpeed(0.0f, 0.0f, 1.0f, 0.0);
        if (sl_pose_.pose.position.z > ref_pose_.pose.position.z)
        {
          resetState();
          return;
        }
        break;
      }
      default:
        break;
      }
      break;
    }
    case as2_msgs::msg::FollowTargetInfo::UNPICK:
      RCLCPP_INFO(this->get_logger(), "UnPick");
      return;
      break;
    case as2_msgs::msg::FollowTargetInfo::DYNAMIC_LAND:
      RCLCPP_INFO(this->get_logger(), "DYNAMIC_LAND");
      return;
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown follow mode");
      break;
    }
    */
    return;
  }

}; // namespace follow_target