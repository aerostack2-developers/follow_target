#include "follow_target.hpp"

namespace follow_target
{

  FollowTarget::FollowTarget() : as2::Node("follow_target") 
  {
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

    RCLCPP_INFO(this->get_logger(), "base_frame: %s", base_frame_.c_str());

    this->declare_parameter("target_topic", "/target_pose");
    target_topic_ = this->get_parameter("target_topic").as_string();

    this->declare_parameter("proportional_limitation", true);
    proportional_limitation_ = this->get_parameter("proportional_limitation").as_bool();

    this->declare_parameter("speed_limit.vx", 0.0);
    double vx = this->get_parameter("speed_limit.vx").as_double();
    this->declare_parameter("speed_limit.vy", 0.0);
    double vy = this->get_parameter("speed_limit.vy").as_double();
    this->declare_parameter("speed_limit.vz", 0.0);
    double vz = this->get_parameter("speed_limit.vz").as_double();
    speed_limit_ = Vector3d(vx, vy, vz);

    controller_handler_ = std::make_shared<SpeedController>();

    std::vector<std::pair<std::string, double>> parameters_list = controller_handler_->getParametersList();
    for (auto &param : parameters_list)
    {
      this->declare_parameter<double>(param.first, param.second);
      double param_value = this->get_parameter(param.first).as_double();
      controller_handler_->setParameter(param.first, param_value);
    }

    // Timer to send command
    static auto timer_commands_ =
        this->create_wall_timer(
            std::chrono::milliseconds(100),
            [this]()
            { this->run(); });
  }

  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

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

  void FollowTarget::run()
  {
    if (!is_active_)
    {
      return;
    }

    last_time_ = rclcpp::Clock().now();
    auto current_time = rclcpp::Clock().now();
    auto dt = (current_time - last_time_).seconds();

    if (manage_flags_.ref_received && manage_flags_.state_received)
    {

      RCLCPP_INFO(this->get_logger(), "State (target_pose_): %f %f %f", target_pose_.pose.position.x, target_pose_.pose.position.y, target_pose_.pose.position.z);
      RCLCPP_INFO(this->get_logger(), "Ref   (ref_pose_):    %f %f %f", ref_pose_.pose.position.x, ref_pose_.pose.position.y, ref_pose_.pose.position.z);
      RCLCPP_INFO(this->get_logger(), "\n");

      computeRefPose();
      follow_target_speed_controller::UAV_state state;
      state.pos = Vector3d(target_pose_.pose.position.x, target_pose_.pose.position.y, target_pose_.pose.position.z);
      // state.vel = Vector3d(target_pose_.twist.linear.x, target_pose_.twist.linear.y, target_pose_.twist.linear.z);
      // state.rot = tf2::Quaternion(target_pose_.pose.orientation.x, target_pose_.pose.orientation.y, target_pose_.pose.orientation.z, target_pose_.pose.orientation.w).getRPY();
     
      follow_target_speed_controller::Control_ref ref;
      ref.pos = Vector3d(ref_pose_.pose.position.x, ref_pose_.pose.position.y, ref_pose_.pose.position.z);
      // ref.vel = Vector3d(ref_twist_.twist.linear.x, ref_twist_.twist.linear.y, ref_twist_.twist.linear.z);
      // ref.yaw = tf2::Quaternion(ref_pose_.pose.orientation.x, ref_pose_.pose.orientation.y, ref_pose_.pose.orientation.z, ref_pose_.pose.orientation.w).getYaw();

      // Speed controller from base_link to target_pose_:
      // reference = desired target position 
      // state = current target position
      // Output = desired speed of target relative to base_link
      // -1.0f * output = desired speed of base_link relative to target_pose_
      Vector3d control_cmd = -controller_handler_->computePositionControl(state, ref, dt); 
      Vector3d motion_speed = controller_handler_->limitSpeed(control_cmd, speed_limit_, proportional_limitation_);

      motion_handler_speed_->sendSpeedCommandWithYawSpeed(motion_speed.x(), motion_speed.y(), motion_speed.z(), 0.0);

      last_time_ = current_time;
    }
  }

  void FollowTarget::setupNode()
  {
    motion_handler_speed_ = std::make_shared<as2::motionReferenceHandlers::SpeedMotion>(this);

    sl_pose_sub_ = std::make_shared<message_filters::Subscriber<geometry_msgs::msg::PoseStamped>>(this, as2_names::topics::self_localization::pose, as2_names::topics::self_localization::qos.get_rmw_qos_profile());
    sl_twist_sub_ = std::make_shared<message_filters::Subscriber<geometry_msgs::msg::TwistStamped>>(this, as2_names::topics::self_localization::twist, as2_names::topics::self_localization::qos.get_rmw_qos_profile());
    synchronizer_ = std::make_shared<message_filters::Synchronizer<approximate_policy>>(approximate_policy(5), *(sl_pose_sub_.get()), *(sl_twist_sub_.get()));
    synchronizer_->registerCallback(&FollowTarget::state_callback, this);

    target_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        this->generate_global_name(target_topic_),
        as2_names::topics::sensor_measurements::qos,
        std::bind(&FollowTarget::targetPoseCallback, this, std::placeholders::_1));

    tfBuffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);
  }

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
    try
    {
      geometry_msgs::msg::TransformStamped tf;
      tf = tfBuffer_->lookupTransform(base_frame_, _msg->header.frame_id, _msg->header.stamp,
                                      rclcpp::Duration::from_nanoseconds(100000));
      target_pose_.header = _msg->header;
      target_pose_.header.frame_id = base_frame_;
      target_pose_.pose.position.x = tf.transform.translation.x + _msg->pose.position.x;
      target_pose_.pose.position.y = tf.transform.translation.y + _msg->pose.position.y;
      target_pose_.pose.position.z = tf.transform.translation.z + _msg->pose.position.z;

      tf2::Quaternion q_pose(_msg->pose.orientation.x, _msg->pose.orientation.y, _msg->pose.orientation.z, _msg->pose.orientation.w);
      tf2::Quaternion q_tf(tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w);
      tf2::Quaternion q_final = q_tf * q_pose;
      target_pose_.pose.orientation.x = q_final.x();
      target_pose_.pose.orientation.y = q_final.y();
      target_pose_.pose.orientation.z = q_final.z();
      target_pose_.pose.orientation.w = q_final.w();

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

    return;
  }

  void FollowTarget::cleanupNode(){
      // TODO: CLeanup Node
  };

  void FollowTarget::computeRefPose()
  {
    ref_pose_.header = target_pose_.header;
    ref_pose_.pose.position.x = 0.0;
    ref_pose_.pose.position.y = 0.0;
    ref_pose_.pose.position.z = 0.0;
  }

}; // namespace follow_target