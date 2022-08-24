#ifndef FOLLOW_TARGET_HPP_
#define FOLLOW_TARGET_HPP_

#include <rclcpp/rclcpp.hpp>
#include "as2_core/node.hpp"
#include "as2_core/names/topics.hpp"
#include "as2_core/names/services.hpp"
#include "as2_core/names/actions.hpp"
#include "motion_reference_handlers/speed_motion.hpp"
#include "motion_reference_handlers/hover_motion.hpp"
#include "as2_msgs/msg/follow_target_info.hpp"
#include "as2_msgs/srv/dynamic_land.hpp"
#include "as2_msgs/srv/package_pick_up.hpp"
#include "as2_msgs/srv/package_un_pick.hpp"

#include "ft_speed_controller.hpp"
#include "ft_utils.hpp"
#include "ft_pickup.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/bool.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

namespace follow_target
{
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  using Vector3d = Eigen::Vector3d;

  struct Manage_flags
  {
    bool parameters_read;
    bool state_received;
    bool ref_received;
  };

  class FollowTarget : public as2::Node
  {
  public:
    FollowTarget();
    ~FollowTarget(){};

    CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;

  public:
    rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters);

  public:
    void setupNode();
    void cleanupNode();
    void run();

  private:
    /* Subscribers */

    // Self localization
    std::shared_ptr<message_filters::Subscriber<geometry_msgs::msg::PoseStamped>> sl_pose_sub_;
    std::shared_ptr<message_filters::Subscriber<geometry_msgs::msg::TwistStamped>> sl_twist_sub_;
    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::msg::PoseStamped, geometry_msgs::msg::TwistStamped> approximate_policy;
    std::shared_ptr<message_filters::Synchronizer<approximate_policy>> synchronizer_;
    void state_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr pose_msg,
                        const geometry_msgs::msg::TwistStamped::ConstSharedPtr twist_msg);

    // Target localizacion
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;
    void targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr _msg);

    /* Publishers */

    // Node info
    rclcpp::Publisher<as2_msgs::msg::FollowTargetInfo>::SharedPtr info_pub_;
    void publishInfo();

    /* Services */

    // Dynamic land
    rclcpp::Service<as2_msgs::srv::DynamicLand>::SharedPtr set_dynamic_land_srv_;
    void setDynamicLandSrvCall(
        const std::shared_ptr<as2_msgs::srv::DynamicLand::Request> _request,
        std::shared_ptr<as2_msgs::srv::DynamicLand::Response> _response);

    // Package pickup
    rclcpp::Service<as2_msgs::srv::PackagePickUp>::SharedPtr set_package_pickup_srv_;
    void setPackagePickUpSrvCall(
        const std::shared_ptr<as2_msgs::srv::PackagePickUp::Request> _request,
        std::shared_ptr<as2_msgs::srv::PackagePickUp::Response> _response);

    // Package unpick
    rclcpp::Service<as2_msgs::srv::PackageUnPick>::SharedPtr set_package_unpick_srv_;
    void setPackageUnPickSrvCall(
        const std::shared_ptr<as2_msgs::srv::PackageUnPick::Request> _request,
        std::shared_ptr<as2_msgs::srv::PackageUnPick::Response> _response);

  private:
    bool is_active_;
    as2_msgs::msg::FollowTargetInfo current_state_;

    Manage_flags manage_flags_;

    std::shared_ptr<as2::motionReferenceHandlers::SpeedMotion> motion_handler_speed_;
    std::shared_ptr<as2::motionReferenceHandlers::HoverMotion> motion_handler_hover_;

    std::shared_ptr<ft_speed_controller::SpeedController> controller_handler_;
    std::shared_ptr<ft_pickup::PickUp> pickup_handler_;

    std::unique_ptr<tf2_ros::Buffer> tfBuffer_;
    std::shared_ptr<tf2_ros::TransformListener> tfListener_{nullptr};

    geometry_msgs::msg::PoseStamped sl_pose_;
    geometry_msgs::msg::TwistStamped sl_twist_;

    geometry_msgs::msg::PoseStamped target_pose_;
    geometry_msgs::msg::TwistStamped target_twist_;

    geometry_msgs::msg::PoseStamped ref_pose_;
    geometry_msgs::msg::TwistStamped ref_twist_;

    std::string base_frame_ = "";
    std::string target_topic_ = "/target_pose";
    std::string gripper_actuator_topic_ = "/gripper_actuator_topic";
    std::string gripper_sensor_topic_ = "/gripper_state_topic";
    bool proportional_limitation_ = true;
    double unpick_threshold_ = 1.0;
    double predict_factor_ = 1.0;
    double gripper_height = 0.0;
    Vector3d speed_limit_ = Vector3d::Zero();

    std::vector<std::string> dynamic_parameters = {
        "proportional_limitation",
        "unpick_threshold",
        "predict_factor",
        "gripper_height",
        "speed_limit.vx",
        "speed_limit.vy",
        "speed_limit.vz"};

  private:
    void declare_parameters();
    void computeRefPose();
    void resetCommand();
    double computeSpeed(const Vector3d &_twist);
    void resetState();
    Vector3d computeControl(const double &dt);
  };

}; // namespace follow_target

#endif // FOLLOW_TARGET_HPP_
