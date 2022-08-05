#ifndef FOLLOW_TARGET_HPP_
#define FOLLOW_TARGET_HPP_

#include <rclcpp/rclcpp.hpp>
#include "as2_core/node.hpp"
#include "as2_core/names/topics.hpp"
#include "as2_core/names/services.hpp"
#include "as2_core/names/actions.hpp"
#include "motion_reference_handlers/speed_motion.hpp"

#include "ft_speed_controller.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


namespace follow_target
{
  using SpeedController = follow_target_speed_controller::SpeedController;
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
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;

  public:
    void setupNode();
    void cleanupNode();
    void run();

  private:
    std::shared_ptr<message_filters::Subscriber<geometry_msgs::msg::PoseStamped>> sl_pose_sub_;
    std::shared_ptr<message_filters::Subscriber<geometry_msgs::msg::TwistStamped>> sl_twist_sub_;
    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::msg::PoseStamped, geometry_msgs::msg::TwistStamped> approximate_policy;
    std::shared_ptr<message_filters::Synchronizer<approximate_policy>> synchronizer_;

    void state_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr pose_msg,
                        const geometry_msgs::msg::TwistStamped::ConstSharedPtr twist_msg);

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;
    void targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr _msg);

  private:
    bool is_active_;
    
    Manage_flags manage_flags_;

    std::string base_frame_;
    std::string target_topic_;

    std::shared_ptr<as2::motionReferenceHandlers::SpeedMotion> motion_handler_speed_;
    std::shared_ptr<SpeedController> controller_handler_;
    Vector3d speed_limit_;
    bool proportional_limitation_;
    
    rclcpp::Time last_time_;

    std::unique_ptr<tf2_ros::Buffer> tfBuffer_;
    std::shared_ptr<tf2_ros::TransformListener> tfListener_{nullptr};

    geometry_msgs::msg::PoseStamped sl_pose_;
    geometry_msgs::msg::TwistStamped sl_twist_;

    geometry_msgs::msg::PoseStamped target_pose_;
    geometry_msgs::msg::TwistStamped target_twist_;

    geometry_msgs::msg::PoseStamped ref_pose_;
    geometry_msgs::msg::TwistStamped ref_twist_;

  private:
    void computeRefPose();
  };

}; // namespace follow_target

#endif // FOLLOW_TARGET_HPP_
