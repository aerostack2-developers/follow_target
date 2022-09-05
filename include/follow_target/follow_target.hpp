#ifndef FOLLOW_TARGET_HPP_
#define FOLLOW_TARGET_HPP_

#include "as2_core/names/actions.hpp"
#include "as2_core/names/services.hpp"
#include "as2_core/names/topics.hpp"
#include "as2_core/node.hpp"
#include "as2_msgs/msg/follow_target_info.hpp"
#include "as2_msgs/srv/dynamic_follower.hpp"
#include "as2_msgs/srv/dynamic_land.hpp"
#include "as2_msgs/srv/package_pick_up.hpp"
#include "as2_msgs/srv/package_un_pick.hpp"
#include "motion_reference_handlers/hover_motion.hpp"
#include "motion_reference_handlers/speed_motion.hpp"
#include <rclcpp/rclcpp.hpp>

#include "ft_dynamic_follow.hpp"
#include "ft_pickup.hpp"
#include "ft_speed_controller.hpp"
#include "ft_unpick.hpp"
#include "ft_utils.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/bool.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

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
    void run();

  private:
    /* Subscribers */

    // Self localization
    std::shared_ptr<message_filters::Subscriber<geometry_msgs::msg::PoseStamped>> sl_pose_sub_;
    std::shared_ptr<message_filters::Subscriber<geometry_msgs::msg::TwistStamped>> sl_twist_sub_;
    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::msg::PoseStamped,
                                                            geometry_msgs::msg::TwistStamped>
        approximate_policy;
    std::shared_ptr<message_filters::Synchronizer<approximate_policy>> synchronizer_;
    void state_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr pose_msg,
                        const geometry_msgs::msg::TwistStamped::ConstSharedPtr twist_msg);

    // Target localizacion
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pickup_pose_sub_;
    void targetPickUpPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr _msg);

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_unpick_pose_sub_;
    void targetUnPickPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr _msg);

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_dynamic_land_pose_sub_;
    void targetDynamicLandPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr _msg);

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_dynamic_follow_pose_sub_;
    void targetDynamicFollowPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr _msg);

    /* Publishers */

    // Node info
    rclcpp::Publisher<as2_msgs::msg::FollowTargetInfo>::SharedPtr info_pub_;
    void publishInfo();

    /* Services */

    // Dynamic land
    rclcpp::Service<as2_msgs::srv::DynamicLand>::SharedPtr set_dynamic_land_srv_;
    void setDynamicLandSrvCall(const std::shared_ptr<as2_msgs::srv::DynamicLand::Request> _request,
                               std::shared_ptr<as2_msgs::srv::DynamicLand::Response> _response);

    // Package pickup
    rclcpp::Service<as2_msgs::srv::PackagePickUp>::SharedPtr set_package_pickup_srv_;
    void setPackagePickUpSrvCall(const std::shared_ptr<as2_msgs::srv::PackagePickUp::Request> _request,
                                 std::shared_ptr<as2_msgs::srv::PackagePickUp::Response> _response);

    // Package unpick
    rclcpp::Service<as2_msgs::srv::PackageUnPick>::SharedPtr set_package_unpick_srv_;
    void setPackageUnPickSrvCall(const std::shared_ptr<as2_msgs::srv::PackageUnPick::Request> _request,
                                 std::shared_ptr<as2_msgs::srv::PackageUnPick::Response> _response);

    // Dynamic follow
    rclcpp::Service<as2_msgs::srv::DynamicFollower>::SharedPtr set_dynamic_follow_srv_;
    void setDynamicFollowSrvCall(const std::shared_ptr<as2_msgs::srv::DynamicFollower::Request> _request,
                                 std::shared_ptr<as2_msgs::srv::DynamicFollower::Response> _response);

  private:
    bool is_active_;
    as2_msgs::msg::FollowTargetInfo current_state_;

    Manage_flags manage_flags_;

    std::shared_ptr<ft_pickup::PickUp> pickup_handler_;
    std::shared_ptr<ft_unpick::UnPick> unpick_handler_;
    std::shared_ptr<ft_dynamic_follow::DynamicFollow> dynamic_follow_handler_;

    std::unique_ptr<tf2_ros::Buffer> tfBuffer_;
    std::shared_ptr<tf2_ros::TransformListener> tfListener_{nullptr};

    std::shared_ptr<as2::motionReferenceHandlers::SpeedMotion> motion_handler_speed_;
    std::shared_ptr<as2::motionReferenceHandlers::HoverMotion> motion_handler_hover_;
    std::shared_ptr<ft_speed_controller::SpeedController> controller_handler_;
    std::shared_ptr<geometry_msgs::msg::PoseStamped> sl_pose_;
    std::shared_ptr<geometry_msgs::msg::TwistStamped> sl_twist_;
    std::shared_ptr<geometry_msgs::msg::PoseStamped> target_pose_;
    std::shared_ptr<Vector3d> speed_limit_;
    Vector3d speed_limit_default_;

    std::string base_frame_ = "";

    rclcpp::Time end_time_;

    std::vector<std::string> dynamic_parameters = {"speed_limit.vx", "speed_limit.vy", "speed_limit.vz"};

  private:
    void declare_parameters();
    void resetCommand();
    void resetState();
};

}; // namespace follow_target

#endif // FOLLOW_TARGET_HPP_
