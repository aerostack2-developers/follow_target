#ifndef FT_BASE_HPP_
#define FT_BASE_HPP_

#include "as2_core/frame_utils/frame_utils.hpp"
#include "as2_core/node.hpp"
#include "motion_reference_handlers/hover_motion.hpp"
#include "motion_reference_handlers/speed_motion.hpp"
#include <rclcpp/rclcpp.hpp>

#include "ft_speed_controller.hpp"
#include "ft_utils.hpp"

#include <Eigen/Dense>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/bool.hpp>

namespace ft_base
{
struct FTBaseStruct
{
    as2::Node *node_ptr;
    std::shared_ptr<as2::motionReferenceHandlers::SpeedMotion> motion_handler_speed;
    std::shared_ptr<as2::motionReferenceHandlers::HoverMotion> motion_handler_hover;
    std::shared_ptr<ft_speed_controller::SpeedController> controller_handler;
    std::shared_ptr<geometry_msgs::msg::PoseStamped> sl_pose;
    std::shared_ptr<geometry_msgs::msg::TwistStamped> sl_twist;
    std::shared_ptr<geometry_msgs::msg::PoseStamped> target_pose;
    std::shared_ptr<Eigen::Vector3d> speed_limit;
};

class FollowTargetBase
{
  public:
    FollowTargetBase(FTBaseStruct tf_base_struct);
    ~FollowTargetBase(){};

  protected:
    as2::Node *node_ptr_;
    std::shared_ptr<as2::motionReferenceHandlers::SpeedMotion> motion_handler_speed_;
    std::shared_ptr<as2::motionReferenceHandlers::HoverMotion> motion_handler_hover_;
    std::shared_ptr<ft_speed_controller::SpeedController> controller_handler_;
    std::shared_ptr<geometry_msgs::msg::PoseStamped> sl_pose_;
    std::shared_ptr<geometry_msgs::msg::TwistStamped> sl_twist_;
    std::shared_ptr<geometry_msgs::msg::PoseStamped> target_pose_;
    std::shared_ptr<Eigen::Vector3d> speed_limit_;

    geometry_msgs::msg::Pose reference_pose_;
    geometry_msgs::msg::Twist target_twist_;
    float target_mean_height = 0.0f;
    bool proportional_limitation_ = false;

  private:
    bool first_run = true;
    geometry_msgs::msg::Pose last_target_pose_;

    float target_twist_alpha_ = 1.0f;
    float target_pose_predict_factor_ = 0.0f;
    float target_height_alpha_ = 1.0f;
    std::vector<std::string> base_parameters = {"base.proportional_limitation", "base.target_twist_alpha",
                                                "base.target_pose_predict_factor", "base.target_height_alpha"};

  public:
    bool finished = false;

  public:
    void declareParameters();
    void updateParam(const rclcpp::Parameter param);
    void run(const double &dt);
    virtual void resetState() = 0;

  protected:
    virtual void ownDeclareParameters(){};
    virtual void ownUpdateParam(const rclcpp::Parameter param){};
    virtual void ownRun(const double &dt) = 0;

    Eigen::Vector3d computeControl(const double &dt, const Eigen::Vector3d &speed_limit,
                                   const bool &proportional_limitation);

    double getPathFacingAngle(const double &dt);
    double computeYawControl(const double &dt, const double &desired_yaw);
    double computeYawDiff(const double &desired_yaw, const double &current_yaw);

    Eigen::Vector2d computeSpeedDif2d(const geometry_msgs::msg::Twist &v0, const geometry_msgs::msg::Twist &v1);
    Eigen::Vector3d computeSpeedDif3d(const geometry_msgs::msg::Twist &v0, const geometry_msgs::msg::Twist &v1);
    Eigen::Vector2d computeRelativeSpeedTargetUav2d();
    Eigen::Vector3d computeRelativeSpeedTargetUav3d();

  private:
    void computeTargetSpeed(const double &dt);
    void computeReference(const double &dt);
    void computeTargetMeanHeight(const double &dt);
};
}; // namespace ft_base

#endif // FT_BASE_HPP_
