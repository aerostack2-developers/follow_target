#ifndef FT_PICKUP_HPP_
#define FT_PICKUP_HPP_

#include "as2_core/names/topics.hpp"
#include <as2_core/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include "ft_base.hpp"
#include "ft_speed_controller.hpp"
#include "ft_utils.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/bool.hpp>

namespace ft_pickup
{
using SpeedController = ft_speed_controller::SpeedController;

class PickUp : public ft_base::FollowTargetBase
{
  public:
    PickUp(ft_base::FTBaseStruct tf_base_struct);
    ~PickUp(){};

  public:
    /* Subscribers */
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gripper_sensor_sub_;
    void gripperSensorCallback(const std::shared_ptr<std_msgs::msg::Bool> _msg);

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gripper_actuator_sub_;
    void gripperActuatorCallback(const std::shared_ptr<std_msgs::msg::Bool> _msg);

    /* Publishers */
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gripper_actuator_pub_;
    void publishGripper(const bool &_state);

    void ownResetState() override;

  private:
    bool gripper_contact_ = false;
    bool gripper_actuator_ = false;
    bool object_gripped_ = false;

    int8_t current_phase_ = 0;

    float pickup_approach_2D_threshold_ = 0.5f;
    float pickup_approach_height_threshold_ = 0.5f;
    float pickup_approach_speed_threshold_ = 0.5f;
    float gripper_height_ = 0.0f;
    float vessel_height_ = 10.0f;

    std::vector<std::string> pickup_parameters = {
        "pickup.pickup_approach_2D_threshold", "pickup.pickup_approach_height_threshold",
        "pickup.pickup_approach_speed_threshold", "pickup.gripper_height", "pickup.vessel_height"};

    Eigen::Vector3d pickup_position_;
    Eigen::Vector3d motion_speed_ = Eigen::Vector3d::Zero();

  protected:
    void ownRun(const double &dt) override;
    void ownDeclareParameters() override;
    void ownUpdateParam(const rclcpp::Parameter param) override;

  private:
    void checkGripperContact();
    void computeGripperTransform();
};

}; // namespace ft_pickup

#endif // FT_PICKUP_HPP_
