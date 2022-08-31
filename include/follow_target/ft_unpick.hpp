#ifndef ft_UNPICK_HPP_
#define ft_UNPICK_HPP_

#include "as2_core/names/topics.hpp"
#include <as2_core/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include "ft_base.hpp"
#include "ft_speed_controller.hpp"
#include "ft_utils.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/bool.hpp>

namespace ft_unpick
{
using SpeedController = ft_speed_controller::SpeedController;

class UnPick : public ft_base::FollowTargetBase
{
  public:
    UnPick(ft_base::FTBaseStruct tf_base_struct);
    ~UnPick(){};

  public:
    /* Subscribers */
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gripper_sensor_sub_;
    void gripperSensorCallback(const std::shared_ptr<std_msgs::msg::Bool> _msg);

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gripper_actuator_sub_;
    void gripperActuatorCallback(const std::shared_ptr<std_msgs::msg::Bool> _msg);

    /* Publishers */
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gripper_actuator_pub_;
    void publishGripper(const bool &_state);

    void ownResetState();

  private:
    bool gripper_contact_ = true;
    bool gripper_actuator_ = true;
    bool object_gripped_ = true;

    int8_t current_phase_;

    float unpick_approach_2D_threshold_ = 0.5f;
    float unpick_approach_height_threshold_ = 0.5f;
    float unpick_approach_speed_threshold_ = 0.5f;
    float gripper_height_ = 0.0f;
    float vessel_height_ = 10.0f;
    float object_height_ = 0.0f;
    float delivery_height_ = 1.0f;

    std::vector<std::string> unpick_parameters = {"unpick.unpick_approach_2D_threshold",
                                                  "unpick.unpick_approach_height_threshold",
                                                  "unpick.unpick_approach_speed_threshold",
                                                  "unpick.gripper_height",
                                                  "unpick.vessel_height",
                                                  "unpick.object_height",
                                                  "unpick.delivery_height"};

    Eigen::Vector3d unpick_position_;
    Eigen::Vector3d motion_speed_ = Eigen::Vector3d::Zero();

  protected:
    void ownRun(const double &dt) override;
    void ownDeclareParameters() override;
    void ownUpdateParam(const rclcpp::Parameter param) override;

  private:
    void checkGripperContact();
};

}; // namespace ft_unpick

#endif // ft_UNPICK_HPP_
