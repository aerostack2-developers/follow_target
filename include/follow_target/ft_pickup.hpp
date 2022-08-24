#ifndef FT_PICKUP_HPP_
#define FT_PICKUP_HPP_

#include <rclcpp/rclcpp.hpp>
#include <as2_core/node.hpp>
#include "as2_core/names/topics.hpp"

#include "ft_speed_controller.hpp"
#include "ft_base.hpp"

#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

namespace ft_pickup
{
    using SpeedController = ft_speed_controller::SpeedController;

    class PickUp : ft_base::FollowTargetBase
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

    private:
        bool gripper_contact_;
        bool gripper_actuator_;
        bool object_gripped_;

        int8_t current_phase_;

    public:
        void test(){
            RCLCPP_INFO(node_ptr_->get_logger(), "Test");
            return;};
        void run(double dt);

    private:
        void checkGripperContact();
    };

}; // namespace ft_pickup

#endif // FT_PICKUP_HPP_
