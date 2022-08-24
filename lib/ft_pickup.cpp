#include "ft_pickup.hpp"

namespace ft_pickup
{
    PickUp::PickUp(ft_base::FTBaseStruct tf_base_struct) : ft_base::FollowTargetBase(tf_base_struct)
    {
        RCLCPP_INFO(node_ptr_->get_logger(), "PickUp()");
        /* Subscribers */
        node_ptr_->declare_parameter("gripper_state_topic", "/gripper_state_topic");
        std::string gripper_state_topic = node_ptr_->get_parameter("gripper_state_topic").as_string();
        gripper_sensor_sub_ = node_ptr_->create_subscription<std_msgs::msg::Bool>(
            gripper_state_topic,
            as2_names::topics::sensor_measurements::qos,
            std::bind(&PickUp::gripperSensorCallback, this, std::placeholders::_1));

        node_ptr_->declare_parameter("gripper_actuator_topic", "/gripper_actuator_topic");
        std::string gripper_actuator_topic = node_ptr_->get_parameter("gripper_actuator_topic").as_string();
        gripper_actuator_sub_ = node_ptr_->create_subscription<std_msgs::msg::Bool>(
            gripper_actuator_topic,
            as2_names::topics::sensor_measurements::qos,
            std::bind(&PickUp::gripperActuatorCallback, this, std::placeholders::_1));

        /* Publishers */
        gripper_actuator_pub_ = node_ptr_->create_publisher<std_msgs::msg::Bool>(
            gripper_actuator_topic,
            rclcpp::QoS(10)); // TODO: rclcpp::SensorDataQoS()
    };

    void PickUp::gripperSensorCallback(const std::shared_ptr<std_msgs::msg::Bool> _msg)
    {
        gripper_contact_ = _msg->data;
        return;
    };

    void PickUp::gripperActuatorCallback(const std::shared_ptr<std_msgs::msg::Bool> _msg)
    {
        gripper_actuator_ = _msg->data;
        return;
    };

    void PickUp::publishGripper(const bool &_state)
    {
        if (gripper_actuator_ == _state)
        {
            return;
        }
        std_msgs::msg::Bool msg;
        msg.data = _state;
        gripper_actuator_pub_->publish(msg);
        return;
    };

    void PickUp::checkGripperContact()
    {
        static rclcpp::Time time_0 = rclcpp::Clock().now();
        rclcpp::Time time_1 = rclcpp::Clock().now();
        if (gripper_contact_ && gripper_actuator_)
        {
            if ((time_1 - time_0).seconds() > 0.5)
            {
                RCLCPP_INFO_ONCE(node_ptr_->get_logger(), "Object grasped");
                object_gripped_ = true;
            }
            return;
        }
        object_gripped_ = false;
        time_0 = time_1;
        return;
    };

    void PickUp::run(double dt)
    {
        checkGripperContact();
        switch (current_phase_)
        {
        case 0:
        {
            // RCLCPP_INFO(this->get_logger(), "Pickup phase 0: Approach to target");
            // publishGripper(false);

            // computeRefPose();
            // ref_pose_.pose.position.z += 0.2;
            // Vector3d motion_speed = computeControl(dt);
            // motion_handler_speed_->sendSpeedCommandWithYawSpeed(motion_speed.x(), motion_speed.y(), motion_speed.z(), 0.0);

            // // TODO: Speed should be relative to the target_pose_
            // float threshold = 0.1f;
            // if (ft_utils::computeDistance2D(sl_pose_.pose.position.x, sl_pose_.pose.position.y, target_pose_.pose.position.x, target_pose_.pose.position.y) < threshold &&
            //     ft_utils::computeDistance1D(sl_pose_.pose.position.z, ref_pose_.pose.position.z) < threshold &&
            //     ft_utils::computeModule(motion_speed) < 0.75f)
            // {
            //     current_phase_++;
            //     motion_handler_speed_->sendSpeedCommandWithYawSpeed(motion_speed.x(), motion_speed.y(), -0.5f, 0.0);
            // }
            break;
        }
        case 1:
        {
            // RCLCPP_INFO(this->get_logger(), "Pickup phase 1: Pickup");
            // publishGripper(true);
            // // ref_pose_ = target_pose_;
            // computeRefPose();
            // ref_pose_.pose.position.z = target_pose_.pose.position.z;
            // Vector3d motion_speed = computeControl(dt);
            // motion_handler_speed_->sendSpeedCommandWithYawSpeed(motion_speed.x(), motion_speed.y(), -0.2f, 0.0);

            // if (object_gripped_)
            // {
            //     pickup_position = Vector3d(target_pose_.pose.position.x, target_pose_.pose.position.y, target_pose_.pose.position.z);
            //     current_phase_++;
            // }
            break;
        }
        case 2:
        {
            // if (!gripper_contact_)
            // {
            //     current_phase_ = 0;
            // }
            // RCLCPP_INFO(this->get_logger(), "Pickup phase 2: Hold object");
            // publishGripper(true);
            // ref_pose_.pose.position.x = pickup_position.x();
            // ref_pose_.pose.position.y = pickup_position.y();
            // ref_pose_.pose.position.z = pickup_position.z() + 1.0;
            // // RCLCPP_INFO(this->get_logger(), "Go to pickup position: %f, %f, %f", ref_pose_.pose.position.x, ref_pose_.pose.position.y, ref_pose_.pose.position.z);
            // Vector3d motion_speed = computeControl(dt);
            // // motion_handler_speed_->sendSpeedCommandWithYawSpeed(motion_speed.x(), motion_speed.y(), motion_speed.z(), 0.0);
            // // if (computeDistance1D(sl_pose_.pose.position.z, ref_pose_.pose.position.z) < 0.1)
            // motion_handler_speed_->sendSpeedCommandWithYawSpeed(0.0f, 0.0f, 1.0f, 0.0);
            // if (sl_pose_.pose.position.z > ref_pose_.pose.position.z)
            // {
            //     resetState();
            //     return;
            // }
            break;
        }
        default:
            break;
        }
        return;
    };
}