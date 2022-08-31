#ifndef ft_DYNAMIC_FOLLOW_HPP_
#define ft_DYNAMIC_FOLLOW_HPP_

#include "as2_core/names/topics.hpp"
#include <as2_core/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include "ft_base.hpp"
#include "ft_speed_controller.hpp"
#include "ft_utils.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/bool.hpp>

namespace ft_dynamic_follow
{
using SpeedController = ft_speed_controller::SpeedController;

class DynamicFollow : public ft_base::FollowTargetBase
{
  public:
    DynamicFollow(ft_base::FTBaseStruct tf_base_struct);
    ~DynamicFollow(){};

  public:
    void ownResetState() override;

  private:
    int8_t current_phase_ = 0;

    std::vector<std::string> dynamic_follow_parameters = {};

    Eigen::Vector3d dynamic_follow_position_;
    Eigen::Vector3d motion_speed_ = Eigen::Vector3d::Zero();

  protected:
    void ownRun(const double &dt) override;
    void ownDeclareParameters() override;
    void ownUpdateParam(const rclcpp::Parameter param) override;
};

}; // namespace ft_dynamic_follow

#endif // ft_DYNAMIC_FOLLOW_HPP_
