#include "ft_utils.hpp"

namespace ft_utils
{
double computeDistance1D(const double &x0, const double &x1)
{
    return fabs(x0 - x1);
};

double computeDistance2D(const double &x0, const double &y0, const double &x1, const double &y1)
{
    return sqrt(pow(x0 - x1, 2) + pow(y0 - y1, 2));
};

double computeDistance3D(const Vector3d &_pose0, const Vector3d &_pose1)
{
    return sqrt(pow(_pose0(0) - _pose1(0), 2) + pow(_pose0(1) - _pose1(1), 2) + pow(_pose0(2) - _pose1(2), 2));
};

void declareParameters(as2::Node *_node, const std::string _parameters)
{
    try
    {
        _node->declare_parameter(_parameters);
    }
    catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &e)
    {
        // RCLCPP_ERROR(_node->get_logger(), "Parameter %s not declared", e.what());
        return;
    }
    return;
};

void declareParameters(as2::Node *_node, const std::string _parameters, const std::string _parameters_default)
{
    try
    {
        _node->declare_parameter(_parameters, _parameters_default);
    }
    catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &e)
    {
        // RCLCPP_ERROR(_node->get_logger(), "Parameter %s not declared", e.what());
        return;
    }
    return;
};

void declareParameters(as2::Node *_node, const std::string _parameters, const double _parameters_default)
{
    try
    {
        _node->declare_parameter(_parameters, _parameters_default);
    }
    catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &e)
    {
        // RCLCPP_ERROR(_node->get_logger(), "Parameter %s not declared", e.what());
        return;
    }
    return;
};

// void declareParameters(as2::Node *_node, const std::string _parameters, const bool _parameters_default)
// {
//     try
//     {
//         _node->declare_parameter(_parameters, _parameters_default);
//     }
//     catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &e)
//     {
//         // RCLCPP_ERROR(_node->get_logger(), "Parameter %s not declared", e.what());
//         return;
//     }
//     return;
// };

} // namespace ft_utils
