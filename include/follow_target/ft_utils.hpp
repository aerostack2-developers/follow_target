#ifndef __FT_UTILS_HPP__
#define __FT_UTILS_HPP__

#include "as2_core/node.hpp"
#include <Eigen/Dense>
#include <math.h>
#include <rclcpp/rclcpp.hpp>

namespace ft_utils
{
using Vector3d = Eigen::Vector3d;

double computeDistance1D(const double &x0, const double &x1);

double computeDistance2D(const double &x0, const double &y0, const double &x1, const double &y1);

double computeDistance3D(const Vector3d &_pose0, const Vector3d &_pose1);

void declareParameters(as2::Node *_node, const std::string _parameters);

void declareParameters(as2::Node *_node, const std::string _parameters, const std::string _parameters_default);

void declareParameters(as2::Node *_node, const std::string _parameters, const double _parameters_default);

// void declareParameters(as2::Node *_node, const std::string _parameters,
//                        const bool _parameters_default);

} // namespace ft_utils

#endif // __FT_UTILS_H__
