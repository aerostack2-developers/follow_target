#ifndef __FT_UTILS_HPP__
#define __FT_UTILS_HPP__

#include <Eigen/Dense>
#include <math.h>

namespace ft_utils
{
    using Vector3d = Eigen::Vector3d;

    double computeDistance1D(const double &x0, const double &x1);

    double computeDistance2D(const double &x0, const double &y0, const double &x1, const double &y1);

    double computeDistance3D(const Vector3d &_pose0, const Vector3d &_pose1);

    double computeModule(const Vector3d &_v);
    
} // namespace ft_utils

#endif // __FT_UTILS_H__

