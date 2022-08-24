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

double computeModule(const Vector3d &_v)
{
    return sqrt(pow(_v(0), 2) + pow(_v(1), 2) + pow(_v(2), 2));
};

} // namespace ft_utils