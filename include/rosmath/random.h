#ifndef ROSMATH_RANDOM_H
#define ROSMATH_RANDOM_H

#include "math.h"

namespace rosmath {

double rand_range(double min, double max);

double rand_angle();

geometry_msgs::Point rand_point(
    const geometry_msgs::Point& pmin, 
    const geometry_msgs::Point& pmax);

geometry_msgs::Quaternion rand_quat();
geometry_msgs::Quaternion rand_quat(const geometry_msgs::Vector3& axis);

} // namespace rosmath

#endif // ROSMATH_RANDOM_H