#ifndef ROSMATH_SENSOR_MSGS_MATH_H
#define ROSMATH_SENSOR_MSGS_MATH_H

// TODO: sensor_msgs
#include <sensor_msgs/PointCloud.h>

// internal deps
#include "rosmath/math.h"

namespace rosmath {

sensor_msgs::PointCloud mult(
    const geometry_msgs::TransformStamped& T,
    const sensor_msgs::PointCloud& pcl);

sensor_msgs::PointCloud operator*(
    const geometry_msgs::TransformStamped& T,
    const sensor_msgs::PointCloud& pcl);


} // namespace rosmath

#endif // ROSMATH_SENSOR_MSGS_MATH_H