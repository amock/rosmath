#ifndef ROSMATH_SENSOR_MSGS_CONVERSIONS_H
#define ROSMATH_SENSOR_MSGS_CONVERSIONS_H

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>

namespace rosmath {

void convert(const sensor_msgs::LaserScan& from,
             sensor_msgs::PointCloud& to);

// Operators
sensor_msgs::PointCloud& operator<<=(
    sensor_msgs::PointCloud& to,
    const sensor_msgs::LaserScan& from);

} // namespace rosmath

#endif // ROSMATH_SENSOR_MSGS_CONVERSIONS_H