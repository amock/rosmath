#ifndef ROSMATH_MISC_H
#define ROSMATH_MISC_H

#include "rosmath.h"

namespace rosmath {


geometry_msgs::Quaternion ros2optical();

geometry_msgs::Quaternion ros2optical(
    const geometry_msgs::Quaternion& qros);

geometry_msgs::Transform ros2optical(
    const geometry_msgs::Transform& Tros);

geometry_msgs::Pose ros2optical(
    const geometry_msgs::Pose& Pros);

geometry_msgs::Quaternion optical2ros();

geometry_msgs::Quaternion optical2ros(
    const geometry_msgs::Quaternion& qopt);

geometry_msgs::Transform optical2ros(
    const geometry_msgs::Transform& Topt);

geometry_msgs::Pose optical2ros(
    const geometry_msgs::Pose& Popt);

} // namespace rosmath

#endif // ROSMATH_MISC_H