#ifndef ROSMATH_MISC_H
#define ROSMATH_MISC_H

#include "rosmath.h"

namespace rosmath {


geometry_msgs::Point RPY(   const double& roll,
                            const double& pitch,
                            const double& yaw);

geometry_msgs::Quaternion rpy2quat(const double& roll, 
                                   const double& pitch,
                                   const double& yaw);
geometry_msgs::Quaternion rpy2quat(const geometry_msgs::Point& rpy);

geometry_msgs::Point quat2rpy(const geometry_msgs::Quaternion& q);

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