#include "rosmath/misc.h"
#include <Eigen/Dense>

namespace rosmath {

geometry_msgs::Quaternion ros2optical()
{
    return rpy2quat(-M_PI/2.0, M_PI/2.0, 0.0);
}

geometry_msgs::Quaternion ros2optical(
    const geometry_msgs::Quaternion& qros)
{
    return qros * ros2optical();
}

geometry_msgs::Transform ros2optical(
    const geometry_msgs::Transform& Tros)
{
    geometry_msgs::Transform Topt;
    Topt.translation = Tros.translation;
    Topt.rotation = ros2optical(Tros.rotation);
    return Topt;
}

geometry_msgs::Pose ros2optical(
    const geometry_msgs::Pose& Pros)
{
    geometry_msgs::Pose Popt;
    Popt.position = Pros.position;
    Popt.orientation = ros2optical(Pros.orientation);
    return Popt;
}

geometry_msgs::Quaternion optical2ros()
{
    return rpy2quat(M_PI/2.0, 0.0, M_PI/2.0);
}

geometry_msgs::Quaternion optical2ros(
    const geometry_msgs::Quaternion& qopt)
{
    return qopt * optical2ros();
}

geometry_msgs::Transform optical2ros(
    const geometry_msgs::Transform& Topt)
{
    geometry_msgs::Transform Tros;
    Tros.translation = Topt.translation;
    Tros.rotation = optical2ros(Topt.rotation);
    return Tros;
}

geometry_msgs::Pose optical2ros(
    const geometry_msgs::Pose& Popt)
{
    geometry_msgs::Pose Pros;
    Pros.position = Popt.position;
    Pros.orientation = optical2ros(Popt.orientation);
    return Pros;
}

} // namespace rosmath