#include "rosmath/misc.h"
#include <Eigen/Dense>


namespace rosmath {

geometry_msgs::Point RPY(   const double& roll,
                            const double& pitch,
                            const double& yaw)
{
    geometry_msgs::Point rpy;
    rpy.x = roll;
    rpy.y = pitch;
    rpy.z = yaw;
    return rpy;
}

geometry_msgs::Quaternion rpy2quat(const geometry_msgs::Point& rpy)
{
    return rpy2quat(rpy.x, rpy.y, rpy.z);
}

geometry_msgs::Quaternion rpy2quat(const double& roll, 
                                   const double& pitch,
                                   const double& yaw)
{
    Eigen::Quaterniond qeig;
    qeig = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
        * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
    geometry_msgs::Quaternion q;
    q <<= qeig;
    return q;
}

geometry_msgs::Point quat2rpy(const geometry_msgs::Quaternion& q)
{
    Eigen::Quaterniond qeig;
    qeig <<= q;
    Eigen::Vector3d euler = qeig.toRotationMatrix().eulerAngles(0, 1, 2);
    geometry_msgs::Point p;
    p <<= euler;
    return p;
}

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