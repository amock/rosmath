#include "Eigen/Dense"
#include "rosmath/random.h"

#include "rosmath/eigen/conversions.h"

namespace rosmath {

double rand_range(double min, double max)
{
    return ((max - min) * ((double)rand() / RAND_MAX)) + min;
}

double rand_angle()
{
    return rand_range(-M_PI, M_PI);
}

geometry_msgs::Point rand_point(
    const geometry_msgs::Point& pmin, 
    const geometry_msgs::Point& pmax) 
{
    geometry_msgs::Point p;
    p.x = rand_range(pmin.x, pmax.x);
    p.y = rand_range(pmin.y, pmax.y);
    p.z = rand_range(pmin.z, pmax.z);
    return p;
}

geometry_msgs::Quaternion rand_quat()
{
    geometry_msgs::Quaternion ret;
    double u = rand_range(0.0, 1.0);
    double v = rand_range(0.0, 1.0);
    double w = rand_range(0.0, 1.0);

    ret.x = sqrt(1-u) * sin(2*M_PI * v);
    ret.y = sqrt(1-u) * cos(2*M_PI * v);
    ret.z = sqrt(u) * sin(2*M_PI * w);
    ret.w = sqrt(u) * cos(2*M_PI * w);

    return ret;
}

geometry_msgs::Quaternion rand_quat(const geometry_msgs::Vector3& axis)
{
    Eigen::Vector3d axis_eig;
    axis_eig <<= axis;
    Eigen::Quaterniond q_eig;
    q_eig = Eigen::AngleAxisd(rand_angle(), axis_eig);
    geometry_msgs::Quaternion q;
    q <<= q_eig;
    return q;
}

} // namespace rosmath