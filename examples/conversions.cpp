#include <ros/ros.h>
#include "rosmath/rosmath.h"
#include "Eigen/Dense"

using namespace rosmath;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rosmath_example_conversions");

    geometry_msgs::Point p;
    geometry_msgs::Vector3 v;
    geometry_msgs::Transform T;
    geometry_msgs::Quaternion q;
    geometry_msgs::Pose pose;
    Eigen::Affine3d Teig;
    Eigen::Quaterniond qeig;
    Eigen::Vector3d peig;

    // Convertion anything into everything
    p <<= v;
    v <<= p;
    peig <<= p;

    T <<= pose;
    pose <<= T;
    Teig <<= pose;
    Teig <<= T;
    T <<= Teig;
    
    q <<= q;
    q <<= qeig;
    qeig <<= q;

    return 0;
}
