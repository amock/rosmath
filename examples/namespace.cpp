#include <ros/ros.h>
#include <rosmath/rosmath.h>
#include <rosmath/usingoperators.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rosmath_example_namespace");

    // transformations
    geometry_msgs::Point p;
    geometry_msgs::Transform T;
    auto p_transformed = T * p;

    geometry_msgs::Quaternion q;
    auto p_rotated = q * p;

    // norm
    double norm = rosmath::norm(p_transformed);
    double sum = rosmath::sum(p_transformed);

    // inverse
    auto Tinv = ~T;

    return 0;
}