#include <ros/ros.h>
#include <rosmath/rosmath.h>

using namespace rosmath;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rosmath_example_minimal");

    // transformations
    geometry_msgs::Point p;
    geometry_msgs::Transform T;
    auto p_transformed = T * p;

    geometry_msgs::Quaternion q;
    auto p_rotated = q * p;

    // norm
    double norm_p = norm(p_transformed);

    // inverse
    auto Tinv = ~T;

    return 0;
}
