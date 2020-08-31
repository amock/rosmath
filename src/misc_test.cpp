#include <ros/ros.h>
#include <rosmath/rosmath.h>
#include <iostream>

using namespace rosmath;

bool testOptical()
{
    bool ret = true;

    geometry_msgs::Transform T1, T2, T_optical;
    geometry_msgs::Quaternion q = rpy2quat(0.0, 0.0, 0.0);
    T1.rotation = q;

    T_optical = ros2optical(T1);
    T2 = optical2ros(T_optical);

    ROS_INFO_STREAM("T1\n" << T1);
    ROS_INFO_STREAM("T_optical\n" << T_optical);
    ROS_INFO_STREAM("T2\n" << T2);

    return ret;
}

std::string result(bool res)
{
    if(res)
    {
        return "success";
    } else {
        return "failure";
    }
}

void test( std::string name, bool (*f)(void) )
{
    std::cout << "-- " << name << ": " << result(f()) << std::endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rosmath_misc_test_node");

    std::cout << "Tests of rosmath library: Misc" << std::endl;

    test("Test Optical", testOptical);
    
    return 0;
}