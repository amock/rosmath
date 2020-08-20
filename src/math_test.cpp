#include <ros/ros.h>
#include <rosmath/rosmath.h>
#include <iostream>

using namespace rosmath;

bool testTransformPoint()
{
    bool ret = true;


    geometry_msgs::Point p;
    p.x = 10.0;
    p.y = 5.0;
    p.z = 1.0;

    geometry_msgs::Transform T;
    T.rotation.w = 1.0;
    T.translation.x = 1.0;
    geometry_msgs::Point p2 = mult(T, p);

    p2 = T * p;

    auto p3 = T.rotation * p;

    ROS_INFO_STREAM("\n" << p);
    ROS_INFO_STREAM("\n" << p2);
    ROS_INFO_STREAM("\n" << p3);

    // x=0, y=0.5, z=0.0, w=0.5
    Eigen::Quaterniond q(0.5, 0.0, 0.5, 0.0);
    q.normalize();
    geometry_msgs::Quaternion q2;
    q2 <<= q;
    Eigen::Quaterniond qinv = q.inverse();
    auto q2inv = ~q2;
    // qinv.normalize();
    std::cout << qinv.x() << " " << qinv.y() << " " << qinv.z() << " " << qinv.w() << std::endl;
    ROS_INFO_STREAM("\n" << q2inv);
    
    T * ~T;

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
    ros::init(argc, argv, "rosmath_math_test_node");

    std::cout << "Tests of rosmath library: Math" << std::endl;

    test("Point Transformation", testTransformPoint);
    

    return 0;
}