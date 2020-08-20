#include <ros/ros.h>
#include <rosmath/rosmath.h>
#include <iostream>

using namespace rosmath;

bool testInternal()
{
    bool ret = true;

    geometry_msgs::Point p1;
    geometry_msgs::Point32 p2;
    geometry_msgs::Vector3 p3;
    geometry_msgs::Quaternion q;
    geometry_msgs::Transform T;
    geometry_msgs::Pose pose;

    p1 <<= p2;
    p2 <<= p1;
    p2 <<= p3;
    p3 <<= p2;
    p1 <<= p3;
    p3 <<= p1;

    p1 <<= p1;
    p2 <<= p2;
    p3 <<= p3;

    q <<= q;
    pose <<= T;
    T <<= pose;
    T <<= T;
    pose <<= pose;

    return ret;
}

bool testPointEigen()
{
    bool ret = true;

    // test values
    double xt = 10.0;
    double yt = 5.0;
    double zt = 1.0;

    // ros -> eigen
    {
        Eigen::Vector3d p_eigen;
        geometry_msgs::Point p_msg;
        
        p_msg.x = xt;
        p_msg.y = yt;
        p_msg.z = zt;

        convert(p_msg, p_eigen);

        ret &= p_eigen.x() == xt;
        ret &= p_eigen.y() == yt;
        ret &= p_eigen.z() == zt;
        ret &= p_msg.x == xt;
        ret &= p_msg.y == yt;
        ret &= p_msg.z == zt;
    }

    {
        Eigen::Vector3d p_eigen;
        geometry_msgs::Point p_msg;
        
        p_msg.x = xt;
        p_msg.y = yt;
        p_msg.z = zt;

        p_eigen <<= p_msg;

        ret &= p_eigen.x() == xt;
        ret &= p_eigen.y() == yt;
        ret &= p_eigen.z() == zt;
        ret &= p_msg.x == xt;
        ret &= p_msg.y == yt;
        ret &= p_msg.z == zt;
    }

    // eigen -> ros
    {
        Eigen::Vector3d p_eigen;
        geometry_msgs::Point p_msg;
        
        p_eigen.x() = xt;
        p_eigen.y() = yt;
        p_eigen.z() = zt;

        convert(p_eigen, p_msg);

        ret &= p_eigen.x() == xt;
        ret &= p_eigen.y() == yt;
        ret &= p_eigen.z() == zt;
        ret &= p_msg.x == xt;
        ret &= p_msg.y == yt;
        ret &= p_msg.z == zt;
    }
    {
        Eigen::Vector3d p_eigen;
        geometry_msgs::Point p_msg;
        
        p_eigen.x() = xt;
        p_eigen.y() = yt;
        p_eigen.z() = zt;

        p_msg <<= p_eigen;

        ret &= p_eigen.x() == xt;
        ret &= p_eigen.y() == yt;
        ret &= p_eigen.z() == zt;
        ret &= p_msg.x == xt;
        ret &= p_msg.y == yt;
        ret &= p_msg.z == zt;
    }

    return ret;
}


bool testTransform()
{
    bool ret = true;

    // test values
    double xt = 10.0;
    double yt = 5.0;
    double zt = 1.0;
    double qxt = 0.5;
    double qyt = 0.5;
    double qzt = 0.0;
    double qwt = 0.0;


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
    ros::init(argc, argv, "rosmath_conversion_test_node");

    std::cout << "Tests of rosmath library: Conversion" << std::endl;

    test("Internal", testInternal);
    test("Point Eigen", testPointEigen);
    test("Transform", testTransform);
    
    

    return 0;
}