#include <ros/ros.h>
#include <rosmath/rosmath.h>
#include <iostream>

#include <rosmath/template.h>

using namespace rosmath;

bool testSensorMsgs()
{
    bool ret = true;

    sensor_msgs::PointCloud pcl;
    geometry_msgs::TransformStamped T;
    pcl.points.resize(100);
    std::vector<geometry_msgs::Vector3> normals(100);
    setNormals(normals, pcl);

    mult(T, pcl);
    T * pcl;

    return true;
}

bool testNavMsgs()
{
    bool ret = true;

    geometry_msgs::TransformStamped Ts;
    nav_msgs::Path path;
    path.poses.resize(100);

    mult(Ts, path);
    Ts * path;


    return ret;
}

bool testExtendedMath()
{
    bool ret = true;
    
    geometry_msgs::Transform T;
    std::vector<geometry_msgs::Pose> p1(100);
    std::vector<geometry_msgs::Point> p2(100);

    mult(T, p1);
    mult(T, p2);
    T * p1;
    T * p2;

    geometry_msgs::TransformStamped Ts;
    std::vector<geometry_msgs::PoseStamped> p1s(100);
    std::vector<geometry_msgs::PointStamped> p2s(100);

    mult(Ts, p1s);
    mult(Ts, p2s);
    Ts * p1s;
    Ts * p2s;


    geometry_msgs::PoseWithCovarianceStamped p3;
    p3.pose.pose = p1[0];
    
    mult(Ts, p3);
    Ts * p3;
    


    return ret;
}

bool testGeneralMath()
{
    bool ret = true;

    // multiply, divide
    geometry_msgs::Point p;
    p.x = 1.0;
    p.y = 2.0;
    p.z = 3.0;
    p = p * 2.0;
    p /= 2.0;
    p *= 2.0;
    p = p / 2.0;
    if(p.x != 1.0 || p.y != 2.0 || p.z != 3.0)
    {
        ROS_WARN_STREAM("error: multiply, divide\n" << p);
        ret = false;
    }

    

    // add, subtract
    p = p + 2.0;
    p -= 2.0;
    p += 2.0;
    p = p - 2.0;
    if(p.x != 1.0 || p.y != 2.0 || p.z != 3.0)
    {
        ROS_WARN_STREAM("error: add/sub point and scalar\n" << p);
        ret = false;
    }

    geometry_msgs::Point p2;
    p2.x = 4.0;
    p2.y = 6.0;
    p2.z = 8.0;

    p = p - p2;
    p += p2;
    p -= p2;
    p = p + p2;
    if(p.x != 1.0 || p.y != 2.0 || p.z != 3.0)
    {
        ROS_WARN_STREAM("error: add/sub point and point\n" << p);
        ret = false;
    }

    std::vector<double> covdata(36);
    for(int i=0; i<36; i++)
    {
        covdata[i] = i;
    }
    Eigen::MatrixXd cov;
    convert(covdata, cov);

    // ROS_INFO_STREAM(cov.rows() << "x" << cov.cols() << "\n" << cov );


    return ret;
}

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

    // x=0, y=0.5, z=0.0, w=0.5
    Eigen::Quaterniond q(0.5, 0.0, 0.5, 0.0);
    q.normalize();
    geometry_msgs::Quaternion q2;
    q2 <<= q;
    Eigen::Quaterniond qinv = q.inverse();
    auto q2inv = ~q2;
    
    T * ~T;

    return ret;
}

bool testTransform()
{
    std::cout << EIGEN_WORLD_VERSION << "." << EIGEN_MAJOR_VERSION << "." << EIGEN_MINOR_VERSION << std::endl;

    geometry_msgs::TransformStamped T_map_base, T_base_scanner;
    geometry_msgs::TransformStamped T_map_scanner;

    T_map_base.header.frame_id = "map";
    T_map_base.child_frame_id = "base_footprint";
    //     transform: 
    //   translation: 
    //     x: -3.94529
    //     y: -4.36069
    //     z: 0
    //   rotation: 
    //     x: 0
    //     y: 0
    //     z: -0.490763
    //     w: 0.871293
    T_map_base.transform.translation.x = -3.94529;
    T_map_base.transform.translation.y = -4.36069;
    T_map_base.transform.translation.z = 0.0;
    T_map_base.transform.rotation.x = 0.0;
    T_map_base.transform.rotation.y = 0.0;
    T_map_base.transform.rotation.z = -0.490763;
    T_map_base.transform.rotation.w = 0.871293;

    
    T_base_scanner.header.frame_id = "base_footprint";
    T_base_scanner.child_frame_id = "velodyne";
    // transform: 
    //   translation: 
    //     x: 0
    //     y: 0
    //     z: 0.6027
    //   rotation: 
    //     x: 0
    //     y: 0
    //     z: 0
    //     w: 1

    T_base_scanner.transform.translation.x = 0.0;
    T_base_scanner.transform.translation.y = 0.0;
    T_base_scanner.transform.translation.z = 0.6027;
    T_base_scanner.transform.rotation.x = 0.0;
    T_base_scanner.transform.rotation.y = 0.0;
    T_base_scanner.transform.rotation.z = 0.0;
    T_base_scanner.transform.rotation.w = 1.0;


    // RES

    //     transform: 
    //   translation: 
    //     x: -3.94529
    //     y: -4.36069
    //     z: 0.6027
    //   rotation: 
    //     x: 0
    //     y: 0
    //     z: 0
    //     w: 0

    // ROS_INFO_STREAM("T_map_base: " << T_map_base);
    // ROS_INFO_STREAM("T_base_scanner: " << T_base_scanner);

    T_map_scanner = T_map_base * T_base_scanner;
    // ROS_INFO_STREAM("T_map_scanner: " << T_map_scanner);
    


    return true;
}

bool testStamped()
{
    bool ret = true;

    geometry_msgs::TransformStamped A, B, C;
    
    A.header.frame_id = "map";
    A.child_frame_id = "base_link";
    A.transform.translation.x = 5.0;
    A.transform.translation.y = 2.0;
    identity(A.transform.rotation);
    
    B.header.frame_id = "base_link";
    B.child_frame_id = "camera";
    B.transform.translation.z = 1.0;
    identity(B.transform.rotation);

    C.header.frame_id = "camera";
    C.child_frame_id = "camera_optical";
    C.transform.rotation = ros2optical();
    
    auto D = A * B * C;

    if(D.header.frame_id != "map" || D.child_frame_id != "camera_optical")
    {
        ROS_WARN_STREAM("ERROR. transform stamped: frame ids");
        ret = false;
    }

    bool exception_throwed = false;
    try{
        C * B; // frame mismatch
    } catch(TransformException ex) {
        exception_throwed = true;
    }
    ret &= exception_throwed;

    ~C * ~B;
    if(B.header.frame_id != (~~B).header.frame_id )
    {
        ret = false;
    }

    // Point stamped
    geometry_msgs::PointStamped p;
    p.header.frame_id = "camera_optical";
    p.point.x = 2.0;

    auto p2 = D * p;
    auto p3 = ~D * p2;


    if(!equal(p3, p))
    {
        ROS_WARN_STREAM("transform error. p and p3 different:\n" << p << "\n" << p3);
        
        std::cout << 2.0*std::numeric_limits<double>::epsilon() << std::endl;
        ret = false;
    }

    exception_throwed = false;
    try{
        ~D * p;
    } catch(TransformException ex) {
        exception_throwed = true;
    }
    ret &= exception_throwed;

    // PoseStamped
    geometry_msgs::PoseStamped pose_cam;
    pose_cam.header.frame_id = "camera_optical";
    pose_cam.pose.position.y = 5.0;
    identity(pose_cam.pose.orientation);

    auto pose_map = D * pose_cam;

    exception_throwed = false;
    try {
        ~D * pose_cam;
    } catch(TransformException ex) {
        exception_throwed = true;
    }
    ret &= exception_throwed;

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

    test("General Math", testGeneralMath);
    test("Extended Math", testExtendedMath);
    test("Transformation", testTransform);
    test("Point Transformation", testTransformPoint);
    test("Stamped Transformation", testStamped);
    test("nav_msgs", testNavMsgs);

    return 0;
}