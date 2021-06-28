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


    // Stamped Transformations
    geometry_msgs::PointStamped P;
    P.header.frame_id = "sensor";
    P.point.x = 10.0;
    P.point.y = 0.0;
    P.point.z = 0.0;

    // Sensor -> Base
    geometry_msgs::TransformStamped T_sensor_base;
    T_sensor_base.header.frame_id = "sensor";
    T_sensor_base.child_frame_id = "base";
    identity(T_sensor_base.transform);
    T_sensor_base.transform.translation.x = 2.0;

    // Base -> Sensor
    geometry_msgs::TransformStamped T_base_map;
    T_base_map.header.frame_id = "base";
    T_base_map.child_frame_id = "map";
    identity(T_base_map.transform);
    T_base_map.transform.translation.x = 1.0;



    auto T_sensor_map = T_base_map * T_sensor_base;
    std::cout << T_sensor_map << std::endl;


    return 0;
}
