#ifndef ROSMATH_SENSOR_MSGS_MISC_H
#define ROSMATH_SENSOR_MSGS_MISC_H

#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Vector3.h>
#include <string>
#include <vector>

namespace rosmath {

constexpr char POINTCLOUD_NORMAL_X[] = "nx";
constexpr char POINTCLOUD_NORMAL_Y[] = "ny";
constexpr char POINTCLOUD_NORMAL_Z[] = "nz";

bool hasChannel(const sensor_msgs::PointCloud& pcl,
                const std::string& name);

sensor_msgs::ChannelFloat32 getChannel(
    const sensor_msgs::PointCloud& pcl,
    const std::string& name);

void setChannel(
    const sensor_msgs::ChannelFloat32& channel,
    sensor_msgs::PointCloud& pcl);

bool hasNormals(const sensor_msgs::PointCloud& pcl);

std::vector<geometry_msgs::Vector3> getNormals(
    const sensor_msgs::PointCloud& pcl);

void setNormals(
    const std::vector<geometry_msgs::Vector3>& normals,
    sensor_msgs::PointCloud& pcl);

} // namespace rosmath

#endif // ROSMATH_SENSOR_MSGS_MISC_H