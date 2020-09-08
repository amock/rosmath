#include "rosmath/sensor_msgs/misc.h"

namespace rosmath {

bool hasChannel(const sensor_msgs::PointCloud& pcl,
                const std::string& name)
{
    bool channel_found = false;
    // check if normals are in channels
    for(size_t i=0; i<pcl.channels.size(); i++)
    {
        if(pcl.channels[i].name == name)
        {
            channel_found = true;
            break;
        }
    }

    return channel_found;
}

sensor_msgs::ChannelFloat32 getChannel(
    const sensor_msgs::PointCloud& pcl,
    const std::string& name)
{
    for(size_t i=0; i<pcl.channels.size(); i++)
    {
        if(pcl.channels[i].name == name)
        {
            return pcl.channels[i];
        }
    }

    // TODO: better exception
    throw std::runtime_error("Channel not found");
}

void setChannel(
    const sensor_msgs::ChannelFloat32& channel,
    sensor_msgs::PointCloud& pcl)
{
    for(size_t i=0; i<pcl.channels.size(); i++)
    {
        if(pcl.channels[i].name == channel.name)
        {
            pcl.channels[i] = channel;
        }
    }
    pcl.channels.push_back(channel);
}

bool hasNormals(const sensor_msgs::PointCloud& pcl)
{
    return hasChannel(pcl, POINTCLOUD_NORMAL_X) 
        && hasChannel(pcl, POINTCLOUD_NORMAL_Y) 
        && hasChannel(pcl, POINTCLOUD_NORMAL_Z);
}

std::vector<geometry_msgs::Vector3> getNormals(
    const sensor_msgs::PointCloud& pcl)
{
    std::vector<geometry_msgs::Vector3> ret(pcl.points.size());

    const sensor_msgs::ChannelFloat32 nx = getChannel(pcl, POINTCLOUD_NORMAL_X);
    const sensor_msgs::ChannelFloat32 ny = getChannel(pcl, POINTCLOUD_NORMAL_Y);
    const sensor_msgs::ChannelFloat32 nz = getChannel(pcl, POINTCLOUD_NORMAL_Z);
    
    for(size_t i=0; i<ret.size(); i++)
    {
        ret[i].x = nx.values[i];
        ret[i].y = ny.values[i];
        ret[i].z = nz.values[i];
    }

    return ret;
}

void setNormals(
    const std::vector<geometry_msgs::Vector3>& normals,
    sensor_msgs::PointCloud& pcl)
{
    sensor_msgs::ChannelFloat32 nx;
    sensor_msgs::ChannelFloat32 ny;
    sensor_msgs::ChannelFloat32 nz;
    nx.name = POINTCLOUD_NORMAL_X;
    ny.name = POINTCLOUD_NORMAL_Y;
    nz.name = POINTCLOUD_NORMAL_Z;
    nx.values.resize(normals.size());
    ny.values.resize(normals.size());
    nz.values.resize(normals.size());

    for(size_t i=0; i<normals.size(); i++)
    {
        nx.values[i] = normals[i].x;
        ny.values[i] = normals[i].y;
        nz.values[i] = normals[i].z;
    }

    setChannel(nx, pcl);
    setChannel(ny, pcl);
    setChannel(nz, pcl);
}




} // namespace rosmath