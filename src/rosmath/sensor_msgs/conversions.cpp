#include "rosmath/sensor_msgs/conversions.h"
#include "rosmath/sensor_msgs/misc.h"

namespace rosmath {

void convert(const sensor_msgs::LaserScan& from,
             sensor_msgs::PointCloud& to)
{
    to.header = from.header;

    if(from.intensities.size() > 0)
    {
        sensor_msgs::ChannelFloat32 intensities;
        intensities.name = POINTCLOUD_INTENSITY;
        for(size_t i = 0; i < from.ranges.size(); i++)
        {
            // Skip outliers
            if(from.ranges[i] < from.range_min || from.ranges[i] >= from.range_max)
            {
                continue;
            }
            geometry_msgs::Point32 p;
            p.x = from.ranges[i] * cos(from.angle_min + i * from.angle_increment);
            p.y = from.ranges[i] * sin(from.angle_min + i * from.angle_increment);
            p.z = 0.0;
            to.points.push_back(p);
            intensities.values.push_back(from.intensities[i]);
        }
        setChannel(intensities, to);
    } else {
        for(size_t i = 0; i < from.ranges.size(); i++)
        {
            // Skip outliers
            if(from.ranges[i] < from.range_min || from.ranges[i] >= from.range_max)
            {
                continue;
            }
            geometry_msgs::Point32 p;
            p.x = from.ranges[i] * cos(from.angle_min + i * from.angle_increment);
            p.y = from.ranges[i] * sin(from.angle_min + i * from.angle_increment);
            p.z = 0.0;
            to.points.push_back(p);
        }
    }
}

sensor_msgs::PointCloud& operator<<=(
    sensor_msgs::PointCloud& to,
    const sensor_msgs::LaserScan& from)
{
    convert(from, to);
    return to;
}

} // namespace rosmath