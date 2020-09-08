#include "rosmath/sensor_msgs/math.h"
#include "rosmath/sensor_msgs/misc.h"
#include "rosmath/exceptions.h"


namespace rosmath {

sensor_msgs::PointCloud mult(
    const geometry_msgs::TransformStamped& T,
    const sensor_msgs::PointCloud& pcl)
{
    sensor_msgs::PointCloud ret;

    if(T.child_frame_id != pcl.header.frame_id)
    {
        throw TransformException(
            "\nCould not do transformation T{" + T.child_frame_id + "->" + T.header.frame_id 
            + "} * path{" + pcl.header.frame_id 
            + "}\nrequired: pcl{B} = T{A->B} * pcl{A}\n"
            + "mismatched frames: " + T.child_frame_id + " != " + pcl.header.frame_id
            );
    }

    ret.header.frame_id = T.header.frame_id;
    ret.header.stamp = pcl.header.stamp;

    ret.points = T.transform * pcl.points;
    // normals in channels??
    ret.channels = pcl.channels;

    if(hasNormals(pcl))
    {
        std::vector<geometry_msgs::Vector3> normals = getNormals(pcl);
        normals = T.transform.rotation * normals;
        setNormals(normals, ret);
    }

    return ret;
}

sensor_msgs::PointCloud operator*(
    const geometry_msgs::TransformStamped& T,
    const sensor_msgs::PointCloud& pcl)
{
    return mult(T, pcl);
}

} // namespace rosmath