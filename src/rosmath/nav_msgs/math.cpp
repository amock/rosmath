#include "rosmath/nav_msgs/math.h"
#include "rosmath/exceptions.h"

namespace rosmath {

nav_msgs::Path mult(
    const geometry_msgs::TransformStamped& T,
    const nav_msgs::Path& p)
{
    nav_msgs::Path ret;

    if(T.child_frame_id != p.header.frame_id)
    {
        throw TransformException(
            "\nCould not do transformation T{" + T.child_frame_id + "->" + T.header.frame_id 
            + "} * path{" + p.header.frame_id 
            + "}\nrequired: path{B} = T{A->B} * path{A}\n"
            + "mismatched frames: " + T.child_frame_id + " != " + p.header.frame_id
            );
    }

    ret.header.frame_id = T.header.frame_id;
    ret.header.stamp = p.header.stamp;

    ret.poses = T * p.poses;

    return ret;
}

nav_msgs::Path operator*(
    const geometry_msgs::TransformStamped& T,
    const nav_msgs::Path& p)
{
    return mult(T, p);
}

} // namespace rosmath