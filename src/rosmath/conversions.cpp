#include "rosmath/conversions.h"

namespace rosmath {

// POINTS

// ros only
void convert(   const geometry_msgs::Point& from,
                geometry_msgs::Vector3& to)
{
    to.x = from.x;
    to.y = from.y;
    to.z = from.z;
}

void convert(   const geometry_msgs::Vector3& from,
                geometry_msgs::Point& to)
{
    to.x = from.x;
    to.y = from.y;
    to.z = from.z;
}

void convert(   const geometry_msgs::Point& from,
                geometry_msgs::Point32& to)
{
    to.x = from.x;
    to.y = from.y;
    to.z = from.z;
}

void convert(   const geometry_msgs::Point32& from,
                geometry_msgs::Point& to)
{
    to.x = from.x;
    to.y = from.y;
    to.z = from.z;
}

void convert(   const geometry_msgs::Vector3& from,
                geometry_msgs::Point32& to)
{
    to.x = from.x;
    to.y = from.y;
    to.z = from.z;
}

void convert(   const geometry_msgs::Point32& from,
                geometry_msgs::Vector3& to)
{
    to.x = from.x;
    to.y = from.y;
    to.z = from.z;
}

void convert(   const geometry_msgs::Point& from,
                geometry_msgs::Point& to)
{
    to = from;
}

void convert(   const geometry_msgs::Vector3& from,
                geometry_msgs::Vector3& to)
{
    to = from;
}

void convert(   const geometry_msgs::Point32& from,
                geometry_msgs::Point32& to)
{
    to = from;
}


// ROTATIONS
void convert(   const geometry_msgs::Quaternion& from,
                geometry_msgs::Quaternion& to)
{
    to = from;
}

// TRANSFORMATIONS
void convert(   const geometry_msgs::Pose& from,
                geometry_msgs::Transform& to)
{
    // to.translation = from.position;
    convert(from.position, to.translation);
    convert(from.orientation, to.rotation);
}

void convert(   const geometry_msgs::Transform& from,
                geometry_msgs::Pose& to)
{
    convert(from.translation, to.position);
    convert(from.rotation, to.orientation);
}

void convert(   const geometry_msgs::Transform& from,
                geometry_msgs::Transform& to)
{
    to = from;
}

void convert(   const geometry_msgs::Pose& from,
                geometry_msgs::Pose& to)
{
    to = from;
}

// OPERATORS

// POINTS
geometry_msgs::Vector3& operator<<=(geometry_msgs::Vector3& to,
                                    const geometry_msgs::Point& from)
{
    convert(from, to);
    return to;
}

geometry_msgs::Point& operator<<=(   geometry_msgs::Point& to,
                                const geometry_msgs::Vector3& from)
{
    convert(from, to);
    return to;
}

geometry_msgs::Point32& operator<<=(   geometry_msgs::Point32& to,
                                const geometry_msgs::Point& from)
{
    convert(from, to);
    return to;
}

geometry_msgs::Point& operator<<=(   geometry_msgs::Point& to,
                                const geometry_msgs::Point32& from)
{
    convert(from, to);
    return to;
}

geometry_msgs::Vector3& operator<<=(geometry_msgs::Vector3& to,
                                    const geometry_msgs::Point32& from)
{
    convert(from, to);
    return to;
}

geometry_msgs::Point32& operator<<=(   geometry_msgs::Point32& to,
                                const geometry_msgs::Vector3& from)
{
    convert(from, to);
    return to;
}

geometry_msgs::Point& operator<<=(   geometry_msgs::Point& to,
                                const geometry_msgs::Point& from)
{
    convert(from, to);
    return to;
}

geometry_msgs::Vector3& operator<<=(geometry_msgs::Vector3& to,
                                    const geometry_msgs::Vector3& from)
{
    convert(from, to);
    return to;
}

geometry_msgs::Point32& operator<<=(   geometry_msgs::Point32& to,
                                const geometry_msgs::Point32& from)
{
    convert(from, to);
    return to;
}

// ROTATIONS
geometry_msgs::Quaternion& operator<<=(   geometry_msgs::Quaternion& to,
                                const geometry_msgs::Quaternion& from)
{
    convert(from, to);
    return to;
}

// TRANSFORMATIONS
geometry_msgs::Pose& operator<<=(   geometry_msgs::Pose& to,
                                    const geometry_msgs::Transform& from)
{
    convert(from, to);
    return to;
}

geometry_msgs::Transform& operator<<=(   geometry_msgs::Transform& to,
                                        const geometry_msgs::Pose& from)
{
    convert(from, to);
    return to;
}


geometry_msgs::Transform& operator<<=(   geometry_msgs::Transform& to,
                                const geometry_msgs::Transform& from)
{
    convert(from, to);
    return to;
}

geometry_msgs::Pose& operator<<=(   geometry_msgs::Pose& to,
                                const geometry_msgs::Pose& from)
{
    convert(from, to);
    return to;
}


} // namespace rosmath