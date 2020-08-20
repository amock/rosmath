#ifndef ROSMATH_CONVERSIONS_H
#define ROSMATH_CONVERSIONS_H

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Vector3Stamped.h>

namespace rosmath {

///////////////////////////////////////////
//
// CONVERSION FUNCTIONS
//
/////////////

// POINTS
void convert(   const geometry_msgs::Point& from,
                geometry_msgs::Vector3& to);

void convert(   const geometry_msgs::Vector3& from,
                geometry_msgs::Point& to);

void convert(   const geometry_msgs::Point& from,
                geometry_msgs::Point32& to);

void convert(   const geometry_msgs::Point32& from,
                geometry_msgs::Point& to);

void convert(   const geometry_msgs::Vector3& from,
                geometry_msgs::Point32& to);

void convert(   const geometry_msgs::Point32& from,
                geometry_msgs::Vector3& to);

void convert(   const geometry_msgs::Point& from,
                geometry_msgs::Point& to);

void convert(   const geometry_msgs::Vector3& from,
                geometry_msgs::Vector3& to);

void convert(   const geometry_msgs::Point32& from,
                geometry_msgs::Point32& to);

// ROTATIONS
void convert(   const geometry_msgs::Quaternion& from,
                geometry_msgs::Quaternion& to);

// TRANSFORMATIONS
void convert(   const geometry_msgs::Pose& from,
                geometry_msgs::Transform& to);

void convert(   const geometry_msgs::Transform& from,
                geometry_msgs::Pose& to);

void convert(   const geometry_msgs::Transform& from,
                geometry_msgs::Transform& to);

void convert(   const geometry_msgs::Pose& from,
                geometry_msgs::Pose& to);

///////////////////////////////////////////
//
// CONVERSION OPERATOR: 
// <<= for first convert than assign
//
/////////////

// POINTS
geometry_msgs::Vector3& operator<<=(geometry_msgs::Vector3& to,
                                    const geometry_msgs::Point& from);

geometry_msgs::Point& operator<<=(   geometry_msgs::Point& to,
                                const geometry_msgs::Vector3& from);

geometry_msgs::Point32& operator<<=(   geometry_msgs::Point32& to,
                                const geometry_msgs::Point& from);

geometry_msgs::Point& operator<<=(   geometry_msgs::Point& to,
                                const geometry_msgs::Point32& from);

geometry_msgs::Vector3& operator<<=(geometry_msgs::Vector3& to,
                                    const geometry_msgs::Point32& from);

geometry_msgs::Point32& operator<<=(   geometry_msgs::Point32& to,
                                const geometry_msgs::Vector3& from);

geometry_msgs::Point& operator<<=(   geometry_msgs::Point& to,
                                const geometry_msgs::Point& from);

geometry_msgs::Vector3& operator<<=(geometry_msgs::Vector3& to,
                                    const geometry_msgs::Vector3& from);

geometry_msgs::Point32& operator<<=(   geometry_msgs::Point32& to,
                                const geometry_msgs::Point32& from);

// ROTATIONS
geometry_msgs::Quaternion& operator<<=(   geometry_msgs::Quaternion& to,
                                        const geometry_msgs::Quaternion& from);

// TRANSFORMATIONS
geometry_msgs::Pose& operator<<=(   geometry_msgs::Pose& to,
                                    const geometry_msgs::Transform& from);

geometry_msgs::Transform& operator<<=(   geometry_msgs::Transform& to,
                                        const geometry_msgs::Pose& from);

geometry_msgs::Transform& operator<<=(   geometry_msgs::Transform& to,
                                        const geometry_msgs::Transform& from);

geometry_msgs::Pose& operator<<=(   geometry_msgs::Pose& to,
                                        const geometry_msgs::Pose& from);



} // namespace rosmath

#endif // ROSMATH_CONVERSIONS_H