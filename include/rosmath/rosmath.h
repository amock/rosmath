#ifndef ROSMATH_ROSMATH_H
#define ROSMATH_ROSMATH_H

// global deps
#include <Eigen/Dense>

// global ros deps
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/QuaternionStamped.h>

// internal deps
#include "conversions.h"

// only include this when eigen was found
#include "conversions_eigen.h"


namespace rosmath {

// functions

geometry_msgs::Point    neg(const geometry_msgs::Point& p);
geometry_msgs::Vector3  neg(const geometry_msgs::Vector3& p);
geometry_msgs::Point32  neg(const geometry_msgs::Point32& p);

geometry_msgs::Point    add(const geometry_msgs::Point& a, 
                            const geometry_msgs::Point& b);
geometry_msgs::Vector3  add(const geometry_msgs::Vector3& a, 
                            const geometry_msgs::Vector3& b);
geometry_msgs::Point32  add(const geometry_msgs::Point32& a,
                            const geometry_msgs::Point32& b);

geometry_msgs::Point    sub(const geometry_msgs::Point& a, 
                            const geometry_msgs::Point& b);
geometry_msgs::Vector3  sub(const geometry_msgs::Vector3& a, 
                            const geometry_msgs::Vector3& b);
geometry_msgs::Point32  sub(const geometry_msgs::Point32& a,
                            const geometry_msgs::Point32& b);

// Point
geometry_msgs::Point mult(
    const geometry_msgs::Transform& T, 
    const geometry_msgs::Point& p);

geometry_msgs::Point mult(
    const geometry_msgs::Quaternion& q,
    const geometry_msgs::Point& p);

geometry_msgs::Vector3 mult(
    const geometry_msgs::Transform& T, 
    const geometry_msgs::Vector3& p);

geometry_msgs::Vector3 mult( const geometry_msgs::Quaternion& q,
                             const geometry_msgs::Vector3& p);

geometry_msgs::Transform mult(  const geometry_msgs::Transform A, 
                                const geometry_msgs::Transform B);

double  dot(    const geometry_msgs::Point& a,
                const geometry_msgs::Point& b);
double  dot(    const geometry_msgs::Vector3& a,
                const geometry_msgs::Vector3& b);
float   dot(    const geometry_msgs::Point32& a,
                const geometry_msgs::Point32& b);

geometry_msgs::Point    cross(  const geometry_msgs::Point& a,
                                const geometry_msgs::Point& b);
geometry_msgs::Vector3  cross(  const geometry_msgs::Vector3& a,
                                const geometry_msgs::Vector3& b);
geometry_msgs::Point32  cross(  const geometry_msgs::Point32& a,
                                const geometry_msgs::Point32& b);

geometry_msgs::Quaternion inv(const geometry_msgs::Quaternion& q);
geometry_msgs::Transform  inv(const geometry_msgs::Transform& T);

double norm(const geometry_msgs::Point& p);
double norm(const geometry_msgs::Vector3& p);
float  norm(const geometry_msgs::Point32& p);
double norm(const geometry_msgs::Quaternion& q);

double sum(const geometry_msgs::Point& p);
double sum(const geometry_msgs::Vector3& p);
float  sum(const geometry_msgs::Point32& p);
double sum(const geometry_msgs::Quaternion& q);

// Transformations


// Operators

geometry_msgs::Point operator-(
    const geometry_msgs::Point& p);

geometry_msgs::Vector3 operator-(
    const geometry_msgs::Vector3& p);

geometry_msgs::Point32 operator-(
    const geometry_msgs::Point32& p);

geometry_msgs::Point operator+(
    const geometry_msgs::Point& a,
    const geometry_msgs::Point& b);

geometry_msgs::Vector3 operator+(
    const geometry_msgs::Vector3& a,
    const geometry_msgs::Vector3& b);

geometry_msgs::Point32 operator+(
    const geometry_msgs::Point32& a,
    const geometry_msgs::Point32& b);

geometry_msgs::Point operator-(
    const geometry_msgs::Point& a,
    const geometry_msgs::Point& b);

geometry_msgs::Vector3 operator-(
    const geometry_msgs::Vector3& a,
    const geometry_msgs::Vector3& b);

double operator*(
    const geometry_msgs::Point& a,
    const geometry_msgs::Point& b);

geometry_msgs::Point operator*(
    const geometry_msgs::Quaternion& q,
    const geometry_msgs::Point& p);

geometry_msgs::Point operator*(
    const geometry_msgs::Transform& T,
    const geometry_msgs::Point& p);

geometry_msgs::Vector3 operator*(
    const geometry_msgs::Quaternion& q,
    const geometry_msgs::Vector3& p);

geometry_msgs::Vector3 operator*(
    const geometry_msgs::Transform& T,
    const geometry_msgs::Vector3& p);

geometry_msgs::Transform operator*(
    const geometry_msgs::Transform& A,
    const geometry_msgs::Transform& B);

// inverse operator
geometry_msgs::Quaternion operator~(
    const geometry_msgs::Quaternion& q);
    
geometry_msgs::Transform operator~(
    const geometry_msgs::Transform& T);

}

#endif // ROSMATH_ROSMATH_H