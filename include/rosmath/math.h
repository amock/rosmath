#ifndef ROSMATH_MATH_H
#define ROSMATH_MATH_H

// global deps
#include <Eigen/Dense>
#include <limits>

// global ros deps
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/PoseStamped.h>

// TODO
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/AccelStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/InertiaStamped.h>
#include <geometry_msgs/WrenchStamped.h>

// TODO covs
#include <geometry_msgs/PoseWithCovarianceStamped.h>


// internal deps
#include "conversions.h"

// only include this when eigen was found
#include "conversions_eigen.h"

namespace rosmath {

constexpr double EPS_DBL = std::numeric_limits<double>::epsilon() * 10.0;
constexpr float EPS_FLT = std::numeric_limits<float>::epsilon() * 10.0;

// functions
geometry_msgs::Point    neg(const geometry_msgs::Point& p);
geometry_msgs::Vector3  neg(const geometry_msgs::Vector3& p);
geometry_msgs::Point32  neg(const geometry_msgs::Point32& p);

// equal? already implemented. 
// but: maybe self implement due to hard precision unequalities
// 
bool equal( const geometry_msgs::Point& a,
            const geometry_msgs::Point& b);
bool equal( const geometry_msgs::Vector3& a,
            const geometry_msgs::Vector3& b);
bool equal( const geometry_msgs::Point32& a,
            const geometry_msgs::Point32& b);
bool equal( const geometry_msgs::PointStamped& a,
            const geometry_msgs::PointStamped& b);

// ADD
geometry_msgs::Point    add(const geometry_msgs::Point& a, 
                            const geometry_msgs::Point& b);
geometry_msgs::Vector3  add(const geometry_msgs::Vector3& a, 
                            const geometry_msgs::Vector3& b);
geometry_msgs::Point32  add(const geometry_msgs::Point32& a,
                            const geometry_msgs::Point32& b);
geometry_msgs::Point    add(const geometry_msgs::Point& p, 
                            const double& scalar);
geometry_msgs::Vector3  add(const geometry_msgs::Vector3& p, 
                            const double& scalar);
geometry_msgs::Point32  add(const geometry_msgs::Point32& p, 
                            const float& scalar);

// SUBSTRACT
geometry_msgs::Point    sub(const geometry_msgs::Point& a, 
                            const geometry_msgs::Point& b);
geometry_msgs::Vector3  sub(const geometry_msgs::Vector3& a, 
                            const geometry_msgs::Vector3& b);
geometry_msgs::Point32  sub(const geometry_msgs::Point32& a,
                            const geometry_msgs::Point32& b);
geometry_msgs::Point    sub(const geometry_msgs::Point& p, 
                            const double& scalar);
geometry_msgs::Vector3  sub(const geometry_msgs::Vector3& p, 
                            const double& scalar);
geometry_msgs::Point32  sub(const geometry_msgs::Point32& p, 
                            const float& scalar);

// MULTIPLY
geometry_msgs::Point mult(  const geometry_msgs::Point& p,
                            const double& scalar);

geometry_msgs::Point mult(  const double& scalar,
                            const geometry_msgs::Point& p);

geometry_msgs::Vector3 mult( const geometry_msgs::Vector3& p, 
                             const double& scalar );

geometry_msgs::Vector3 mult( const double& scalar,
                             const geometry_msgs::Vector3& p );

geometry_msgs::Point32 mult( const geometry_msgs::Point32& p, 
                             const float& scalar );

geometry_msgs::Point32 mult( const float& scalar,
                             const geometry_msgs::Point32& p );

geometry_msgs::Quaternion mult( const geometry_msgs::Quaternion& p, 
                                const double& scalar );

geometry_msgs::Quaternion mult( const double& scalar,
                                const geometry_msgs::Quaternion& p );

geometry_msgs::Point mult(  const geometry_msgs::Transform& T, 
                            const geometry_msgs::Point& p);

geometry_msgs::Point mult(  const geometry_msgs::Quaternion& q,
                            const geometry_msgs::Point& p);

geometry_msgs::Vector3 mult(    const geometry_msgs::Transform& T, 
                                const geometry_msgs::Vector3& p);

geometry_msgs::Vector3 mult( const geometry_msgs::Quaternion& q,
                             const geometry_msgs::Vector3& p);

geometry_msgs::Transform mult(  const geometry_msgs::Transform& A, 
                                const geometry_msgs::Transform& B);

geometry_msgs::Quaternion mult(  const geometry_msgs::Quaternion& a, 
                                const geometry_msgs::Quaternion& b);

geometry_msgs::Pose mult(  const geometry_msgs::Transform& T, 
                                const geometry_msgs::Pose& p);

geometry_msgs::Polygon mult(  const geometry_msgs::Transform& T, 
                                const geometry_msgs::Polygon& p);

geometry_msgs::Accel mult( const geometry_msgs::Transform& T,
                            const geometry_msgs::Accel& a);

// stamped
geometry_msgs::TransformStamped mult(
    const geometry_msgs::TransformStamped& A,
    const geometry_msgs::TransformStamped& B);

geometry_msgs::PointStamped mult(
    const geometry_msgs::TransformStamped& T,
    const geometry_msgs::PointStamped& p);

geometry_msgs::PoseStamped mult(
    const geometry_msgs::TransformStamped& T,
    const geometry_msgs::PoseStamped& p);

// DIVIDE
geometry_msgs::Point        div(const geometry_msgs::Point& p, 
                                const double& scalar);
geometry_msgs::Vector3      div(const geometry_msgs::Vector3& p,
                                const double& scalar);
geometry_msgs::Point32      div(const geometry_msgs::Point32& p,
                                const double& scalar);
geometry_msgs::Quaternion   div(const geometry_msgs::Quaternion& p,
                                const double& scalar);

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
geometry_msgs::TransformStamped  inv(const geometry_msgs::TransformStamped& T);

double norm(const geometry_msgs::Point& p);
double norm(const geometry_msgs::Vector3& p);
float  norm(const geometry_msgs::Point32& p);
double norm(const geometry_msgs::Quaternion& q);

double sum(const geometry_msgs::Point& p);
double sum(const geometry_msgs::Vector3& p);
float  sum(const geometry_msgs::Point32& p);
double sum(const geometry_msgs::Quaternion& q);

void normalize(geometry_msgs::Quaternion& q);
void identity(geometry_msgs::Quaternion& q);
void identity(geometry_msgs::Transform& T);

// Transformations


// Operators

// EQUAL operator
// bool operator==(const geometry_msgs::Point& a,
//                 const geometry_msgs::Point& b);
// bool operator==(const geometry_msgs::Vector3& a,
//                 const geometry_msgs::Vector3& b);
// bool operator==(const geometry_msgs::Point32& a,
//                 const geometry_msgs::Point32& b);
// bool operator==(const geometry_msgs::PointStamped& a,
//                 const geometry_msgs::PointStamped& b);

// NOT EQUAL
// bool operator!=(const geometry_msgs::Point& a,
//                 const geometry_msgs::Point& b);
// bool operator!=(const geometry_msgs::Vector3& a,
//                 const geometry_msgs::Vector3& b);
// bool operator!=(const geometry_msgs::Point32& a,
//                 const geometry_msgs::Point32& b);
// bool operator!=(const geometry_msgs::PointStamped& a,
//                 const geometry_msgs::PointStamped& b);


// NEGATE
geometry_msgs::Point operator-(
    const geometry_msgs::Point& p);

geometry_msgs::Vector3 operator-(
    const geometry_msgs::Vector3& p);

geometry_msgs::Point32 operator-(
    const geometry_msgs::Point32& p);

// PLUS
geometry_msgs::Point operator+(
    const geometry_msgs::Point& a,
    const geometry_msgs::Point& b);

geometry_msgs::Vector3 operator+(
    const geometry_msgs::Vector3& a,
    const geometry_msgs::Vector3& b);

geometry_msgs::Point32 operator+(
    const geometry_msgs::Point32& a,
    const geometry_msgs::Point32& b);

geometry_msgs::Point operator+(
    const geometry_msgs::Point& p,
    const double& scalar);

geometry_msgs::Vector3 operator+(
    const geometry_msgs::Vector3& p,
    const double& scalar);

geometry_msgs::Point32 operator+(
    const geometry_msgs::Point32& p,
    const float& scalar);

// PLUS=
geometry_msgs::Point operator+=(
    geometry_msgs::Point& a,
    const geometry_msgs::Point& b);

geometry_msgs::Vector3 operator+=(
    geometry_msgs::Vector3& a,
    const geometry_msgs::Vector3& b);

geometry_msgs::Point32 operator+=(
    geometry_msgs::Point32& a,
    const geometry_msgs::Point32& b);

geometry_msgs::Point operator+=(
    geometry_msgs::Point& p,
    const double& scalar);

geometry_msgs::Vector3 operator+=(
    geometry_msgs::Vector3& p,
    const double& scalar);

geometry_msgs::Point32 operator+=(
    geometry_msgs::Point32& p,
    const float& scalar);

// MINUS
geometry_msgs::Point operator-(
    const geometry_msgs::Point& a,
    const geometry_msgs::Point& b);

geometry_msgs::Vector3 operator-(
    const geometry_msgs::Vector3& a,
    const geometry_msgs::Vector3& b);

geometry_msgs::Point32 operator-(
    const geometry_msgs::Point32& a,
    const geometry_msgs::Point32& b);

geometry_msgs::Point operator-(
    const geometry_msgs::Point& p,
    const double& scalar);

geometry_msgs::Vector3 operator-(
    const geometry_msgs::Vector3& p,
    const double& scalar);

geometry_msgs::Point32 operator-(
    const geometry_msgs::Point32& p,
    const float& scalar);

// MINUS=
geometry_msgs::Point operator-=(
    geometry_msgs::Point& a,
    const geometry_msgs::Point& b);

geometry_msgs::Vector3 operator-=(
    geometry_msgs::Vector3& a,
    const geometry_msgs::Vector3& b);

geometry_msgs::Point32 operator-=(
    geometry_msgs::Point32& a,
    const geometry_msgs::Point32& b);

geometry_msgs::Point operator-=(
    geometry_msgs::Point& p,
    const double& scalar);

geometry_msgs::Vector3 operator-=(
    geometry_msgs::Vector3& p,
    const double& scalar);

geometry_msgs::Point32 operator-=(
    geometry_msgs::Point32& p,
    const float& scalar);

// MULTIPLY
geometry_msgs::Point operator*(  
    const geometry_msgs::Point& p,
    const double& scalar);

geometry_msgs::Point operator*(  
    const double& scalar,
    const geometry_msgs::Point& p);

geometry_msgs::Vector3 operator*( 
    const geometry_msgs::Vector3& p, 
    const double& scalar );

geometry_msgs::Vector3 operator*( 
    const double& scalar,
    const geometry_msgs::Vector3& p );

geometry_msgs::Point32 operator*( 
    const geometry_msgs::Point32& p, 
    const float& scalar );

geometry_msgs::Point32 operator*( 
    const float& scalar,
    const geometry_msgs::Point32& p );

geometry_msgs::Quaternion operator*( 
    const geometry_msgs::Quaternion& p, 
    const double& scalar );

geometry_msgs::Quaternion operator*( 
    const double& scalar,
    const geometry_msgs::Quaternion& p );

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

geometry_msgs::Quaternion operator*(
    const geometry_msgs::Quaternion& a,
    const geometry_msgs::Quaternion& b);

geometry_msgs::Pose operator*(
    const geometry_msgs::Transform& T,
    const geometry_msgs::Pose& p);

// STAMPED
geometry_msgs::TransformStamped operator*(
    const geometry_msgs::TransformStamped& A,
    const geometry_msgs::TransformStamped& B);

geometry_msgs::PointStamped operator*(
    const geometry_msgs::TransformStamped& T,
    const geometry_msgs::PointStamped& p);

geometry_msgs::PoseStamped operator*(
    const geometry_msgs::TransformStamped& T,
    const geometry_msgs::PoseStamped& p);

geometry_msgs::Point operator*=(  
    geometry_msgs::Point& p,
    const double& scalar);

geometry_msgs::Vector3 operator*=(  
    geometry_msgs::Vector3& p,
    const double& scalar);

geometry_msgs::Point32 operator*=(  
    geometry_msgs::Point32& p,
    const float& scalar);

// DIVIDE
geometry_msgs::Point operator/(
    const geometry_msgs::Point& p,
    const double& scalar);

geometry_msgs::Vector3 operator/(
    const geometry_msgs::Vector3& p,
    const double& scalar);

geometry_msgs::Point32 operator/(
    const geometry_msgs::Point32& p,
    const double& scalar);

geometry_msgs::Quaternion operator/(
    const geometry_msgs::Quaternion& p,
    const double& scalar);

geometry_msgs::Point operator/=(
    geometry_msgs::Point& p,
    const double& scalar);

geometry_msgs::Vector3 operator/=(
    geometry_msgs::Vector3& p,
    const double& scalar);

geometry_msgs::Point32 operator/=(
    geometry_msgs::Point32& p,
    const double& scalar);

geometry_msgs::Quaternion operator/=(
    geometry_msgs::Quaternion& p,
    const double& scalar);

// inverse operator
geometry_msgs::Quaternion operator~(
    const geometry_msgs::Quaternion& q);
    
geometry_msgs::Transform operator~(
    const geometry_msgs::Transform& T);

geometry_msgs::TransformStamped operator~(
    const geometry_msgs::TransformStamped& T);

}

#endif // ROSMATH_MATH_H