#include "rosmath/rosmath.h"

// internal deps
#include "rosmath/conversions.h"
#include "rosmath/conversions_eigen.h"

#include <Eigen/Dense>


namespace rosmath {

// Functions

geometry_msgs::Point    neg(const geometry_msgs::Point& p)
{
    geometry_msgs::Point ret;
    ret.x = -p.x;
    ret.y = -p.y;
    ret.z = -p.z;
    return ret;
}

geometry_msgs::Vector3  neg(const geometry_msgs::Vector3& p)
{
    geometry_msgs::Vector3 ret;
    ret.x = -p.x;
    ret.y = -p.y;
    ret.z = -p.z;
    return ret;
}

geometry_msgs::Point32  neg(const geometry_msgs::Point32& p)
{
    geometry_msgs::Point32 ret;
    ret.x = -p.x;
    ret.y = -p.y;
    ret.z = -p.z;
    return ret;
}

geometry_msgs::Point    add(const geometry_msgs::Point& a, 
                            const geometry_msgs::Point& b)
{
    geometry_msgs::Point ret;
    ret.x = a.x + b.x;
    ret.y = a.y + b.y;
    ret.z = a.z + b.z;
    return ret;
}

geometry_msgs::Vector3  add(const geometry_msgs::Vector3& a, 
                            const geometry_msgs::Vector3& b)
{
    geometry_msgs::Vector3 ret;
    ret.x = a.x + b.x;
    ret.y = a.y + b.y;
    ret.z = a.z + b.z;
    return ret;
}

geometry_msgs::Point32  add(const geometry_msgs::Point32& a,
                            const geometry_msgs::Point32& b)
{
    geometry_msgs::Point32 ret;
    ret.x = a.x + b.x;
    ret.y = a.y + b.y;
    ret.z = a.z + b.z;
    return ret;
}

geometry_msgs::Point    sub(const geometry_msgs::Point& a, 
                            const geometry_msgs::Point& b)
{
    geometry_msgs::Point ret;
    ret.x = a.x - b.x;
    ret.y = a.y - b.y;
    ret.z = a.z - b.z;
    return ret;
}

geometry_msgs::Vector3  sub(const geometry_msgs::Vector3& a, 
                            const geometry_msgs::Vector3& b)
{
    geometry_msgs::Vector3 ret;
    ret.x = a.x - b.x;
    ret.y = a.y - b.y;
    ret.z = a.z - b.z;
    return ret;
}

geometry_msgs::Point32  sub(const geometry_msgs::Point32& a,
                            const geometry_msgs::Point32& b)
{
    geometry_msgs::Point32 ret;
    ret.x = a.x - b.x;
    ret.y = a.y - b.y;
    ret.z = a.z - b.z;
    return ret;
}

geometry_msgs::Point mult(
    const geometry_msgs::Transform& T, 
    const geometry_msgs::Point& p)
{
    Eigen::Vector3d p_eigen;
    p_eigen <<= p;
    Eigen::Affine3d aff;
    aff <<= T;
    Eigen::Vector3d ret_eigen = aff * p_eigen;
    geometry_msgs::Point ret;
    ret <<= ret_eigen;
    return ret;
}

geometry_msgs::Point mult(
    const geometry_msgs::Quaternion& q,
    const geometry_msgs::Point& p)
{
    Eigen::Quaterniond q_eigen;
    q_eigen <<= q;
    Eigen::Vector3d p_eigen;
    p_eigen <<= p;
    Eigen::Vector3d p_rot_eigen = q_eigen * p_eigen;
    geometry_msgs::Point p_rot;
    p_rot <<= p_rot_eigen;
    return p_rot;
}

geometry_msgs::Vector3 mult(
    const geometry_msgs::Transform& T, 
    const geometry_msgs::Vector3& p)
{
    Eigen::Vector3d p_eigen;
    p_eigen <<= p;
    Eigen::Affine3d aff;
    aff <<= T;
    Eigen::Vector3d ret_eigen = aff * p_eigen;
    geometry_msgs::Vector3 ret;
    ret <<= ret_eigen;
    return ret;
}

geometry_msgs::Vector3 mult(
    const geometry_msgs::Quaternion& q,
    const geometry_msgs::Vector3& p)
{
    Eigen::Quaterniond q_eigen;
    q_eigen <<= q;
    Eigen::Vector3d p_eigen;
    p_eigen <<= p;
    Eigen::Vector3d p_rot_eigen = q_eigen * p_eigen;
    geometry_msgs::Vector3 p_rot;
    p_rot <<= p_rot_eigen;
    return p_rot;
}

geometry_msgs::Transform mult(  const geometry_msgs::Transform A, 
                                const geometry_msgs::Transform B)
{
    Eigen::Affine3d Aeig;
    Eigen::Affine3d Beig;
    Aeig <<= A;
    Beig <<= B;
    Eigen::Affine3d Ceig = Aeig * Beig;
    geometry_msgs::Transform C;
    C <<= Ceig;
    return C;
}

double dot(
    const geometry_msgs::Point& a,
    const geometry_msgs::Point& b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z; 
}

double dot(
    const geometry_msgs::Vector3& a,
    const geometry_msgs::Vector3& b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z; 
}

float dot(
    const geometry_msgs::Point32& a,
    const geometry_msgs::Point32& b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z; 
}

geometry_msgs::Point cross(
    const geometry_msgs::Point& a,
    const geometry_msgs::Point& b)
{
    geometry_msgs::Point ret;
    ret.x = a.y * b.z + a.z * b.y;
    ret.y = a.z * b.x + a.x * b.z;
    ret.z = a.x * b.y + a.y * b.x;
    return ret;
}

geometry_msgs::Vector3 cross(
    const geometry_msgs::Vector3& a,
    const geometry_msgs::Vector3& b)
{
    geometry_msgs::Vector3 ret;
    ret.x = a.y * b.z + a.z * b.y;
    ret.y = a.z * b.x + a.x * b.z;
    ret.z = a.x * b.y + a.y * b.x;
    return ret;
}

geometry_msgs::Point32 cross(
    const geometry_msgs::Point32& a,
    const geometry_msgs::Point32& b)
{
    geometry_msgs::Point32 ret;
    ret.x = a.y * b.z + a.z * b.y;
    ret.y = a.z * b.x + a.x * b.z;
    ret.z = a.x * b.y + a.y * b.x;
    return ret;
}

geometry_msgs::Quaternion inv(
    const geometry_msgs::Quaternion& q)
{
    geometry_msgs::Quaternion qinv;
    qinv.x = -q.x;
    qinv.y = -q.y;
    qinv.z = -q.z;
    qinv.w = q.w;
    // tf tutorial's solution:
    // qinv.x = q.x;
    // qinv.y = q.y;
    // qinv.z = q.z;
    // qinv.w = -q.w;
    // but why?
    // http://wiki.ros.org/tf2/Tutorials/Quaternions
    return qinv;
}

geometry_msgs::Transform inv(
    const geometry_msgs::Transform& T)
{
    geometry_msgs::Transform Tinv;

    // rigid transformation inverse
    // 
    Tinv.rotation = inv(T.rotation);
    Eigen::Quaterniond qinv;
    Tinv.translation = -(Tinv.rotation * T.translation);

    return Tinv;
}

double norm(const geometry_msgs::Point& p)
{
    return sqrt(dot(p, p));
}

double norm(const geometry_msgs::Vector3& p)
{
    return sqrt(dot(p, p));
}

float  norm(const geometry_msgs::Point32& p)
{
    return sqrt(dot(p, p));
}

double norm(const geometry_msgs::Quaternion& q)
{
    return sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
}

double sum(const geometry_msgs::Point& p)
{
    return p.x + p.y + p.z;
}

double sum(const geometry_msgs::Vector3& p)
{
    return p.x + p.y + p.z;
}

float  sum(const geometry_msgs::Point32& p)
{
    return p.x + p.y + p.z;
}

double  sum(const geometry_msgs::Quaternion& q)
{
    return q.x + q.y + q.z + q.w;
}

// Operators

geometry_msgs::Point operator-(const geometry_msgs::Point& p)
{
    return neg(p);
}

geometry_msgs::Vector3 operator-(const geometry_msgs::Vector3& p)
{
    return neg(p);
}

geometry_msgs::Point32 operator-(const geometry_msgs::Point32& p)
{
    return neg(p);
}

geometry_msgs::Point operator+(
    const geometry_msgs::Point& a,
    const geometry_msgs::Point& b)
{
    return add(a, b);
}

geometry_msgs::Vector3 operator+(
    const geometry_msgs::Vector3& a,
    const geometry_msgs::Vector3& b)
{
    return add(a, b);
}

geometry_msgs::Point32 operator+(
    const geometry_msgs::Point32& a,
    const geometry_msgs::Point32& b)
{
    return add(a, b);
}

geometry_msgs::Point operator-(
    const geometry_msgs::Point& a,
    const geometry_msgs::Point& b)
{
    return sub(a, b);
}

geometry_msgs::Vector3 operator-(
    const geometry_msgs::Vector3& a,
    const geometry_msgs::Vector3& b)
{
    return sub(a, b);
}

double operator*(
    const geometry_msgs::Point& a,
    const geometry_msgs::Point& b)
{
    return dot(a, b);
}

geometry_msgs::Point operator*(
    const geometry_msgs::Quaternion& q,
    const geometry_msgs::Point& p)
{
    return mult(q, p);
}

geometry_msgs::Point operator*(
    const geometry_msgs::Transform& T,
    const geometry_msgs::Point& p)
{
    return mult(T, p);
}

geometry_msgs::Vector3 operator*(
    const geometry_msgs::Quaternion& q,
    const geometry_msgs::Vector3& p)
{
    return mult(q, p);
}

geometry_msgs::Vector3 operator*(
    const geometry_msgs::Transform& T,
    const geometry_msgs::Vector3& p)
{
    return mult(T, p);
}

geometry_msgs::Transform operator*(
    const geometry_msgs::Transform& A,
    const geometry_msgs::Transform& B)
{
    return mult(A, B);
}

// Inverse operator
geometry_msgs::Quaternion operator~(
    const geometry_msgs::Quaternion& q)
{
    return inv(q);
}

geometry_msgs::Transform operator~(
    const geometry_msgs::Transform& T)
{
    return inv(T);
}

} // namespace rosmath