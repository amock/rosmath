#include "rosmath/math.h"

// internal deps
#include "rosmath/conversions.h"
#include "rosmath/conversions_eigen.h"
#include "rosmath/exceptions.h"

#include <Eigen/Dense>


namespace rosmath {

// Functions


geometry_msgs::Vector3 RPY(   const double& roll,
                            const double& pitch,
                            const double& yaw)
{
    geometry_msgs::Vector3 rpy;
    rpy.x = roll;
    rpy.y = pitch;
    rpy.z = yaw;
    return rpy;
}

geometry_msgs::Quaternion rpy2quat(const geometry_msgs::Vector3& rpy)
{
    return rpy2quat(rpy.x, rpy.y, rpy.z);
}

geometry_msgs::Quaternion rpy2quat(const double& roll, 
                                   const double& pitch,
                                   const double& yaw)
{
    Eigen::Quaterniond qeig;
    qeig = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
        * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
    geometry_msgs::Quaternion q;
    q <<= qeig;
    return q;
}

geometry_msgs::Vector3 quat2rpy(const geometry_msgs::Quaternion& q)
{
    Eigen::Quaterniond qeig;
    qeig <<= q;
    Eigen::Vector3d euler = qeig.toRotationMatrix().eulerAngles(0, 1, 2);
    geometry_msgs::Vector3 p;
    p <<= euler;
    return p;
}

bool equal( const geometry_msgs::Point& a,
            const geometry_msgs::Point& b)
{
    return ( (abs(a.x - b.x) < EPS_DBL)
          && (abs(a.y - b.y) < EPS_DBL) 
          && (abs(a.z - b.z) < EPS_DBL) );
}

bool equal( const geometry_msgs::Vector3& a,
            const geometry_msgs::Vector3& b)
{
    return ( (abs(a.x - b.x) < EPS_DBL)
          && (abs(a.y - b.y) < EPS_DBL) 
          && (abs(a.z - b.z) < EPS_DBL) );
}

bool equal( const geometry_msgs::Point32& a,
            const geometry_msgs::Point32& b)
{
    return ( (abs(a.x - b.x) < EPS_FLT)
          && (abs(a.y - b.y) < EPS_FLT) 
          && (abs(a.z - b.z) < EPS_FLT) );
}

bool equal( const geometry_msgs::PointStamped& a,
            const geometry_msgs::PointStamped& b)
{
    return a.header == b.header && equal(a.point,b.point);
}

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

geometry_msgs::Point    add(const geometry_msgs::Point& p, 
                            const double& scalar)
{
    geometry_msgs::Point ret;
    ret.x = p.x + scalar;
    ret.y = p.y + scalar;
    ret.z = p.z + scalar;
    return ret;
}

geometry_msgs::Vector3  add(const geometry_msgs::Vector3& p, 
                            const double& scalar)
{
    geometry_msgs::Vector3 ret;
    ret.x = p.x + scalar;
    ret.y = p.y + scalar;
    ret.z = p.z + scalar;
    return ret;
}

geometry_msgs::Point32  add(const geometry_msgs::Point32& p, 
                            const float& scalar)
{
    geometry_msgs::Point32 ret;
    ret.x = p.x + scalar;
    ret.y = p.y + scalar;
    ret.z = p.z + scalar;
    return ret;
}

geometry_msgs::Quaternion add(
    const geometry_msgs::Quaternion& a,
    const geometry_msgs::Quaternion& b)
{
    geometry_msgs::Quaternion ret;
    ret.x = a.x + b.x;
    ret.y = a.y + b.y;
    ret.z = a.z + b.z;
    ret.w = a.w + b.w;
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

geometry_msgs::Point    sub(const geometry_msgs::Point& p, 
                            const double& scalar)
{
    geometry_msgs::Point ret;
    ret.x = p.x - scalar;
    ret.y = p.y - scalar;
    ret.z = p.z - scalar;
    return ret;
}

geometry_msgs::Vector3  sub(const geometry_msgs::Vector3& p, 
                            const double& scalar)
{
    geometry_msgs::Vector3 ret;
    ret.x = p.x - scalar;
    ret.y = p.y - scalar;
    ret.z = p.z - scalar;
    return ret;
}

geometry_msgs::Point32  sub(const geometry_msgs::Point32& p, 
                            const float& scalar)
{
    geometry_msgs::Point32 ret;
    ret.x = p.x - scalar;
    ret.y = p.y - scalar;
    ret.z = p.z - scalar;
    return ret;
}

geometry_msgs::Quaternion sub(
    const geometry_msgs::Quaternion& a,
    const geometry_msgs::Quaternion& b)
{
    geometry_msgs::Quaternion ret;
    ret.x = a.x - b.x;
    ret.y = a.y - b.y;
    ret.z = a.z - b.z;
    ret.w = a.w - b.w;
    return ret;
}

geometry_msgs::Point mult(  
    const geometry_msgs::Point& p,
    const double& scalar)
{
    geometry_msgs::Point res;
    res.x = p.x * scalar;
    res.y = p.y * scalar;
    res.z = p.z * scalar;
    return res;
}

geometry_msgs::Vector3 mult(  const double& scalar,
                            const geometry_msgs::Vector3& p)
{
    return mult(p, scalar);
}

geometry_msgs::Vector3 mult(  
    const geometry_msgs::Vector3& p,
    const double& scalar)
{
    geometry_msgs::Vector3 res;
    res.x = p.x * scalar;
    res.y = p.y * scalar;
    res.z = p.z * scalar;
    return res;
}

geometry_msgs::Point mult(  const double& scalar,
                            const geometry_msgs::Point& p)
{
    return mult(p, scalar);
}

geometry_msgs::Point32 mult(  
    const geometry_msgs::Point32& p,
    const float& scalar)
{
    geometry_msgs::Point32 res;
    res.x = p.x * scalar;
    res.y = p.y * scalar;
    res.z = p.z * scalar;
    return res;
}

geometry_msgs::Point32 mult(  const float& scalar,
                            const geometry_msgs::Point32& p)
{
    return mult(p, scalar);
}

geometry_msgs::Quaternion mult(  
    const geometry_msgs::Quaternion& p,
    const double& scalar)
{
    geometry_msgs::Quaternion res;
    res.x = p.x * scalar;
    res.y = p.y * scalar;
    res.z = p.z * scalar;
    res.w = p.w * scalar;
    return res;
}

geometry_msgs::Quaternion mult(  const double& scalar,
                            const geometry_msgs::Quaternion& p)
{
    return mult(p, scalar);
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
    // Eigen::Vector3d p_eigen;
    // p_eigen <<= p;
    // Eigen::Affine3d aff;
    // aff <<= T;
    // Eigen::Vector3d ret_eigen = aff * p_eigen;
    // geometry_msgs::Vector3 ret;
    // ret <<= ret_eigen;
    
    // WRONG 
    // vector3 is only direction: http://docs.ros.org/api/geometry_msgs/html/msg/Vector3.html
    // so apply only rotation

    return mult(T.rotation, p);
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

geometry_msgs::Transform mult(  const geometry_msgs::Transform& A, 
                                const geometry_msgs::Transform& B)
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

geometry_msgs::Quaternion mult(  const geometry_msgs::Quaternion& a, 
                                const geometry_msgs::Quaternion& b)
{
    Eigen::Quaterniond aeig, beig, ceig;
    aeig <<= a;
    beig <<= b;
    ceig = aeig * beig;
    geometry_msgs::Quaternion qret;
    qret <<= ceig;
    return qret;
}

geometry_msgs::Pose mult(   const geometry_msgs::Transform& T, 
                            const geometry_msgs::Pose& p)
{
    geometry_msgs::Pose ret;
    geometry_msgs::Transform pt;
    pt <<= p;
    pt = T * pt;
    ret <<= pt; 
    return ret;
}

geometry_msgs::Polygon mult(  const geometry_msgs::Transform& T, 
                              const geometry_msgs::Polygon& p)
{
    geometry_msgs::Polygon ret;
    ret.points.resize(p.points.size());
    for(int i=0; i<p.points.size(); i++)
    {
        geometry_msgs::Point point;
        point <<= p.points[i];
        ret.points[i] <<= T * point;
    }

    return ret;
}

geometry_msgs::Accel mult( const geometry_msgs::Transform& T,
                            const geometry_msgs::Accel& a)
{
    geometry_msgs::Accel ret;
    ret.linear = T.rotation * a.linear;
    ret.angular = quat2rpy(T.rotation * rpy2quat(a.angular));
    return ret;
}

geometry_msgs::Inertia mult(const geometry_msgs::Transform& T,
                            const geometry_msgs::Inertia& inertia)
{
    geometry_msgs::Inertia ret;
    // TODO: check if this is correct
    Eigen::Matrix3d R, ieig, ieig_transformed;
    R <<= T.rotation;

    ieig(0,0) = inertia.ixx;
    ieig(1,1) = inertia.iyy;
    ieig(2,2) = inertia.izz;
    ieig(0,1) = inertia.ixy;
    ieig(1,0) = inertia.ixy;
    ieig(0,2) = inertia.ixz;
    ieig(2,0) = inertia.ixz;
    ieig(1,2) = inertia.iyz;
    ieig(2,1) = inertia.iyz;

    ieig_transformed = R * ieig;

    ret.m = inertia.m;
    ret.com = T * inertia.com;
    ret.ixx = ieig_transformed(0,0);
    ret.iyy = ieig_transformed(1,1);
    ret.izz = ieig_transformed(2,2);
    ret.ixy = ieig_transformed(0,1);
    ret.ixz = ieig_transformed(0,2);
    ret.iyz = ieig_transformed(1,2);

    return ret;
}

geometry_msgs::Wrench mult(const geometry_msgs::Transform& T,
                            const geometry_msgs::Wrench& w)
{
    geometry_msgs::Wrench ret;
    ret.force = T.rotation * w.force;
    ret.torque = quat2rpy(T.rotation * rpy2quat(w.torque));
    return ret;
}

geometry_msgs::Twist mult(const geometry_msgs::Transform& T,
                            const geometry_msgs::Twist& twist)
{
    geometry_msgs::Twist ret;
    ret.linear = T.rotation * twist.linear;
    ret.angular = quat2rpy(T.rotation * rpy2quat(twist.angular));
    return ret;
}

geometry_msgs::PoseWithCovariance mult(
    const geometry_msgs::Transform& T,
    const geometry_msgs::PoseWithCovariance& p)
{
    geometry_msgs::PoseWithCovariance ret;
    ret.pose = T * p.pose;
    Eigen::Matrix<double, 6, 6> cov;
    convert(p.covariance, cov);
    Eigen::Matrix3d R;
    R <<= T.rotation;
    cov.block<3,3>(0,0) = R * cov.block<3,3>(0,0) * R.transpose();

    // how to rotate euler angles covariances?
    // cov.block<3,3>(3,3) = R * cov.block<3,3>(3,3) * R.transpose();

    // just a guess
    std::cout << "check if cov transform calulation is correct!" << std::endl;
    Eigen::Quaterniond qcov(cov.block<3,3>(3,3));
    Eigen::Quaterniond q;
    q <<= T.rotation;
    Eigen::Matrix3d tmp;
    tmp <<= q * qcov * q.inverse();
    cov.block<3,3>(3,3) = tmp;
    convert(cov, ret.covariance);
    return ret;
}

// STAMPED
geometry_msgs::TransformStamped mult(  
    const geometry_msgs::TransformStamped& A, 
    const geometry_msgs::TransformStamped& B)
{
    geometry_msgs::TransformStamped ret;

    // from child_frame_id to header.frame_id

    if(A.child_frame_id != B.header.frame_id)
    {
        throw TransformException(
            "\nCould not do transformation T{" + A.child_frame_id + "->" + A.header.frame_id 
            + "} * T{" + B.child_frame_id + "->" + B.header.frame_id 
            + "}\nrequired: T{A->B} = T{X->B} * T{A->X}\n"
            + "mismatched frames: " + A.child_frame_id + " != " + B.header.frame_id
            );
    }

    // check if stamps of A and B differ?
    ret.header = A.header;
    ret.child_frame_id = B.child_frame_id;
    ret.transform = mult(A.transform, B.transform);
    return ret;
}

geometry_msgs::PointStamped mult(
    const geometry_msgs::TransformStamped& T,
    const geometry_msgs::PointStamped& p)
{
    geometry_msgs::PointStamped ret;
    if(T.child_frame_id != p.header.frame_id)
    {
        throw TransformException(
            "\nCould not do transformation T{" + T.child_frame_id + "->" + T.header.frame_id 
            + "} * p{" + p.header.frame_id 
            + "}\nrequired: p{B} = T{A->B} * p{A}\n"
            + "mismatched frames: " + T.child_frame_id + " != " + p.header.frame_id
            );
    }
    ret.header.frame_id = T.header.frame_id;
    ret.header.stamp = p.header.stamp;
    ret.point = T.transform * p.point;
    return ret;
}

geometry_msgs::Vector3Stamped mult(
    const geometry_msgs::TransformStamped& T,
    const geometry_msgs::Vector3Stamped& v)
{
    geometry_msgs::Vector3Stamped ret;
    if(T.child_frame_id != v.header.frame_id)
    {
        throw TransformException(
            "\nCould not do transformation T{" + T.child_frame_id + "->" + T.header.frame_id 
            + "} * v{" + v.header.frame_id 
            + "}\nrequired: p{B} = T{A->B} * v{A}\n"
            + "mismatched frames: " + T.child_frame_id + " != " + v.header.frame_id
            );
    }
    ret.header.frame_id = T.header.frame_id;
    ret.header.stamp = v.header.stamp;
    ret.vector = T.transform * v.vector;
    return ret;
}

geometry_msgs::PoseStamped mult(
    const geometry_msgs::TransformStamped& T,
    const geometry_msgs::PoseStamped& p)
{
    geometry_msgs::PoseStamped ret;
    if(T.child_frame_id != p.header.frame_id)
    {
        throw TransformException(
            "\nCould not do transformation T{" + T.child_frame_id + "->" + T.header.frame_id 
            + "} * p{" + p.header.frame_id 
            + "}\nrequired: p{B} = T{A->B} * p{A}\n"
            + "mismatched frames: " + T.child_frame_id + " != " + p.header.frame_id
            );
    }
    ret.header.frame_id = T.header.frame_id;
    ret.header.stamp = p.header.stamp;
    
    // actual transformation
    ret.pose = T.transform * p.pose;

    return ret;
}

geometry_msgs::PoseArray mult(
    const geometry_msgs::TransformStamped& T,
    const geometry_msgs::PoseArray& parr)
{
    geometry_msgs::PoseArray ret;
    if(T.child_frame_id != parr.header.frame_id)
    {
        throw TransformException(
            "\nCould not do transformation T{" + T.child_frame_id + "->" + T.header.frame_id 
            + "} * p{" + parr.header.frame_id 
            + "}\nrequired: p{B} = T{A->B} * p{A}\n"
            + "mismatched frames: " + T.child_frame_id + " != " + parr.header.frame_id
            );
    }

    ret.header.frame_id = T.header.frame_id;
    ret.header.stamp = parr.header.stamp;
    
    // actual transformation
    ret.poses.resize(parr.poses.size());
    for(size_t i=0; i<parr.poses.size(); i++)
    {
        ret.poses[i] = T.transform * parr.poses[i];
    }

    return ret;
}

geometry_msgs::PolygonStamped mult(
    const geometry_msgs::TransformStamped& T,
    const geometry_msgs::PolygonStamped& p)
{
    geometry_msgs::PolygonStamped ret;

    if(T.child_frame_id != p.header.frame_id)
    {
        throw TransformException(
            "\nCould not do transformation T{" + T.child_frame_id + "->" + T.header.frame_id 
            + "} * p{" + p.header.frame_id 
            + "}\nrequired: p{B} = T{A->B} * p{A}\n"
            + "mismatched frames: " + T.child_frame_id + " != " + p.header.frame_id
            );
    }

    ret.header.frame_id = T.header.frame_id;
    ret.header.stamp = p.header.stamp;
    ret.polygon = mult(T.transform, p.polygon);

    return ret;
}

geometry_msgs::AccelStamped mult(
    const geometry_msgs::TransformStamped& T,
    const geometry_msgs::AccelStamped& a)
{
    geometry_msgs::AccelStamped ret;

    if(T.child_frame_id != a.header.frame_id)
    {
        throw TransformException(
            "\nCould not do transformation T{" + T.child_frame_id + "->" + T.header.frame_id 
            + "} * acc{" + a.header.frame_id 
            + "}\nrequired: acc{B} = T{A->B} * acc{A}\n"
            + "mismatched frames: " + T.child_frame_id + " != " + a.header.frame_id
            );
    }

    ret.header.frame_id = T.header.frame_id;
    ret.header.stamp = a.header.stamp;
    ret.accel = mult(T.transform, a.accel);

    return ret;
}

geometry_msgs::InertiaStamped mult(
    const geometry_msgs::TransformStamped& T,
    const geometry_msgs::InertiaStamped& inertia)
{
    geometry_msgs::InertiaStamped ret;

    if(T.child_frame_id != inertia.header.frame_id)
    {
        throw TransformException(
            "\nCould not do transformation T{" + T.child_frame_id + "->" + T.header.frame_id 
            + "} * inertia{" + inertia.header.frame_id 
            + "}\nrequired: inertia{B} = T{A->B} * inertia{A}\n"
            + "mismatched frames: " + T.child_frame_id + " != " + inertia.header.frame_id
            );
    }

    ret.header.frame_id = T.header.frame_id;
    ret.header.stamp = inertia.header.stamp;
    ret.inertia = mult(T.transform, inertia.inertia);

    return ret;
}

geometry_msgs::WrenchStamped mult(
    const geometry_msgs::TransformStamped& T,
    const geometry_msgs::WrenchStamped& wrench)
{
    geometry_msgs::WrenchStamped ret;

    if(T.child_frame_id != wrench.header.frame_id)
    {
        throw TransformException(
            "\nCould not do transformation T{" + T.child_frame_id + "->" + T.header.frame_id 
            + "} * wrench{" + wrench.header.frame_id 
            + "}\nrequired: wrench{B} = T{A->B} * wrench{A}\n"
            + "mismatched frames: " + T.child_frame_id + " != " + wrench.header.frame_id
            );
    }

    ret.header.frame_id = T.header.frame_id;
    ret.header.stamp = wrench.header.stamp;
    ret.wrench = mult(T.transform, wrench.wrench);

    return ret;
}

geometry_msgs::TwistStamped mult(
    const geometry_msgs::TransformStamped& T,
    const geometry_msgs::TwistStamped& twist)
{
    geometry_msgs::TwistStamped ret;

    if(T.child_frame_id != twist.header.frame_id)
    {
        throw TransformException(
            "\nCould not do transformation T{" + T.child_frame_id + "->" + T.header.frame_id 
            + "} * twist{" + twist.header.frame_id 
            + "}\nrequired: twist{B} = T{A->B} * twist{A}\n"
            + "mismatched frames: " + T.child_frame_id + " != " + twist.header.frame_id
            );
    }

    ret.header.frame_id = T.header.frame_id;
    ret.header.stamp = twist.header.stamp;
    ret.twist = mult(T.transform, twist.twist);

    return ret;
}

geometry_msgs::Point        div(const geometry_msgs::Point& p, 
                                const double& scalar)
{
    geometry_msgs::Point ret;
    ret.x = p.x / scalar;
    ret.y = p.y / scalar;
    ret.z = p.z / scalar;
    return ret;
}

geometry_msgs::Vector3 div(
    const geometry_msgs::Vector3& p,
    const double& scalar)
{
    geometry_msgs::Vector3 ret;
    ret.x = p.x / scalar;
    ret.y = p.y / scalar;
    ret.z = p.z / scalar;
    return ret;
}

geometry_msgs::Point32 div(
    const geometry_msgs::Point32& p,
    const double& scalar)
{
    geometry_msgs::Point32 ret;
    ret.x = p.x / scalar;
    ret.y = p.y / scalar;
    ret.z = p.z / scalar;
    return ret;
}

geometry_msgs::Quaternion div(
    const geometry_msgs::Quaternion& p,
    const double& scalar)
{
    geometry_msgs::Quaternion ret;
    ret.x = p.x / scalar;
    ret.y = p.y / scalar;
    ret.z = p.z / scalar;
    ret.w = p.w / scalar;
    return ret;
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

geometry_msgs::Pose inv(const geometry_msgs::Pose& p)
{
    geometry_msgs::Pose ret;
    geometry_msgs::Transform T;
    T <<= p;
    T = ~T;
    ret <<= T;
    return ret;
}

geometry_msgs::TransformStamped  inv(
    const geometry_msgs::TransformStamped& T)
{
    geometry_msgs::TransformStamped Tinv;
    Tinv.header.stamp = T.header.stamp;
    Tinv.header.frame_id = T.child_frame_id;
    Tinv.child_frame_id = T.header.frame_id;
    Tinv.transform = inv(T.transform);
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

void normalize(geometry_msgs::Quaternion& q)
{
    q /= norm(q);
}

void identity(geometry_msgs::Quaternion& q)
{
    q.x = 0.0;
    q.y = 0.0;
    q.z = 0.0;
    q.w = 1.0;
}

void identity(geometry_msgs::Transform& T)
{
    T.translation.x = 0.0;
    T.translation.y = 0.0;
    T.translation.z = 0.0;
    identity(T.rotation);
}

// Operators

// EQUAL
// bool operator==(const geometry_msgs::Point& a,
//                 const geometry_msgs::Point& b)
// {
//     return equal(a, b);
// }
// bool operator==(const geometry_msgs::Vector3& a,
//                 const geometry_msgs::Vector3& b)
// {
//     return equal(a, b);
// }
// bool operator==(const geometry_msgs::Point32& a,
//                 const geometry_msgs::Point32& b)
// {
//     return equal(a, b);
// }
// bool operator==(const geometry_msgs::PointStamped& a,
//                 const geometry_msgs::PointStamped& b)
// {
//     return equal(a, b);
// }

// NOT EQUAL
// bool operator!=(const geometry_msgs::Point& a,
//                 const geometry_msgs::Point& b)
// {
//     return !(a==b);
// }
// bool operator!=(const geometry_msgs::Vector3& a,
//                 const geometry_msgs::Vector3& b)
// {
//     return !(a==b);
// }
// bool operator!=(const geometry_msgs::Point32& a,
//                 const geometry_msgs::Point32& b)
// {
//     return !(a==b);
// }
// bool operator!=(const geometry_msgs::PointStamped& a,
//                 const geometry_msgs::PointStamped& b)
// {
//     return !(a==b);
// }

// NEGATE
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

// PLUS
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

geometry_msgs::Point operator+(
    const geometry_msgs::Point& p,
    const double& scalar)
{
    return add(p, scalar);
}

geometry_msgs::Vector3 operator+(
    const geometry_msgs::Vector3& p,
    const double& scalar)
{
    return add(p, scalar);
}

geometry_msgs::Point32 operator+(
    const geometry_msgs::Point32& p,
    const float& scalar)
{
    return add(p, scalar);
}

geometry_msgs::Quaternion operator+(
    const geometry_msgs::Quaternion& a,
    const geometry_msgs::Quaternion& b)
{
    return add(a, b);
}

// PLUS=
geometry_msgs::Point operator+=(
    geometry_msgs::Point& a,
    const geometry_msgs::Point& b)
{
    a.x += b.x;
    a.y += b.y;
    a.z += b.z;
    return a;
}

geometry_msgs::Vector3 operator+=(
    geometry_msgs::Vector3& a,
    const geometry_msgs::Vector3& b)
{
    a.x += b.x;
    a.y += b.y;
    a.z += b.z;
    return a;
}

geometry_msgs::Point32 operator+=(
    geometry_msgs::Point32& a,
    const geometry_msgs::Point32& b)
{
    a.x += b.x;
    a.y += b.y;
    a.z += b.z;
    return a;
}

geometry_msgs::Point operator+=(
    geometry_msgs::Point& p,
    const double& scalar)
{
    p.x += scalar;
    p.y += scalar;
    p.z += scalar;
    return p;
}

geometry_msgs::Vector3 operator+=(
    geometry_msgs::Vector3& p,
    const double& scalar)
{
    p.x += scalar;
    p.y += scalar;
    p.z += scalar;
    return p;
}

geometry_msgs::Point32 operator+=(
    geometry_msgs::Point32& p,
    const float& scalar)
{
    p.x += scalar;
    p.y += scalar;
    p.z += scalar;
    return p;
}

// MINUS
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

geometry_msgs::Point32 operator-(
    const geometry_msgs::Point32& a,
    const geometry_msgs::Point32& b)
{
    return sub(a, b);
}

geometry_msgs::Point operator-(
    const geometry_msgs::Point& p,
    const double& scalar)
{
    return sub(p, scalar);
}

geometry_msgs::Vector3 operator-(
    const geometry_msgs::Vector3& p,
    const double& scalar)
{
    return sub(p, scalar);
}

geometry_msgs::Point32 operator-(
    const geometry_msgs::Point32& p,
    const float& scalar)
{
    return sub(p, scalar);
}

geometry_msgs::Quaternion operator-(
    const geometry_msgs::Quaternion& a,
    const geometry_msgs::Quaternion& b)
{
    return sub(a, b);
}

// MINUS=
geometry_msgs::Point operator-=(
    geometry_msgs::Point& a,
    const geometry_msgs::Point& b)
{
    a.x -= b.x;
    a.y -= b.y;
    a.z -= b.z;
    return a;
}

geometry_msgs::Vector3 operator-=(
    geometry_msgs::Vector3& a,
    const geometry_msgs::Vector3& b)
{
    a.x -= b.x;
    a.y -= b.y;
    a.z -= b.z;
    return a;
}

geometry_msgs::Point32 operator-=(
    geometry_msgs::Point32& a,
    const geometry_msgs::Point32& b)
{
    a.x -= b.x;
    a.y -= b.y;
    a.z -= b.z;
    return a;
}

geometry_msgs::Point operator-=(
    geometry_msgs::Point& p,
    const double& scalar)
{
    p.x -= scalar;
    p.y -= scalar;
    p.z -= scalar;
    return p;
}

geometry_msgs::Vector3 operator-=(
    geometry_msgs::Vector3& p,
    const double& scalar)
{
    p.x -= scalar;
    p.y -= scalar;
    p.z -= scalar;
    return p;
}

geometry_msgs::Point32 operator-=(
    geometry_msgs::Point32& p,
    const float& scalar)
{
    p.x -= scalar;
    p.y -= scalar;
    p.z -= scalar;
    return p;
}

// MULTIPLY
geometry_msgs::Point operator*(  
    const geometry_msgs::Point& p,
    const double& scalar)
{
    return mult(p, scalar);
}

geometry_msgs::Point operator*(  
    const double& scalar,
    const geometry_msgs::Point& p)
{
    return mult(scalar, p);
}

geometry_msgs::Vector3 operator*( 
    const geometry_msgs::Vector3& p, 
    const double& scalar )
{
    return mult(p, scalar);
}

geometry_msgs::Vector3 operator*( 
    const double& scalar,
    const geometry_msgs::Vector3& p )
{
    return mult(scalar, p);
}

geometry_msgs::Point32 operator*( 
    const geometry_msgs::Point32& p, 
    const float& scalar )
{
    return mult(p, scalar);
}

geometry_msgs::Point32 operator*( 
    const float& scalar,
    const geometry_msgs::Point32& p )
{
    return mult(scalar, p);
}

geometry_msgs::Quaternion operator*( 
    const geometry_msgs::Quaternion& p, 
    const double& scalar )
{
    return mult(p, scalar);
}

geometry_msgs::Quaternion operator*( 
    const double& scalar,
    const geometry_msgs::Quaternion& p )
{
    return mult(scalar, p);
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

geometry_msgs::Quaternion operator*(
    const geometry_msgs::Quaternion& a,
    const geometry_msgs::Quaternion& b)
{
    return mult(a, b);
}


geometry_msgs::Pose operator*(
    const geometry_msgs::Transform& T,
    const geometry_msgs::Pose& p)
{
    return mult(T, p);
}

// STAMPED
geometry_msgs::TransformStamped operator*(
    const geometry_msgs::TransformStamped& A,
    const geometry_msgs::TransformStamped& B)
{
    return mult(A, B);
}

geometry_msgs::PointStamped operator*(
    const geometry_msgs::TransformStamped& T,
    const geometry_msgs::PointStamped& p)
{
    return mult(T, p);
}

geometry_msgs::PoseStamped operator*(
    const geometry_msgs::TransformStamped& T,
    const geometry_msgs::PoseStamped& p)
{
    return mult(T, p);
}

// *= Operator
geometry_msgs::Point operator*=(  
    geometry_msgs::Point& p,
    const double& scalar)
{
    p.x *= scalar;
    p.y *= scalar;
    p.z *= scalar;
    return p;
}

geometry_msgs::Vector3 operator*=(  
    geometry_msgs::Vector3& p,
    const double& scalar)
{
    p.x *= scalar;
    p.y *= scalar;
    p.z *= scalar;
    return p;
}

geometry_msgs::Point32 operator*=(  
    geometry_msgs::Point32& p,
    const float& scalar)
{
    p.x *= scalar;
    p.y *= scalar;
    p.z *= scalar;
    return p;
}

// Divide
geometry_msgs::Point operator/(
    const geometry_msgs::Point& p,
    const double& scalar)
{
    return div(p, scalar);
}

geometry_msgs::Vector3 operator/(
    const geometry_msgs::Vector3& p,
    const double& scalar)
{
    return div(p, scalar);
}

geometry_msgs::Point32 operator/(
    const geometry_msgs::Point32& p,
    const double& scalar)
{
    return div(p, scalar);
}

geometry_msgs::Quaternion operator/(
    const geometry_msgs::Quaternion& p,
    const double& scalar)
{
    return div(p, scalar);
}

geometry_msgs::Point operator/=(
    geometry_msgs::Point& p,
    const double& scalar)
{
    p.x /= scalar;
    p.y /= scalar;
    p.z /= scalar;
    return p;
}

geometry_msgs::Vector3 operator/=(
    geometry_msgs::Vector3& p,
    const double& scalar)
{
    p.x /= scalar;
    p.y /= scalar;
    p.z /= scalar;
    return p;
}

geometry_msgs::Point32 operator/=(
    geometry_msgs::Point32& p,
    const double& scalar)
{
    p.x /= scalar;
    p.y /= scalar;
    p.z /= scalar;
    return p;
}

geometry_msgs::Quaternion operator/=(
    geometry_msgs::Quaternion& p,
    const double& scalar)
{
    p.x /= scalar;
    p.y /= scalar;
    p.z /= scalar;
    p.w /= scalar;
    return p;
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

geometry_msgs::Pose operator~(
    const geometry_msgs::Pose& p)
{
    return inv(p);
}

geometry_msgs::TransformStamped operator~(
    const geometry_msgs::TransformStamped& T)
{
    return inv(T);
}

} // namespace rosmath