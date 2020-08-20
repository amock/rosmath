# rosmath


## Why
After I looked up the conversion from `geometry_mgs::Transform` to `Eigen::Affine3d` the 1000th I was finally able to build this ROS library. It helps me doing this kind of stuff not the 1001th time.

## Examples

### Simple math 

Doing math directly with commonly used ROS messages.

```c++
#include <ros/ros.h>
#include <rosmath/rosmath.h>
#include <Eigen/Dense>

using namespace rosmath;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rosmath_example_minimal");

    // transformations
    geometry_msgs::Point p;
    geometry_msgs::Transform T;
    auto p_transformed = T * p;

    geometry_msgs::Quaternion q;
    auto p_rotated = q * p;

    // norm
    double norm_p = norm(p_transformed);

    // inverse
    auto Tinv = ~T;

    return 0;
}
```

### Conversions

Converting well known types via `<<=` operator or `convert` function.

```c++
#include <ros/ros.h>
#include <rosmath/rosmath.h>
#include <Eigen/Dense>

using namespace rosmath;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rosmath_example_conversions");

    geometry_msgs::Point p;
    geometry_msgs::Vector3 v;
    geometry_msgs::Transform T;
    geometry_msgs::Quaternion q;
    geometry_msgs::Pose pose;
    Eigen::Affine3d Teig;
    Eigen::Quaterniond qeig;
    Eigen::Vector3d peig;

    // Convertion anything into everything
    p <<= v;
    v <<= p;
    peig <<= p;

    T <<= pose;
    pose <<= T;
    Teig <<= pose;
    Teig <<= T;
    T <<= Teig;
    
    q <<= q;
    q <<= qeig;
    qeig <<= q;

    return 0;
}
```