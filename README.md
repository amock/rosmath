# rosmath


## Why
After I looked up the conversion from `geometry_mgs::Transform` to `Eigen::Affine3d` the 1000th I was finally able to build this ROS library. It helps me doing this kind of stuff not the 1001th time.

## Include rosmath
Clone rosmath into the `src` directory of your ROS workspace.
Create a ROS package with `rosmath` as dependency.

```console
foo@bar:~/catkin_ws/src$ catkin_create_pkg mypackagename roscpp rosmath
```

Register your ROS-Nodes in your `CMakeLists.txt` 
(see ROS-Tutorials). Now your should be able to use `rosmath`
in your own code.

## Examples

### Simple math 

Doing math with commonly used ROS messages.

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

### Namespaces
Maybe you dont want to use `using namespace rosmath;` in every
of your projects. In this case there are two options to choose.
With the first options you choose to not use any of rosmaths namespaces. Then you may use the operators equivalent functions instead like:
- `~`: `inv`
- `+`: `add`
- `-`: `sub`,`neg`
- `*`: `mult`, `dot`
- `<<=`: `convert`

The second option consists of using only the operators like:
`using rosmath::operator<<=;`. If you want to use every operator
then you may choose to include the operator using-directives bundled in `rosmath/usingoperators.h` as you can see here:

```c++
#include <ros/ros.h>
#include <rosmath/rosmath.h>
#include <rosmath/usingoperators.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rosmath_example_namespace");

    // transformations
    geometry_msgs::Point p;
    geometry_msgs::Transform T;
    auto p_transformed = T * p;

    geometry_msgs::Quaternion q;
    auto p_rotated = q * p;

    // norm
    double norm = rosmath::norm(p_transformed);
    double sum = rosmath::sum(p_transformed);

    // inverse
    auto Tinv = ~T;

    return 0;
}
```

### TODOs
- More math
- Include Stamped messages/transformes: Throw proper errors if source CS of transformation missmatches the coordinate system of the Stamped
- Documentation. Code, Doxygen, etc.
- Advanced examples
- ...