#ifndef ROSMATH_NAV_MSGS_MATH_H
#define ROSMATH_NAV_MSGS_MATH_H

#include "rosmath/math.h"

namespace rosmath {

nav_msgs::Path mult(
    const geometry_msgs::TransformStamped& T,
    const nav_msgs::Path& p);

nav_msgs::Path operator*(
    const geometry_msgs::TransformStamped& T,
    const nav_msgs::Path& p);

} // namespace rosmath

#endif // ROSMATH_NAV_MSGS_MATH_H