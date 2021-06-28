#include "Eigen/Dense"
#include "rosmath/random.h"

#include "rosmath/eigen/conversions.h"

namespace rosmath {

namespace random {

// void seed(size_t seed)
// {
//     engine = std::default_random_engine(seed);
// }

size_t uniform_number(
    const size_t min,
    const size_t max)
{
    return std::uniform_int_distribution<size_t>(min, max)(engine);
}

std::vector<size_t> uniform_numbers(
    const size_t min,
    const size_t max,
    const size_t size)
{
    std::vector<size_t> ret(size);
    uniform_fill(ret, min, max);
    return ret;
}

double uniform_number(const double min, const double max)
{
    return std::uniform_real_distribution<double>(min, max)(engine);
}

std::vector<double> uniform_numbers(
    const double min, 
    const double max, 
    const size_t size)
{
    std::vector<double> ret(size);
    uniform_fill(ret, min, max);
    return ret;
}

double uniform_angle()
{
    return uniform_number(-M_PI, M_PI);
}

geometry_msgs::Point uniform_point(
    const geometry_msgs::Point pmin, 
    const geometry_msgs::Point pmax) 
{
    geometry_msgs::Point p;
    p.x = uniform_number(pmin.x, pmax.x);
    p.y = uniform_number(pmin.y, pmax.y);
    p.z = uniform_number(pmin.z, pmax.z);
    return p;
}

std::vector<geometry_msgs::Point> uniform_points(
    const geometry_msgs::Point pmin, 
    const geometry_msgs::Point pmax,
    size_t size)
{
    std::vector<geometry_msgs::Point> points(size);
    uniform_fill(points, pmin, pmax);
    return points;
}

geometry_msgs::Quaternion uniform_quaternion()
{
    geometry_msgs::Quaternion ret;
    double u = uniform_number(0.0, 1.0);
    double v = uniform_number(0.0, 1.0);
    double w = uniform_number(0.0, 1.0);

    ret.x = sqrt(1-u) * sin(2*M_PI * v);
    ret.y = sqrt(1-u) * cos(2*M_PI * v);
    ret.z = sqrt(u) * sin(2*M_PI * w);
    ret.w = sqrt(u) * cos(2*M_PI * w);
    normalize(ret);

    return ret;
}

geometry_msgs::Quaternion uniform_quaternion(
    const geometry_msgs::Vector3 axis)
{
    Eigen::Vector3d axis_eig;
    axis_eig <<= axis;
    Eigen::Quaterniond q_eig;
    q_eig = Eigen::AngleAxisd(uniform_angle(), axis_eig);
    geometry_msgs::Quaternion q;
    q <<= q_eig;
    return q;
}

void uniform_fill(
    std::vector<size_t>& data, 
    const size_t min,
    const size_t max)
{
    std::uniform_int_distribution<size_t> dist(min, max);
    std::generate(data.begin(), data.end(), [&dist](){
                   return dist(engine);
               });
}

void uniform_fill(std::vector<double>& data, const double min, const double max)
{
    std::uniform_real_distribution<double> dist(min, max);
    std::generate(data.begin(), data.end(), [&dist](){
                   return dist(engine);
               });
}

void uniform_fill(std::vector<geometry_msgs::Point>& data, 
    const geometry_msgs::Point pmin, 
    const geometry_msgs::Point pmax)
{
    std::uniform_real_distribution<double> distx(pmin.x, pmax.x);
    std::uniform_real_distribution<double> disty(pmin.y, pmax.y);
    std::uniform_real_distribution<double> distz(pmin.z, pmax.z);

    std::generate(data.begin(), data.end(), [&distx, &disty, &distz](){
                    geometry_msgs::Point p;
                    p.x = distx(engine);
                    p.y = disty(engine);
                    p.z = distz(engine);
                    return p;
               });
}

double normal_number(const double mu, const double sigma)
{
    return std::normal_distribution<double>(mu, sigma)(engine);
}

geometry_msgs::Point normal_point(
    const geometry_msgs::Point mu,
    const geometry_msgs::Point sigma)
{
    geometry_msgs::Point ret;

    ret.x = std::normal_distribution<double>(mu.x, sigma.x)(engine);
    ret.y = std::normal_distribution<double>(mu.y, sigma.y)(engine);
    ret.z = std::normal_distribution<double>(mu.z, sigma.z)(engine);

    return ret;
}

geometry_msgs::Quaternion normal_quaternion(
    const double sx,
    const double sy,
    const double sz)
{
    geometry_msgs::Quaternion q;

    q.x = std::normal_distribution<double>(0.0, sx)(engine);
    q.y = std::normal_distribution<double>(0.0, sy)(engine);
    q.z = std::normal_distribution<double>(0.0, sz)(engine);
    q.w = 1.0;

    normalize(q);

    return q;
}

std::vector<double> normal_numbers(
    const double mu, 
    const double sigma, 
    const size_t size)
{
    std::vector<double> ret(size);
    normal_fill(ret, mu, sigma);
    return ret;
}

std::vector<geometry_msgs::Point> normal_points(
    const geometry_msgs::Point mu,
    const geometry_msgs::Point sigma,
    const size_t size)
{
    std::vector<geometry_msgs::Point> ret(size);
    normal_fill(ret, mu, sigma);
    return ret;
}

void normal_fill(
    std::vector<double>& data,
    const double mu,
    const double sigma)
{
    std::normal_distribution<double> dist(mu, sigma);
    std::generate(data.begin(), data.end(), [&dist](){return dist(engine);});
}

void normal_fill(
    std::vector<geometry_msgs::Point>& data,
    const geometry_msgs::Point mu,
    const geometry_msgs::Point sigma)
{
    std::normal_distribution<double> distx(mu.x, sigma.x);
    std::normal_distribution<double> disty(mu.y, sigma.y);
    std::normal_distribution<double> distz(mu.z, sigma.z);

    std::generate(data.begin(), data.end(), [&distx, &disty, &distz](){
            geometry_msgs::Point p;
            p.x = distx(engine);
            p.y = disty(engine);
            p.z = distz(engine);
            return p;
        });
}

} // namespace random

} // namespace rosmath