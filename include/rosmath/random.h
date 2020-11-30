#ifndef ROSMATH_RANDOM_H
#define ROSMATH_RANDOM_H

#include "math.h"
#include <random>



namespace rosmath {

namespace random {

std::default_random_engine engine;

void seed(size_t seed)
{
    engine = std::default_random_engine(seed);
}

double uniform_number(
    const double min,
    const double max);

std::vector<double> uniform_numbers(
    const double min,
    const double max,
    const size_t size);

double uniform_angle();

geometry_msgs::Point uniform_point(
    const geometry_msgs::Point pmin, 
    const geometry_msgs::Point pmax);

std::vector<geometry_msgs::Point> uniform_points(
    const geometry_msgs::Point pmin, 
    const geometry_msgs::Point pmax,
    const size_t size
);

geometry_msgs::Quaternion uniform_quaternion();
geometry_msgs::Quaternion uniform_quaternion(
    const geometry_msgs::Vector3 axis);


void uniform_fill(
    std::vector<double>& data, 
    const double min,
    const double max);

void uniform_fill(
    std::vector<geometry_msgs::Point>& data, 
    const geometry_msgs::Point pmin, 
    const geometry_msgs::Point pmax);

double normal_number(
    const double mu,
    const double sigma);

geometry_msgs::Point normal_point(
    const geometry_msgs::Point mu,
    const geometry_msgs::Point sigma);

geometry_msgs::Quaternion normal_quaternion(
    const double sx, const double sy, const double sz);

std::vector<double> normal_numbers(
    const double mu, 
    const double sigma,
    const size_t size);

std::vector<geometry_msgs::Point> normal_points(
    const geometry_msgs::Point mu,
    const geometry_msgs::Point sigma,
    const size_t size);

void normal_fill(
    std::vector<double>& data,
    const double mu,
    const double sigma);

void normal_fill(
    std::vector<geometry_msgs::Point>& data,
    const geometry_msgs::Point mu,
    const geometry_msgs::Point sigma);

} // namespace random

} // namespace rosmath

#endif // ROSMATH_RANDOM_H