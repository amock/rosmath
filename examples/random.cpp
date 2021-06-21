#include <ros/ros.h>
#include <iostream>
#include <rosmath/random.h>
#include <rosmath/stats.h>
#include <time.h>

using namespace rosmath;

int main(int argc, char** argv)
{
    size_t seed = time(NULL);

    random::seed(seed);

    std::vector<double> rand_vec_1 = random::uniform_numbers(-1.0, 1.0, 10);
    for(size_t i=0; i<rand_vec_1.size(); i++)
    {
        std::cout << rand_vec_1[i] << " ";
    }
    std::cout << std::endl;

    random::seed(seed);

    std::vector<double> rand_vec_2 = random::uniform_numbers(-1.0, 1.0, 10);
    for(size_t i=0; i<rand_vec_2.size(); i++)
    {
        std::cout << rand_vec_2[i] << " ";
    }
    std::cout << std::endl;


    geometry_msgs::Quaternion q = random::normal_quaternion(1.0, 0.0, 0.0);

    ROS_INFO_STREAM(q);

    geometry_msgs::Point mu, sigma;
    mu.x = 1.0;
    mu.y = 2.0;
    mu.z = 0.0;

    sigma.x = 0.5;
    sigma.y = 1.0;
    sigma.z = 0.0;

    std::vector<geometry_msgs::Point> points = random::normal_points(mu, sigma, 1000000);

    // geometry_msgs::Point p_mean = mean(points);
    // ROS_INFO_STREAM(p_mean);


    auto st = stats<PointMean, PointVariance>(points);

    ROS_INFO_STREAM(st.mean);
    ROS_INFO_STREAM(st.variance);

    pca(points);


    // Multivariate Normal
    std::cout << "Multivariate Normal" << std::endl;

    Eigen::VectorXd mean(5);
    Eigen::MatrixXd cov(5,5);
    cov.setIdentity();
    mean.setZero();
    mean(3) = 2.0;

    random::Normal N(mean, cov);

    std::cout << "100000 Samples" << std::endl;
    // std::vector<Eigen::VectorXd> samples;
    // for(size_t i=0; i<1000; i++)
    // {
    //     samples.push_back(N.sample());
    // }

    Eigen::MatrixXd samples2 = N.samples(100000);
    std::cout << samples2.rows() << "x" << samples2.cols() << std::endl;

    std::cout << "Calculate Mean: " << std::endl;
    Eigen::VectorXd mean2(5);
    mean2.setZero();

    for(size_t i=0; i<100000; i++)
    {
        auto sample =  samples2.block<5,1>(0,i);
        mean2 += sample;
    }

    mean2 /= 100000.0;

    std::cout << mean2 << std::endl;


    return 0;
}