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

    std::cout << cov << std::endl;

    random::Normal N(mean, cov);

    std::cout << "100000 Samples" << std::endl;
    Eigen::MatrixXd sX = N.samples(100000);
    std::cout << "Samples: " << std::endl;
    std::cout << sX.rows() << "x" << sX.cols() << std::endl;

    std::cout << "Fit Normal Distribution to X samples" << std::endl;
    random::Normal Nest = random::Normal::fit(sX);
    std::cout << Nest.mean() << std::endl;
    std::cout << Nest.cov() << std::endl;

    std::cout << "KLD N -> Nest" << std::endl;
    std::cout << N.kld(Nest) << std::endl;
    Eigen::VectorXd mean2(5);
    mean2.setZero();
    mean2(3) = 0.0;
    random::Normal Nbad(mean2, cov * 2.0);
    std::cout << N.kld(Nbad) << std::endl;

    std::cout << "Joint Distributions" << std::endl;
    auto Nmerged = N.joint(Nbad);

    std::cout << "Nmerged " << std::endl;

    std::cout << Nmerged.mean() << std::endl;
    std::cout << Nmerged.cov() << std::endl;

    // std::cout << "Fit Normal Distribution to X,Y samples" << std::endl;
    // auto sY = N.pdf(sX);
    // auto Nest2 = random::Normal::fit(sX, sY);

    return 0;
}