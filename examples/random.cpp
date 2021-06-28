#include <ros/ros.h>
#include <iostream>
#include <rosmath/random.h>
#include <rosmath/stats.h>
#include <rosmath/eigen/stats.h>
#include <time.h>

using namespace rosmath;

void random_simple()
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


    auto st = calculate_stats<PointMean, PointVariance>(points);

    ROS_INFO_STREAM(st.mean);
    ROS_INFO_STREAM(st.variance);

    pca(points);
}

void normal_sampling_fits()
{


    // Multivariate Normal
    std::cout << "Multivariate Normal" << std::endl;

    Eigen::VectorXd mean(5);
    Eigen::MatrixXd cov(5,5);
    cov.setIdentity();
    mean.setZero();
    mean(3) = 2.0;

    std::cout << cov << std::endl;

    stats::Normal N(mean, cov);

    std::cout << "100000 Samples" << std::endl;
    Eigen::MatrixXd sX = N.samples(100000);
    std::cout << "Samples: " << std::endl;
    std::cout << sX.rows() << "x" << sX.cols() << std::endl;

    std::cout << "Fit Normal Distribution to X samples" << std::endl;
    stats::Normal Nest = stats::Normal::fit(sX);
    std::cout << Nest.mean() << std::endl;
    std::cout << Nest.cov() << std::endl;

    // TODO: multivariate fit to X and Y
}

void normal_distribution_measures()
{
    Eigen::VectorXd mu1(3), mu2(3), mu3(3);
    Eigen::MatrixXd cov1(3,3), cov2(3,3), cov3(3,3);
    mu1.setZero();
    mu2.setZero();
    mu3.setZero();
    cov1.setIdentity();
    cov2.setIdentity();
    cov3.setIdentity();

    mu1(0) = 1.0;
    mu2(0) = 2.0;
    mu3(0) = 0.0;
    cov2 *= 2.0;
    cov3 /= 2.0;

    stats::Normal N1(mu1, cov1);
    stats::Normal N2(mu2, cov2);
    stats::Normal N3(mu3, cov3);

    std::cout << "N1(1, 1), N2(2, 2), N3(0, 0.5)" << std::endl;

    std::cout << "Fisher: " << std::endl;
    std::cout << " - N1 -> N2: " << stats::fisher_information(N1, N2) << std::endl;
    std::cout << " - N2 -> N1: " << stats::fisher_information(N2, N1) << std::endl;
    std::cout << " - N1 -> N1: " << stats::fisher_information(N1, N1) << std::endl;
    std::cout << " - N1 -> N3: " << stats::fisher_information(N1, N3) << std::endl;
    std::cout << " - N3 -> N1: " << stats::fisher_information(N3, N1) << std::endl;

    std::cout << "KLD: " << std::endl;
    std::cout << " - N1 -> N2: " << stats::kullback_leibler_diveregence(N1, N2) << std::endl;
    std::cout << " - N2 -> N1: " << stats::kullback_leibler_diveregence(N2, N1) << std::endl;
    std::cout << " - N1 -> N1: " << stats::kullback_leibler_diveregence(N1, N1) << std::endl;
    std::cout << " - N1 -> N3: " << stats::kullback_leibler_diveregence(N1, N3) << std::endl;
    std::cout << " - N3 -> N1: " << stats::kullback_leibler_diveregence(N3, N1) << std::endl;

    std::cout << "Wasserstein: " << std::endl;
    std::cout << " - N1 -> N2: " << stats::wasserstein(N1, N2) << std::endl;
    std::cout << " - N2 -> N1: " << stats::wasserstein(N2, N1) << std::endl;
    std::cout << " - N1 -> N1: " << stats::wasserstein(N1, N1) << std::endl;

    std::cout << "Hellinger: " << std::endl;
    std::cout << " - N1 -> N2: " << stats::hellinger(N1, N2) << std::endl;
    std::cout << " - N2 -> N1: " << stats::hellinger(N2, N1) << std::endl;
    std::cout << " - N1 -> N1: " << stats::hellinger(N1, N1) << std::endl;
}

int main(int argc, char** argv)
{
    random_simple();
    normal_sampling_fits();


    normal_distribution_measures();


    return 0;
}