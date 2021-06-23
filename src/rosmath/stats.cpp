#include "rosmath/stats.h"

namespace rosmath {

double mean(const std::vector<double>& data)
{
    double sum = std::accumulate(std::begin(data), std::end(data), 0.0);
    return sum / data.size();
}


double variance(
    const std::vector<double>& data,
    const double mean)
{
    double var = 0.0;

    std::for_each(std::begin(data), std::end(data), 
        [&](const double d) {
            var += (d - mean) * (d - mean);
        });

    return var / (data.size() - 1.0);
}

double variance(
    const std::vector<double>& data)
{
    double dmean = mean(data);
    return variance(data, dmean);
}

geometry_msgs::Point mean(
    const std::vector<geometry_msgs::Point>& points)
{
    geometry_msgs::Point mean;
    std::for_each(std::begin(points), std::end(points), [&](const geometry_msgs::Point p) {
        mean += p;
    });
    mean /= points.size();
    return mean;
}

geometry_msgs::Point variance(
    const std::vector<geometry_msgs::Point>& points,
    const geometry_msgs::Point mean)
{
    geometry_msgs::Point var;
    std::for_each(std::begin(points), std::end(points), 
            [&](const geometry_msgs::Point p) {
                var.x += (p.x - mean.x) * (p.x - mean.x);
                var.y += (p.y - mean.y) * (p.y - mean.y);
                var.z += (p.z - mean.z) * (p.z - mean.z);
            });
    return var / (points.size()-1);
}


geometry_msgs::Point variance(
    const std::vector<geometry_msgs::Point>& points)
{
    return variance(points, mean(points));
}

Eigen::Matrix3d covariance(
    const std::vector<geometry_msgs::Point>& points,
    const geometry_msgs::Point mean)
{
    Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor> > mat(&points[0].x, points.size(), 3);
    Eigen::Matrix<double,1,3> eig_mean;
    eig_mean(0,0) = mean.x;
    eig_mean(0,1) = mean.y;
    eig_mean(0,2) = mean.z;
    Eigen::MatrixXd centered = mat.rowwise() - eig_mean;
    return (centered.adjoint() * centered) / double(mat.rows() - 1);
}

Eigen::Matrix3d covariance(
    const std::vector<geometry_msgs::Point>& points)
{
    return covariance(points, mean(points));
}

void pca(
    const std::vector<geometry_msgs::Point>& points)
{
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es;

    auto cov = covariance(points);
    std::cout << "compute eigen value and vectors of " << std::endl;
    std::cout << cov << std::endl;

    es.compute(cov);

    std::cout << "Principal Components:" << std::endl;
    
    std::cout << "-- " << es.eigenvectors().col(2).transpose() 
                       << ": " << es.eigenvalues()(2) << std::endl;
    std::cout << "-- " << es.eigenvectors().col(1).transpose() 
                       << ": " << es.eigenvalues()(1) << std::endl;
    std::cout << "-- " << es.eigenvectors().col(0).transpose() 
                       << ": " << es.eigenvalues()(0) << std::endl;
}

} // namespace rosmath