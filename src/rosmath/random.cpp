#include "Eigen/Dense"
#include "rosmath/random.h"

#include "rosmath/eigen/conversions.h"

namespace rosmath {

namespace random {

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

double mahalanobis_dist(
    const Eigen::VectorXd& mean, 
    const Eigen::MatrixXd& covInv,
    const Eigen::VectorXd& X)
{
    return (X - mean).transpose() * covInv * (X - mean);
}

Normal::Normal(const Eigen::VectorXd& mean, 
            const Eigen::MatrixXd& cov)
:m_mean(mean)
,m_cov(cov)
{
    m_cov_inv = m_cov.inverse();
    m_det = m_cov.determinant();

    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen_solver(cov);
    Eigen::MatrixXd eigenvectors = eigen_solver.eigenvectors().real();

    // Find the eigenvalues of the covariance matrix
    Eigen::MatrixXd eigenvalues = eigen_solver.eigenvalues().real().asDiagonal();
    
    // Find the transformation matrix
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(eigenvalues);
    Eigen::MatrixXd sqrt_eigenvalues = es.operatorSqrt();
    m_transform = eigenvectors * sqrt_eigenvalues;
}

Normal::~Normal()
{
    
}

double Normal::mahalanobisDist(const Eigen::VectorXd& X) const
{
    return mahalanobis_dist(m_mean, m_cov_inv, X);
}

double Normal::pdf(const Eigen::VectorXd& X) const
{
    double dim = X.rows();
    double invnorm = std::pow(SQRT2PI, dim) * std::sqrt(m_det);
    return exp(-0.5 * mahalanobisDist(X)) / invnorm;
}

Eigen::VectorXd Normal::pdf(const Eigen::MatrixXd& X) const
{
    size_t dim = X.rows();
    Eigen::VectorXd ret(X.cols());
    for(size_t i=0; i<X.cols(); i++)
    {
        ret(i) = pdf(Eigen::VectorXd(X.block(0,i,dim,1)));
    }
    return ret;
}

Eigen::VectorXd Normal::sample() const
{
    std::normal_distribution<double> dist(0.0, 1.0);
    size_t dim = m_mean.rows();
    Eigen::VectorXd x(dim);

    for(size_t i=0; i<dim; i++)
    {
        x(i) = dist(engine);
    }

    return m_transform * x + m_mean;
}

Eigen::MatrixXd Normal::samples(size_t N) const
{
    std::normal_distribution<double> dist(0.0, 1.0);
    size_t dim = m_mean.rows();
    Eigen::MatrixXd samples(dim, N);

    for(size_t i=0; i<dim; i++)
    {
        for(size_t j=0; j<N; j++)
        {
            samples(i,j) = dist(engine);
        }
    }

    return m_transform * samples + m_mean.replicate(1, N);
}

Eigen::VectorXd Normal::mean() const
{
    return m_mean;
}

Eigen::MatrixXd Normal::cov() const
{
    return m_cov;
}

Eigen::MatrixXd Normal::covInv() const
{
    return m_cov_inv;
}

double Normal::covDet() const
{
    return m_det;
}

Normal Normal::fit(const Eigen::MatrixXd& sX)
{
    size_t dim = sX.rows();
    Eigen::VectorXd mean = sX.rowwise().mean();
    auto centered = sX.colwise() - mean;
    auto cov = (centered * centered.adjoint()) / double(sX.cols() - 1);
    return Normal(mean, cov);
}

Normal Normal::fit(
        const Eigen::MatrixXd& sX, 
        const Eigen::VectorXd& sY)
{
    size_t dim = sX.rows();
    Eigen::VectorXd mean = sX.rowwise().mean();
    auto centered = sX.colwise() - mean;
    auto cov = (centered * centered.adjoint()) / double(sX.cols() - 1);
    return Normal(mean, cov);    
}

double Normal::kld(const Normal& N) const
{
    // this: N1, N: N0
    double dim = m_mean.rows();
    return 0.5 * ( ( m_cov_inv * N.cov() ).trace() + mahalanobis_dist(N.mean(), m_cov_inv, m_mean) - dim + std::log(m_det / N.covDet()) );
}

} // namespace random

} // namespace rosmath