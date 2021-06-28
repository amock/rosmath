#include "rosmath/eigen/stats.h"
#include <unsupported/Eigen/MatrixFunctions>
#include "rosmath/stats.h"

namespace rosmath {

namespace stats {

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
        x(i) = dist(random::engine);
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
            samples(i,j) = dist(random::engine);
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
    throw std::runtime_error("TODO: implement");
    size_t dim = sX.rows();
    Eigen::VectorXd mean = sX.rowwise().mean();
    auto centered = sX.colwise() - mean;
    auto cov = (centered * centered.adjoint()) / double(sX.cols() - 1);
    return Normal(mean, cov);
}

Normal Normal::joint(const Normal& N) const
{
    Eigen::MatrixXd JI = (N.cov() + m_cov).inverse();   
    Eigen::MatrixXd cov = N.cov() * m_cov * JI;
    Eigen::VectorXd mean = N.cov() * JI * m_mean + m_cov * JI * N.mean();
    return Normal(mean, cov);
}

Normal Normal::transform(const Eigen::MatrixXd& T) const
{
    return Normal(T * m_mean, T * m_cov * T.transpose());
}

Normal joint(const std::vector<Normal>& Ns)
{
    Normal N = Ns[0];
    
    for(size_t i=1; i<Ns.size(); i++)
    {
        N = N.joint(Ns[i]);
    }

    return N;
}

double kullback_leibler_diveregence(const Normal& N1, const Normal& N2)
{
    // N1: N1, N2: N0
    double dim = N1.mean().rows();
    return 0.5 * ( ( N1.covInv() * N2.cov() ).trace() + mahalanobis_dist(N2.mean(), N1.covInv(), N1.mean()) - dim + std::log(N1.covDet() / N2.covDet()) );
}

double fisher_information(const Normal& N1, const Normal& N2)
{
    return (N2.covInv() * (N1.mean() - N2.mean())).squaredNorm() + (N2.covInv() * N2.covInv() * N1.cov() - 2.0 * N2.covInv() + N1.covInv() ).trace();
}

double wasserstein(
    const Normal& N1,
    const Normal& N2)
{
    Eigen::MatrixXd cov2sqrt = N2.cov().sqrt();
    return std::sqrt((N2.mean() - N1.mean()).squaredNorm() + 
            (
                N2.cov() + N1.cov() 
                - 2 * (
                    cov2sqrt * N1.cov() * cov2sqrt 
                ).sqrt()
            ).trace());
}

double hellinger(
    const Normal& N1,
    const Normal& N2)
{
    double combDet1 = (N2.cov() * N1.cov()).determinant();
    double combDet2 = ((N2.cov() + N1.cov())/2.0).determinant();
    return std::sqrt(2.0 - 2.0 * std::sqrt(std::sqrt(combDet1)) / std::sqrt(combDet2) 
       * std::exp(-1.0/4.0 * mahalanobis_dist(N2.mean(), (N2.cov() + N1.cov()).inverse(), N1.mean() ) ) );
}

} // namespace stats

} // namespace rosmath