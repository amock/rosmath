#ifndef ROSMATH_EIGEN_STATS_H
#define ROSMATH_EIGEN_STATS_H

#include <rosmath/random.h>
#include <Eigen/Dense>
#include <vector>

namespace rosmath {

namespace stats {

double mahalanobis_dist(
    const Eigen::VectorXd& mean, 
    const Eigen::MatrixXd& covInv,
    const Eigen::VectorXd& X);

/**
 * @brief General Normal Distribution Class
 * 
 * Improved Version of http://blog.sarantop.com/notes/mvn
 * 
 * TODO: 
 * - Test class functions for correctness
 * - Implement multivariate fit to X and Y values
 * - Implement some other helpful functions:
 *   - Joint (check), Marginal probability distributions
 *   - Mutual Information
 *   - Self Information
 *   - Shannon Entropy
 *   - Conditional Entropy
 *   - Cross Entropy
 *   - Quantum Relative Entropy
 *   - Bayesian Update
 *   - Fisher Information (Matrix)
 * 
 */
class Normal {
public:
    Normal(const Eigen::VectorXd& mean, 
            const Eigen::MatrixXd& cov);

    ~Normal();

    double mahalanobisDist(const Eigen::VectorXd& X) const;

    // N(X)
    double pdf(const Eigen::VectorXd& X) const;

    Eigen::VectorXd pdf(const Eigen::MatrixXd& X) const;

    Eigen::VectorXd mean() const;
    Eigen::MatrixXd cov() const;
    Eigen::MatrixXd covInv() const;
    double covDet() const;
    
    // sample
    Eigen::VectorXd sample() const;

    /**
     * samples
     * 
     * returns MatrixXd. rows: dim, cols: N
     */
    Eigen::MatrixXd samples(size_t N) const;

    /**
     * Fit to X data
     * 
     * sX is a MatrixXd. rows: dim, cols: N samples
     */
    static Normal fit(const Eigen::MatrixXd& sX);

    /**
     * Fit to X and Y data
     * 
     * TODO: implement
     * 
     */
    static Normal fit(
        const Eigen::MatrixXd& sX, 
        const Eigen::VectorXd& sY);

    /**
     * Calculate the Joint Distribution of this and another
     * 
     * TODO: check if the implementation is correct
     * 
     * Is this maybe a bayesian update instead of a joint?
     * 
     */
    Normal joint(const Normal& N) const;

    Normal transform(const Eigen::MatrixXd& T) const;

private:
    Eigen::VectorXd m_mean;
    Eigen::MatrixXd m_cov;

    // other

    // PDF
    Eigen::MatrixXd m_cov_inv;
    double m_det;

    // Transform for samples
    Eigen::MatrixXd m_transform;
};

Normal joint(const std::vector<Normal>& Ns);

/**
 * Kullback-Leibler Divergence from N1 to N2
 * source: https://en.wikipedia.org/wiki/Multivariate_normal_distribution#Kullback%E2%80%93Leibler_divergence
 */
double kullback_leibler_diveregence(
    const Normal& N1, 
    const Normal& N2);

/**
 * Fisher-Information
 * source: https://djalil.chafai.net/blog/2021/02/22/fisher-information-between-two-gaussians/
 */
double fisher_information(
    const Normal& N1,
    const Normal& N2);

double wasserstein(
    const Normal& N1,
    const Normal& N2);

double hellinger(
    const Normal& N1,
    const Normal& N2);

} // namespace stats

} // namespace rosmath

#endif // ROSMATH_EIGEN_STATS_H