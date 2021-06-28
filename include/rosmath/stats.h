#ifndef ROSMATH_STATS_H
#define ROSMATH_STATS_H

#include "math.h"
#include <random>
#include <cmath>

namespace rosmath {

namespace stats {

constexpr static double SQRT2PI = std::sqrt(2 * M_PI);

} // namespace stats

template<typename ...Tp>
struct Stats : public Tp... 
{
public:
    static constexpr std::size_t N = sizeof...(Tp);
    using elems = std::tuple<Tp...>;

private:
    template <typename T, typename Tuple>
    struct has_type;

    template <typename T>
    struct has_type<T, std::tuple<>> : std::false_type {};

    template <typename T, typename U, typename... Ts>
    struct has_type<T, std::tuple<U, Ts...>> : has_type<T, std::tuple<Ts...>> {};


    template <typename T, typename... Ts>
    struct has_type<T, std::tuple<T, Ts...>> : std::true_type {};    

    template<typename F> 
    struct has_elem {
        static constexpr bool value = has_type<F, elems>::type::value;
    };

public:
    /**
     * @brief Check if Intersection container has a specific intelem (lvr2::intelem).
     * 
     * @tparam F lvr2::intelem type
     * @return true 
     * @return false 
     */
    template<typename F>
    static constexpr bool has() {
        return has_elem<F>::value;
    }
};

struct PointMean {
    geometry_msgs::Point mean;
};

struct PointVariance {
    geometry_msgs::Point variance;
};

struct PointCovariance {
    Eigen::Matrix3d covariance;
};

double mean(
    const std::vector<double>& data);

double variance(
    const std::vector<double>& data,
    const double mean);

double variance(
    const std::vector<double>& data);

geometry_msgs::Point mean(
    const std::vector<geometry_msgs::Point>& points);

geometry_msgs::Point variance(
    const std::vector<geometry_msgs::Point>& points,
    const geometry_msgs::Point mean
);

geometry_msgs::Point variance(
    const std::vector<geometry_msgs::Point>& points
);

Eigen::Matrix3d covariance(
    const std::vector<geometry_msgs::Point>& points,
    const geometry_msgs::Point mean);

Eigen::Matrix3d covariance(
    const std::vector<geometry_msgs::Point>& points);

void pca(
    const std::vector<geometry_msgs::Point>& points);

template<typename ...Tp>
Stats<Tp...> calculate_stats(
    const std::vector<geometry_msgs::Point>& points)
{
    using StatsType = Stats<Tp...>;
    StatsType ret;

    if constexpr(StatsType::template has<PointMean>())
    {
        ret.mean = mean(points);
    }

    // calculate variance
    if constexpr(StatsType::template has<PointVariance>())
    {
        // check if mean was already computed
        if constexpr(StatsType::template has<PointMean>())
        {
            ret.variance = variance(points, ret.mean);
        } else {
            // calculate mean first
            ret.variance = variance(points);
        }
    }

    // calculate covariance
    if constexpr(StatsType::template has<PointCovariance>())
    {
        // check if mean was already computed
        if constexpr(StatsType::template has<PointMean>())
        {
            ret.covariance = covariance(points, ret.mean);
        } else {
            // calculate mean first
            ret.covariance = covariance(points);
        }
    }
    
    return ret;
}

} // namespace rosmath

#endif // ROSMATH_STATS_H