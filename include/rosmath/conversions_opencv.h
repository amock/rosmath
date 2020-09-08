#ifndef ROSMATH_CONVERSIONS_OPENCV_H
#define ROSMATH_CONVERSIONS_OPENCV_H

#include "conversions.h"
#include "math.h"
#include "misc.h"
#include <opencv2/core.hpp>

namespace rosmath 
{

void convert(const Eigen::Matrix3d& from, cv::Mat& to);
void convert(const cv::Mat& from, Eigen::Matrix3d& to);

void convert(const geometry_msgs::Quaternion& from, cv::Mat& to);
void convert(const cv::Mat& from, geometry_msgs::Quaternion& to);

void convert(const geometry_msgs::Point& from, cv::Mat& to);
void convert(const geometry_msgs::Vector3& from, cv::Mat& to);

void convert(const cv::Mat& from, geometry_msgs::Point& to);
void convert(const cv::Mat& from, geometry_msgs::Vector3& to);

/**
 * @brief Calculates the corresponding rvec, tvec from an optical ROS pose (x-right, y-down, z-front).
 * The rvec, tvec can be used to project points etc. Therefore, the rvec, tvec are the inverse of the ROS pose.
 * 
 * @param from[in] pose in optical coordinates. 
 * @param rvec[out] OpenCV rotation: inverse of the ROS pose
 * @param tvec[out] OpenCV translation: inverse of the ROS pose
 */
void convertOptical(const geometry_msgs::Pose& from, 
    cv::Mat& rvec, cv::Mat& tvec);

/**
 * @brief Calculates the corresponding optical ROS pose (x-right, y-down, z-front) from an OpenCV rotation(rvec) and translation(tvec).
 * The rvec, tvec can be used to project points etc. Therefore, the rvec, tvec are the inverse of the ROS pose.
 *  
 * @param[in] rvec OpenCV rotation: inverse of the ROS pose
 * @param[in] tvec OpenCV translation: inverse of the ROS pose
 * @param[out] optical ROS pose
 */
void convertOptical(const cv::Mat& rvec, const cv::Mat& tvec,
    geometry_msgs::Pose& to);

/**
 * @brief ROS pose to OpenCV rotation(rvec) and translation(tvec)
 * In difference to convertOptical, this function takes a standard ROS pose: x-front, y-left, z-up
 * 
 * @param from[in] ROS pose
 * @param rvec[out] OpenCV rotation
 * @param tvec[out] OpenCV translation
 */
void convert(const geometry_msgs::Pose& from, 
    cv::Mat& rvec, cv::Mat& tvec);

/**
 * @brief OpenCV rotation(rvec) and translation(tvec) to ROS pose (x-front, y-left, z-up)
 * 
 * @param from[in] ROS pose
 * @param rvec[out] OpenCV rotation
 * @param tvec[out] OpenCV translation
 */
void convert(const cv::Mat& rvec, const cv::Mat& tvec,
    geometry_msgs::Pose& to);


//convert non optical
// void convert(const geometry_msgs::Pose& from, 
//     cv::Mat& rvec, cv::Mat& tvec);


} // namespace rosmath

#endif // ROSMATH_CONVERSIONS_OPENCV_H