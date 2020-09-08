#include "rosmath/opencv/conversions.h"
#include "rosmath/math.h"
#include "rosmath/misc.h"

#include <opencv2/calib3d.hpp>

namespace rosmath 
{

void convert(const Eigen::Matrix3d& from, cv::Mat& to)
{
    to = cv::Mat(3,3, CV_64F, double(0));
    for(int i=0; i<3; i++)
    {
        for(int j=0; j<3; j++)
        {
            to.at<double>(i,j) = from(i,j);
        }
    }
}

void convert(const cv::Mat& from, Eigen::Matrix3d& to)
{
    if(from.type() == CV_64F)
    {
        for(int i=0; i<3; i++)
        {
            for(int j=0; j<3; j++)
            {
                to(i,j) = from.at<double>(i,j);
            }
        }
    } else {
        // need to convert first
        cv::Mat from_converted;
        from.convertTo(from_converted, CV_64F);
        for(int i=0; i<3; i++)
        {
            for(int j=0; j<3; j++)
            {
                to(i,j) = from_converted.at<double>(i,j);
            }
        }
    }
}

void convert(
    const geometry_msgs::Quaternion& from, 
    cv::Mat& to)
{
    Eigen::Matrix3d fromeig;
    fromeig <<= from;
    convert(fromeig, to);   
}

void convert(const cv::Mat& from, geometry_msgs::Quaternion& to)
{
    Eigen::Matrix3d fromeig;
    convert(from, fromeig);
    convert(fromeig, to);
}

void convert(const geometry_msgs::Point& from, cv::Mat& to)
{
    to = cv::Mat_<double>(1, 3);
    to.at<double>(0) = from.x;
    to.at<double>(1) = from.y;
    to.at<double>(2) = from.z;
}

void convert(const geometry_msgs::Vector3& from, cv::Mat& to)
{
    to = cv::Mat_<double>(1,3);
    to.at<double>(0) = from.x;
    to.at<double>(1) = from.y;
    to.at<double>(2) = from.z;
}

void convert(const cv::Mat& from, geometry_msgs::Point& to)
{
    if(from.type() == CV_64F)
    {
        to.x = from.at<double>(0);
        to.y = from.at<double>(1);
        to.z = from.at<double>(2);
    } else {
        cv::Mat from_converted;
        from.convertTo(from_converted, CV_64F);
        to.x = from_converted.at<double>(0);
        to.y = from_converted.at<double>(1);
        to.z = from_converted.at<double>(2);
    }
}

void convert(const cv::Mat& from, geometry_msgs::Vector3& to)
{
    if(from.type() == CV_64F)
    {
        to.x = from.at<double>(0);
        to.y = from.at<double>(1);
        to.z = from.at<double>(2);
    } else {
        cv::Mat from_converted;
        from.convertTo(from_converted, CV_64F);
        to.x = from_converted.at<double>(0);
        to.y = from_converted.at<double>(1);
        to.z = from_converted.at<double>(2);
    }
}

void convertOptical(const geometry_msgs::Pose& from_optical, 
    cv::Mat& rvec, cv::Mat& tvec)
{
    geometry_msgs::Pose popt_inv = ~from_optical;
    cv::Mat R;
    convert(popt_inv.orientation, R);
    cv::Rodrigues(R, rvec);
    convert(popt_inv.position, tvec);
}

void convertOptical(const cv::Mat& rvec, const cv::Mat& tvec,
    geometry_msgs::Pose& to)
{
    cv::Mat R;
    cv::Rodrigues(rvec, R);
    geometry_msgs::Pose popt_inv;
    convert(R, popt_inv.orientation);
    convert(tvec, popt_inv.position);
    to = ~popt_inv;
}

void convert(const geometry_msgs::Pose& from, 
    cv::Mat& rvec, cv::Mat& tvec)
{
    geometry_msgs::Pose popt = ros2optical(from);
    convertOptical(popt, rvec, tvec);
}

void convert(const cv::Mat& rvec, const cv::Mat& tvec,
    geometry_msgs::Pose& to)
{
    geometry_msgs::Pose popt;
    convertOptical(rvec, tvec, popt);
    to = optical2ros(popt);
}

} // namespace rosmath
