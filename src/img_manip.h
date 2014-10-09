#ifndef IMG_MANIP_H
#define IMG_MANIP_H

#include <opencv2/imgproc/imgproc.hpp>

namespace img {

void rotate(cv::Mat& img, double angle);
void rotate(const cv::Mat& src, cv::Mat& dst, double angle);
void rotate_no_resize(const cv::Mat& src, cv::Mat& dst, double angle);
void rotate_no_resize(cv::Mat& img, double angle);
void resize(const cv::Mat& src, cv::Mat& dst, double coef);
void resize(cv::Mat& img, double coef);

}

#endif