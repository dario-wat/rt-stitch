#ifndef IMAGE_CONVERTER_CV_H
#define IMAGE_CONVERTER_CV_H

#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/Image.h>

namespace image_converter {

void to_cv_copy(cv::Mat& cvimg, const sensor_msgs::Image::ConstPtr& img);

}

#endif