#include "callbacks.h"
#include "parrot.h"
#include "image_converter.h"
#include <sensor_msgs/Image.h>
#include <opencv2/imgproc/imgproc.hpp>

namespace cbk {

void set_callback(const sensor_msgs::Image::ConstPtr& img, Parrot* const parrot) {
	cv::Mat cvimg;
	image_converter::to_cv_copy(cvimg, img);
	parrot->set_image(cvimg);
}

}
