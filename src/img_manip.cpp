#include "img_manip.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <cmath>

#define PI 3.14159265

namespace img {

void rotate(cv::Mat& img, double angle) {
	rotate(img, img, angle);
}

// computes new dimensions of a rotated image
void rotate_dim(int* height, int* width, double angle) {
	int h = *height / 2, w = *width / 2;
	double rad = angle * PI / 180.;
	int x1 = abs(w * cos(rad) - h * sin(rad));
	int y1 = abs(w * sin(rad) + h * cos(rad));
	h *= -1;
	int x2 = abs(w * cos(rad) - h * sin(rad));
	int y2 = abs(w * sin(rad) + h * cos(rad));
	*width = 2 * std::max(x1, x2);
	*height = 2 * std::max(y1, y2);
}

double first_c(double angle) {
	int n = abs(angle / 360);
	if (angle < 0) {
		return angle + (n+1) * 360;
	} else {
		return angle - n * 360;
	}
}

void rotate(const cv::Mat& src, cv::Mat& dst, double angle) {
	// new height and width computation
	int height = src.rows, width = src.cols;
	rotate_dim(&height, &width, angle);
	int len = std::max(height, width);
	int dh = (len - src.rows) / 2, dw = (len - src.cols) / 2;
	
	// copy image to empty one
	cv::Mat img(len, len, CV_8UC3, cv::Scalar(0, 0, 0));
	src.copyTo(img(cv::Rect(dw, dh, src.cols, src.rows)));
	
	// rotation
	cv::Mat tmp;
	cv::Point2f pt(len/2., len/2.);
	cv::Mat rot = cv::getRotationMatrix2D(pt, angle, 1.0);
	cv::warpAffine(img, tmp, rot, cv::Size(len, len));

	// final translation (to fit the frame)
	int diff = abs(dh - dw) - std::min(dh, dw);
	double nangle = first_c(angle);
	if (nangle >= 45 && nangle < 135 || nangle >= 225 && nangle < 315) {
		dst = tmp(cv::Rect(diff, 0, len-2*diff, len));
	} else {
		dst = tmp(cv::Rect(0, diff, len, len-2*diff));
	}
}

void rotate_no_resize(cv::Mat& img, double angle) {
	rotate_no_resize(img, img, angle);
}

void rotate_no_resize(const cv::Mat& src, cv::Mat& dst, double angle) {
	cv::Point2f pt(src.cols/2., src.rows/2.);
	cv::Mat rot = cv::getRotationMatrix2D(pt, angle, 1.0);
	cv::warpAffine(src, dst, rot, cv::Size(src.cols, src.rows));
}

void resize(cv::Mat& img, double coef) {
	resize(img, img, coef);
}

void resize(const cv::Mat& src, cv::Mat& dst, double coef) {
	cv::resize(src, dst, cv::Size(src.cols * coef, src.rows * coef));
}

}