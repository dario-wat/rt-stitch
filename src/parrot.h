#ifndef PARROT_H
#define PARROT_H

#include <opencv2/imgproc/imgproc.hpp>
#include <string>
#include <vector>
#include <ros/ros.h>
#include "callbacks.h"

namespace img_const {

const int WIDTH = 320;
const int HEIGHT = 240;

}

class Point {
public:
	const double x, y;
	Point();
	Point(double x, double y);
};

class Coverage {
public:
	const Point a, b;
	const double angle;
};

class Parrot {
private:
	double x, y, z, angle;
	cv::Mat image;
	std::string name;
	std::vector<ros::Subscriber> subscribers;
public:
	Parrot();
	Parrot(const std::string& name);
	Parrot(double x, double y, double z, double angle);
	Parrot(double x, double y, double z, double angle, const cv::Mat& image);
	~Parrot();
	void set_name(const std::string& name);
	void set_image(const cv::Mat& image);
	void set_coords(double x, double y, double z, double angle);
	void set_x(double x);
	void set_y(double y);
	void set_z(double z);
	void set_angle(double angle);
	void subscribe(ros::NodeHandle& n, const cbk::Callback_fun& callback);
	const std::string& get_name() const;
	double get_x() const;
	double get_y() const;
	double get_z() const;
	double get_angle() const;
	const cv::Mat& get_image() const;
	void clear_subscribers();
};

#endif