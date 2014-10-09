#include "parrot.h"
#include "callbacks.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <boost/bind.hpp>

const char* SUB_NAME = "/ardrone/front/image_raw";

Point::Point() : x(0), y(0) {}

Point::Point(double x, double y) : x(x), y(y) {}

Parrot::Parrot() : x(0), y(0), z(0), angle(0), name(""), 
	image(img_const::HEIGHT, img_const::WIDTH, CV_8UC3, cv::Scalar(0, 0, 0)) {}

Parrot::Parrot(const std::string& name) : x(0), y(0), z(0), angle(0), name(name),
	image(img_const::HEIGHT, img_const::WIDTH, CV_8UC3, cv::Scalar(0, 0, 0)){}

Parrot::Parrot(double x, double y, double z, double angle)
	: x(x), y(y), z(z), angle(angle), name(""), 
	image(img_const::HEIGHT, img_const::WIDTH, CV_8UC3, cv::Scalar(0, 0, 0)) {
}

Parrot::Parrot(double x, double y, double z, double angle, const cv::Mat& image)
	: Parrot(x, y, z, angle) {
	this->image = image;
}

Parrot::~Parrot() {}

void Parrot::set_name(const std::string& name) {
	this->name = name;
}

void Parrot::set_image(const cv::Mat& image) {
	this->image = image;
}

void Parrot::set_coords(double x, double y, double z, double angle) {
	this->x = x;
	this->y = y;
	this->z = z;
	this->angle = angle;
}

void Parrot::set_x(double x) {
	this->x = x;
}

void Parrot::set_y(double y) {
	this->y = y;
}

void Parrot::set_z(double z) {
	this->z = z;
}

void Parrot::set_angle(double angle) {
	this->angle = angle;
}

void Parrot::subscribe(ros::NodeHandle& n, const cbk::Callback_fun& callback) {
	subscribers.push_back(n.subscribe<sensor_msgs::Image>(
		"/" + this->name + SUB_NAME,
		10,
		boost::bind(callback, _1, this)
		)
	);
}

const std::string& Parrot::get_name() const {
	return this->name;
}

double Parrot::get_x() const {
	return this->x;
}

double Parrot::get_y() const {
	return this->y;
}

double Parrot::get_z() const {
	return this->z;
}

double Parrot::get_angle() const {
	return this->angle;
}

const cv::Mat& Parrot::get_image() const {
	return this->image;
}

void Parrot::clear_subscribers() {
	subscribers.clear();
}