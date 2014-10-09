#include "parrot_swarm.h"
#include "parrot.h"
#include "callbacks.h"
#include "img_manip.h"
#include <vector>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/stitching/stitcher.hpp>
#include <iostream>
#include <ctime>

typedef std::vector<Parrot*>::const_iterator Parr_it_const;
typedef std::vector<Parrot*>::iterator Parr_it;

const double D_ANGLE = 92.0;	// diagonal angle
const double L_ANGLE = 79.3;	// longer side angle
const double S_ANGLE = 63.7;	// shorter side angle

Parrot_Swarm::Parrot_Swarm() {}

Parrot_Swarm::Parrot_Swarm(const std::vector<Parrot*>& parrots)
	: parrots(parrots) {}

Parrot_Swarm::~Parrot_Swarm() {}

void Parrot_Swarm::add_parrot(Parrot* p) {
	this->parrots.push_back(p);
}

int Parrot_Swarm::size() const {
	return this->parrots.size();
}

void Parrot_Swarm::subscribe_all(ros::NodeHandle& n, const cbk::Callback_fun& callback) {
	for (Parr_it it = parrots.begin(); it != parrots.end(); it++) {
		(*it)->subscribe(n, callback);
	}
}

void Parrot_Swarm::clear_subscriptions() {
	for (Parr_it it = parrots.begin(); it != parrots.end(); it++) {
		(*it)->clear_subscribers();
	}
}

void Parrot_Swarm::extract_images(std::vector<cv::Mat>& dst) {
	dst.clear();
	for (Parr_it it = parrots.begin(); it != parrots.end(); it++) {
		dst.push_back((*it)->get_image());
	}
}

void Parrot_Swarm::show_separate(const Parrot_Swarm& swarm) {
	for (Parr_it_const it = swarm.parrots.begin(); it != swarm.parrots.end(); it++) {
		cv::imshow("stream: " + (*it)->get_name(), (*it)->get_image());
	}
	cv::waitKey(1);
}

void Parrot_Swarm::show_concat(const Parrot_Swarm& swarm) {
	int size = swarm.parrots.size();
	cv::Mat conc(img_const::HEIGHT, size*img_const::WIDTH, CV_8UC3, cv::Scalar(0, 0, 0));
	int i = 0;
	for (Parr_it_const it = swarm.parrots.begin(); it != swarm.parrots.end(); it++, i++) {
		const cv::Mat& tmp = (*it)->get_image();
		tmp.copyTo(conc(cv::Rect(img_const::WIDTH*i, 0, tmp.cols, tmp.rows)));
	}
	cv::imshow("concat", conc);
	cv::waitKey(1);
}

void Parrot_Swarm::show_stitch(const Parrot_Swarm& swarm) {
	std::vector<cv::Mat> images;
	for (Parr_it_const it = swarm.parrots.begin(); it != swarm.parrots.end(); it++) {
		images.push_back((*it)->get_image());
	}
	cv::Mat pano;
	cv::Stitcher st = cv::Stitcher::createDefault(true);
	size_t start = clock();
	cv::Stitcher::Status status = st.stitch(images, pano);
	size_t end = clock();
	if (status != cv::Stitcher::OK) return;
	std::cout << "stitchano: " << (float)(end - start) / CLOCKS_PER_SEC << std::endl;
	cv::imshow("stitch", pano);
	cv::waitKey(1);
}
