#include "stitcher.hpp"
#include "parrot_swarm.h"
#include "parrot.h"
#include "utils.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/stitching/stitcher.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <vector>
#include <thread>
#include <mutex>
#include <ctime>
#include <iostream>
#include <unistd.h>

#define MIN_HS 400
#define MAX_HEIGHT 690
#define MAX_WIDTH 920

long time_in_ms() {
 	long            ms; // Milliseconds
    time_t          s;  // Seconds
    struct timespec spec;

    clock_gettime(CLOCK_REALTIME, &spec);

    s  = spec.tv_sec;
    ms = round(spec.tv_nsec / 1.0e6);
    return s*1000+ms;
}

Stitching_Strategy::Stitching_Strategy(Parrot_Swarm* swarm) :
	swarm(swarm),
	image(img_const::HEIGHT, img_const::WIDTH, CV_8UC3, cv::Scalar(0, 0, 0)) {}

void Stitching_Strategy::save(const std::vector<cv::Mat>& imgs) const {
	std::string t = get_current_time();
	for (int i = 0; i < imgs.size(); i++) {
		cv::imwrite("slika" + to_string(i) + "_" + t + ".jpg", imgs[i]);
	}
	cv::imwrite("stitchana_" + t + ".jpg", this->image);
}

void Stitching_Strategy::set_image(const cv::Mat& img) {
	//std::cout << "[Time] New image: " << (float)clock() / CLOCKS_PER_SEC << "s" << std::endl;
	//std::cout << clock()/CLOCKS_PER_SEC << std::endl;
	//std::cout.precision(15);
	//std::cout << "Postavljeno: " << (double)time_in_ms()/1000.0 << "s" << std::endl;
	this->image = img;
}

void Stitching_Strategy::get_image(cv::Mat& img) {
	img = this->image;
}

Multi_stitch::Multi_stitch(Parrot_Swarm* swarm, int n) : Stitching_Strategy(swarm) {
	this->n = n;
}

void Multi_stitch::ser_stitch(std::mutex* mtx) {
	cv::Mat pano;
	cv::Stitcher st = cv::Stitcher::createDefault(true);
	st.setWarper(new cv::PlaneWarper());
	st.setFeaturesFinder(new cv::detail::SurfFeaturesFinder(100,3,4,3,4));
	st.setRegistrationResol(0.1);
	st.setSeamEstimationResol(0.1);
	st.setCompositingResol(1);
	st.setPanoConfidenceThresh(0.5);
	st.setWaveCorrection(true);
	st.setWaveCorrectKind(cv::detail::WAVE_CORRECT_HORIZ);
	st.setFeaturesMatcher(new cv::detail::BestOf2NearestMatcher(false, 0.3));
	st.setBundleAdjuster(new cv::detail::BundleAdjusterRay());
	st.setSeamFinder(new cv::detail::GraphCutSeamFinder(cv::detail::GraphCutSeamFinderBase::COST_COLOR));
	st.setBlender(cv::detail::Blender::createDefault(cv::detail::Blender::MULTI_BAND, false));
	st.setExposureCompensator(cv::detail::ExposureCompensator::createDefault(cv::detail::ExposureCompensator::NO));

	std::vector<cv::Mat> imgs;
	while (true) {
		try {
			//usleep(100000);
			swarm->extract_images(imgs);

			long start = time_in_ms();
			cv::Stitcher::Status status = st.stitch(imgs, pano);
			long end = time_in_ms();
			
			if (status != cv::Stitcher::OK) continue;
			cv::Size siz = pano.size();
			int h = siz.height, w = siz.width;
			if (h > 900 || w > 1300) continue;


			cv::Mat tmp;
			cv::Mat def;
			get_image(def);
			center_image(pano, tmp, def);
			mtx->lock();
			long stop = time_in_ms();
			if (stop > now) {
				now = stop;
				set_image(tmp);
			}
			mtx->unlock();
			//save(imgs);
		} catch (...) {
			continue;
		}
	}
}

void Multi_stitch::stitch() {
	now = time_in_ms();
	std::mutex *mtx = new std::mutex();
	std::vector<std::thread*> threads;
	for (int i = 0; i < n; i++) {
		std::thread *t = new std::thread(&Multi_stitch::ser_stitch, this, mtx);
		threads.push_back(t);
	}
	for (int i = 0; i < n; i++) {
		threads[i]->join();
	}
	delete mtx;	
}

void center_image(const cv::Mat& src, cv::Mat& dst, const cv::Mat& def) {
	cv::Mat tmp(MAX_HEIGHT+50, MAX_WIDTH+50, CV_8UC3, cv::Scalar(0, 0, 0));
	int h = src.size().height, w = src.size().width;
	int dh = MAX_HEIGHT - h, dw = MAX_WIDTH - w;
	if (dh < 0 || dw < 0) {
		dst = def;
		return;
	}
	cv::Rect r(std::max(0,dh/2-1), std::max(0,dw/2-1), src.cols, src.rows);
	try {
		cv::Mat center(tmp, r);
		src.copyTo(center);
	} catch (cv::Exception& e) {
		dst = def;
		return;
	}
	dst = tmp;
}

Stitching::Stitching(Parrot_Swarm* swarm) : Stitching_Strategy(swarm) {}

void Stitching::ser_stitch(std::mutex* mtx) {
	cv::Mat result;
	while (true) {
		try {
			stitching(swarm, result);
		} catch (cv::Exception e) {
			continue;
		}
		mtx->lock();
		set_image(result);
		mtx->unlock();
	}
}

void Stitching::stitch() {
	int n = 1;
	std::mutex* mtx = new std::mutex;
	std::vector<std::thread*> threads;
	for (int i = 0; i < n; i++) {
		std::thread* t = new std::thread(&Stitching::ser_stitch, this, mtx);
		threads.push_back(t);
	}
	for (int i = 0; i < n; i++) {
		threads[i]->join();
		delete threads[i];
	}
	delete mtx;
}

void Stitching::stitching(Parrot_Swarm* swarm, cv::Mat& dst) {
	std::vector<cv::Mat> imgs;
	swarm->extract_images(imgs);
	cv::Mat img1 = imgs[0];
	cv::Mat img2 = imgs[1];

	cv::Mat gray_img1;
	cv::Mat gray_img2;
	cvtColor(img1, gray_img1, CV_RGB2GRAY);
	cvtColor(img2, gray_img2, CV_RGB2GRAY);

	int minhs = 400;
	cv::SurfFeatureDetector detector(minhs);
	std::vector<cv::KeyPoint> keypoints1, keypoints2;
	detector.detect(gray_img1, keypoints1);
	detector.detect(gray_img2, keypoints2);

	cv::SurfDescriptorExtractor extractor;
	cv::Mat descr1, descr2;
	extractor.compute(gray_img1, keypoints1, descr1);
	extractor.compute(gray_img2, keypoints2, descr2);
	
	cv::FlannBasedMatcher matcher;
	std::vector<cv::DMatch> matches;
	matcher.match(descr1, descr2, matches);
	
	double max_dist = 0, min_dist = 100;
	for( int i = 0; i < descr1.rows; i++ ) {
		double dist = matches[i].distance;
		if( dist < min_dist ) min_dist = dist;
		if( dist > max_dist ) max_dist = dist;
	}
	//std::cout << "-- Max dist: " << max_dist << std::endl;
	//std::cout << "-- Min dist: " << min_dist << std::endl;

	std::vector<cv::DMatch> good_matches;
	for (int i = 0; i < descr1.rows; i++) {
		if (matches[i].distance <= std::max(2*min_dist, 0.02)) {
			good_matches.push_back(matches[i]);
		}
	}
	if (good_matches.size() <= 20) {
		get_image(dst);
		return;
	}
	std::cout << "Good matches: " << good_matches.size() << std::endl;

	std::vector<cv::Point2f> obj, scene;
	for (int i = 0; i < good_matches.size(); i++) {
		obj.push_back(keypoints1[good_matches[i].queryIdx].pt);
		scene.push_back(keypoints2[good_matches[i].trainIdx].pt);
	}

	cv::Mat h = cv::findHomography(obj, scene, CV_RANSAC);
	cv::Mat result;
	cv::warpPerspective(img1, result, h, cv::Size(img1.cols + img2.cols, img1.rows));
	cv::Mat half(result, cv::Rect(0, 0, img2.cols, img2.rows));
	img2.copyTo(half);
	//cv::imshow("stitch", result);
	//cv::waitKey(1);
	dst = result;
}