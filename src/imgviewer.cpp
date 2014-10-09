#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/highgui/highgui.hpp>
#include <boost/bind.hpp>
#include "parrot.h"
#include "parrot_swarm.h"
#include "stitcher.hpp"
#include <typeinfo>
#include <thread>
#include "getrss.c"
#include <unistd.h>		// usleep

#define THREAD_COUNT 8

using namespace std;
using namespace cv;
using namespace ros;

const char* WINDOW_NAME = "stream";
const int BUFF_SIZE = 10;
const int RATE = 20;

void mem_printer() {
	while (true) {
		usleep(200000);
		cout << "MEM:  " << getCurrentRSS()/(1024*1024) << " MB" << endl;
		cout << "PEAK: " << getPeakRSS()/(1024*1024) << " MB" << endl;
	}
}

int main(int argc, char** argv) {
	init(argc, argv, "stitcher");
	NodeHandle n;
	Parrot parrot("quad1");
	Parrot parrot2("quad2");

	Parrot_Swarm imgh;
	imgh.add_parrot(&parrot);
	imgh.add_parrot(&parrot2);
	imgh.subscribe_all(n, cbk::set_callback);

	Multi_stitch ss(&imgh, THREAD_COUNT);
	thread t(&Multi_stitch::stitch, &ss);
	//Stitching ss(&imgh);
	//thread t(&Stitching::stitch, &ss);

	//thread pr(mem_printer);

	Rate loop_rate(RATE);
	Mat dst;
	
	namedWindow("stitch");
	moveWindow("stitch", 330, 0);
	namedWindow("stream: quad1");
	moveWindow("stream: quad1", 0, 0);
	namedWindow("stream: quad2");
	moveWindow("stream: quad2", 0, 290);
	
	while (ok()) {
		spinOnce();
		loop_rate.sleep();
		waitKey(1);
		Parrot_Swarm::show_separate(imgh);
		ss.get_image(dst);
		imshow("stitch", dst);
		
	}
	t.join();
	//pr.join();
	return 0;
}

/*
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/nonfree/ocl.hpp"
#include <opencv2/ocl/ocl.hpp>
#include "opencv2/calib3d/calib3d.hpp"
#include <vector>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/stitching/stitcher.hpp>
#include <opencv2/nonfree/ocl.hpp>

void show_stitch(const std::vector<cv::Mat>& images) {
	cv::Mat pano;
	cv::Stitcher st = cv::Stitcher::createDefault(true);
	st.stitch(images, pano);
	cv::imshow("stitch", pano);
	cv::waitKey(0);
}

int main(int argc, char** argv) {
	cv::Mat img;
	std::vector<cv::Mat> imgs;
	for (int i = 1; i < argc; i++) {
		img = cv::imread(argv[i]);
		imgs.push_back(img);
	}
	cout << "doso sam do ovdje" << endl;
	ocl::oclMat m1(imgs[0]);
	cout << "doso sam do ovdje" << endl;
	ocl::oclMat m2(imgs[1]);
	ocl::oclMat d;
	ocl::add(m1, m2, d);
	cv::Mat d2(d);
	cv::imshow("slika", d2);
	show_stitch(imgs);
	return 0;
}
*/