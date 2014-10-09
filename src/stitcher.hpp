#ifndef STITCHING_STRATEGIES
#define STITCHING_STRATEGIES

#include "parrot_swarm.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/stitching/stitcher.hpp>
#include <vector>
#include <mutex>

class Stitching_Strategy {
protected:
	cv::Mat image;
	Parrot_Swarm* swarm;

	void set_image(const cv::Mat& img);
	void save(const std::vector<cv::Mat>& imgs) const;
public:
	Stitching_Strategy(Parrot_Swarm* swarm);
	void get_image(cv::Mat& img);
	virtual void stitch() = 0;
};

class Multi_stitch : public Stitching_Strategy {
private:
	int n;
	time_t now;

	void ser_stitch(std::mutex* mtx);
public:
	Multi_stitch(Parrot_Swarm* swarm, int n);
	virtual void stitch();
};

class Stitching : public Stitching_Strategy {
private:
	void ser_stitch(std::mutex* mtx);
	void stitching(Parrot_Swarm* swarm, cv::Mat& dst);
public:
	Stitching(Parrot_Swarm* swarm);
	virtual void stitch();
};

void center_image(const cv::Mat& src, cv::Mat& dst, const cv::Mat& def);

#endif