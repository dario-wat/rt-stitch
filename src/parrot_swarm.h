#ifndef IMG_HANDLER_H
#define IMG_HANDLER_H

#include <vector>
#include <sensor_msgs/Image.h>
#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include "parrot.h"
#include "callbacks.h"

class Parrot_Swarm {
private:
	std::vector<Parrot*> parrots;
public:
	Parrot_Swarm();
	Parrot_Swarm(const std::vector<Parrot*>& parrots);
	~Parrot_Swarm();
	void add_parrot(Parrot* p);
	int size() const;
	void subscribe_all(ros::NodeHandle& n, const cbk::Callback_fun& callback);
	void clear_subscriptions();
	void extract_images(std::vector<cv::Mat>& dst);
	static void show_separate(const Parrot_Swarm& swarm);
	static void show_concat(const Parrot_Swarm& swarm);
	static void show_stitch(const Parrot_Swarm& swarm);
};

#endif