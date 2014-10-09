#ifndef CALLBACKS_H
#define CALLBACKS_H

#include <sensor_msgs/Image.h>
class Parrot;	// forward declaration

namespace cbk {

typedef void Callback_fun(const sensor_msgs::Image::ConstPtr& img, Parrot* const parrot);

void set_callback(const sensor_msgs::Image::ConstPtr& img, Parrot* const parrot);

}

#endif