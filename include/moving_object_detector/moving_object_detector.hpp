#ifndef MOVING_OBJECT_DETECTOR_MOVING_OBJECT_DETECTOR_HPP
#define MOVING_OBJECT_DETECTOR_MOVING_OBJECT_DETECTOR_HPP

#include <image_transport/image_transport.h>
#include <image_transport/subscriber.h>
#include <nodelet/nodelet.h>
#include <object_detection_msgs/Objects.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>

namespace moving_object_detector {

class MovingObjectDetector : public nodelet::Nodelet {
public:
  MovingObjectDetector() {}

  virtual ~MovingObjectDetector() {}

private:
  virtual void onInit() {
    ros::NodeHandle &nh(getNodeHandle());
    ros::NodeHandle &pnh(getPrivateNodeHandle());

    // load params
    republish_image_ = pnh.param("republish_image", false);

    // start detection
    image_transport::ImageTransport it(nh);
    if (republish_image_) {
      image_publisher_ = it.advertise("image_out", 1, true);
    }
    object_publisher_ = nh.advertise< object_detection_msgs::Objects >("objects_out", 1, true);
    image_subscriber_ = it.subscribe("image_raw", 1, &MovingObjectDetector::detect, this);
  }

  void detect(const sensor_msgs::ImageConstPtr &msg) {}

private:
  bool republish_image_;

  image_transport::Subscriber image_subscriber_;
  image_transport::Publisher image_publisher_;
  ros::Publisher object_publisher_;
};

} // namespace moving_object_detector

#endif