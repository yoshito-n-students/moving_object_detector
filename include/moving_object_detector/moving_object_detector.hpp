#ifndef MOVING_OBJECT_DETECTOR_MOVING_OBJECT_DETECTOR_HPP
#define MOVING_OBJECT_DETECTOR_MOVING_OBJECT_DETECTOR_HPP

#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber.h>
#include <image_transport/transport_hints.h>
#include <nodelet/nodelet.h>
#include <object_detection_msgs/Objects.h>
#include <object_detection_msgs/cv_conversions.hpp>
#include <ros/console.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>

#include <opencv2/bgsegm.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/background_segm.hpp>

#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>

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
    const std::string default_algorithm("CNT");
    const std::string algorithm(pnh.param("detection_algorithm", default_algorithm));
    enumerate_objects_ = pnh.param("enumerate_objects", true);
    republish_image_ = pnh.param("republish_image", false);

    // setup a detector
    // (TODO: make a way to tune the algorithm)
    detector_ = createDetector(algorithm);
    if (!detector_) {
      ROS_WARN_STREAM("Unsupported detection algorithm type: " << algorithm << ". Will use "
                                                               << default_algorithm << ".");
      detector_ = createDetector(default_algorithm);
    }

    // setup result publishers
    image_transport::ImageTransport it(nh);
    if (republish_image_) {
      image_publisher_ = it.advertise("image_out", 1, true);
    }
    object_publisher_ = nh.advertise< object_detection_msgs::Objects >("objects_out", 1, true);

    // start detection
    const image_transport::TransportHints default_hints;
    image_subscriber_ =
        it.subscribe("image_raw", 1, &MovingObjectDetector::detect, this,
                     image_transport::TransportHints(default_hints.getTransport(),
                                                     default_hints.getRosHints(), pnh));
  }

  void detect(const sensor_msgs::ImageConstPtr &image_msg) {
    // ROS image message -> opencv's 24-bit image the detector can accept
    cv_bridge::CvImageConstPtr image(cv_bridge::toCvShare(image_msg, "bgr8"));
    if (!image) {
      ROS_ERROR("Image conversion error");
      return;
    }
    if (image->image.empty()) {
      ROS_ERROR("Empty image message");
      return;
    }

    // detect foreground mask
    cv::Mat foreground_mask;
    detector_->apply(image->image, foreground_mask);

    // cluster foreground mask by finding contours
    std::vector< std::vector< cv::Point > > contours;
    cv::findContours(foreground_mask, contours, cv::noArray() /* optional contours hierarchy */,
                     cv::RETR_EXTERNAL /* detect external contours only */,
                     cv::CHAIN_APPROX_SIMPLE /* return only vertices of coutours */);
    if (contours.empty()) {
      return;
    }

    // publish the image where the objects are found if required
    if (republish_image_) {
      image_publisher_.publish(image_msg);
    }

    // publish found objects data
    const object_detection_msgs::ObjectsPtr object_msg(new object_detection_msgs::Objects);
    object_msg->header = image_msg->header;
    if (enumerate_objects_) {
      object_msg->names = toNamesMsg(contours.size());
    }
    object_msg->contours = object_detection_msgs::toContoursMsg(contours);
    object_publisher_.publish(object_msg);
  }

  // utility function to create foreground detection algorithm by name
  static cv::Ptr< cv::BackgroundSubtractor > createDetector(const std::string &algorithm) {
    if (algorithm == "CNT") {
      return cv::bgsegm::createBackgroundSubtractorCNT();
    } else if (algorithm == "GMG") {
      return cv::bgsegm::createBackgroundSubtractorGMG();
    } else if (algorithm == "GSOC") {
      return cv::bgsegm::createBackgroundSubtractorGSOC();
    } else if (algorithm == "KNN") {
      return cv::createBackgroundSubtractorKNN();
    } else if (algorithm == "LSBP") {
      return cv::bgsegm::createBackgroundSubtractorLSBP();
    } else if (algorithm == "MOG") {
      return cv::bgsegm::createBackgroundSubtractorMOG();
    } else if (algorithm == "MOG2") {
      return cv::createBackgroundSubtractorMOG2();
    }
    return cv::Ptr< cv::BackgroundSubtractor >();
  }

  // utility function to give simple names to objects
  static std::vector< std::string > toNamesMsg(const std::size_t n) {
    std::vector< std::string > names_msg;
    for (std::size_t i = 1; i <= n; ++i) {
      names_msg.push_back(boost::lexical_cast< std::string >(i));
    }
    return names_msg;
  }

private:
  bool enumerate_objects_,republish_image_;

  cv::Ptr< cv::BackgroundSubtractor > detector_;

  image_transport::Subscriber image_subscriber_;
  image_transport::Publisher image_publisher_;
  ros::Publisher object_publisher_;
}; // namespace moving_object_detector

} // namespace moving_object_detector

#endif