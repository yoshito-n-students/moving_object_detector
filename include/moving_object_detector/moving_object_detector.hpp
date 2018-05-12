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
#include <object_detection_msgs/Point.h>
#include <object_detection_msgs/Points.h>
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
    const std::string type(pnh.param< std::string >("type", "CNT"));
    republish_image_ = pnh.param("republish_image", false);

    // setup a detector
    // (TODO: make a way to specify parameters)
    if (type == "CNT") {
      detector_ = cv::bgsegm::createBackgroundSubtractorCNT();
    } else if (type == "GMG") {
      detector_ = cv::bgsegm::createBackgroundSubtractorGMG();
    } else if (type == "GSOC") {
      detector_ = cv::bgsegm::createBackgroundSubtractorGSOC();
    } else if (type == "KNN") {
      detector_ = cv::createBackgroundSubtractorKNN();
    } else if (type == "LSBP") {
      detector_ = cv::bgsegm::createBackgroundSubtractorLSBP();
    } else if (type == "MOG") {
      detector_ = cv::bgsegm::createBackgroundSubtractorMOG();
    } else if (type == "MOG2") {
      detector_ = cv::createBackgroundSubtractorMOG2();
    } else {
      ROS_WARN_STREAM("Unsupported background subtractor type: " << type
                                                                 << ". Will use defalut type.");
      detector_ = cv::bgsegm::createBackgroundSubtractorCNT();
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
    // ROS image message -> opencv's image
    cv_bridge::CvImageConstPtr image(cv_bridge::toCvShare(image_msg /* , "bgr8" */));
    if (!image) {
      ROS_ERROR("Image conversion error");
      return;
    }
    if (image->image.empty()) {
      ROS_ERROR("Empty image message");
      return;
    }

    // detect forground mask
    cv::Mat forground_mask;
    detector_->apply(image->image, forground_mask);

    // cluster forground mask by finding contours
    std::vector< std::vector< cv::Point > > contours;
    cv::findContours(forground_mask, contours, cv::noArray() /* optional contours hierarchy */,
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
    object_msg->names = toNamesMsg(contours.size());
    object_msg->contours = toContoursMsg(contours);
    object_publisher_.publish(object_msg);
  }

  // utility function to give simple names to objects
  static std::vector< std::string > toNamesMsg(const std::size_t n) {
    std::vector< std::string > names_msg;
    for (std::size_t i = 1; i <= n; ++i) {
      names_msg.push_back(boost::lexical_cast< std::string >(i));
    }
    return names_msg;
  }

  // utility function to convert opencv's contours to a ROS message
  static std::vector< object_detection_msgs::Points >
  toContoursMsg(const std::vector< std::vector< cv::Point > > &contours) {
    namespace odm = object_detection_msgs;

    std::vector< odm::Points > contours_msg;
    BOOST_FOREACH (const std::vector< cv::Point > &points, contours) {
      odm::Points points_msg;
      BOOST_FOREACH (const cv::Point &point, points) {
        odm::Point point_msg;
        point_msg.x = point.x;
        point_msg.y = point.y;
        points_msg.points.push_back(point_msg);
      }
      contours_msg.push_back(points_msg);
    }
    return contours_msg;
  }

private:
  bool republish_image_;

  cv::Ptr< cv::BackgroundSubtractor > detector_;

  image_transport::Subscriber image_subscriber_;
  image_transport::Publisher image_publisher_;
  ros::Publisher object_publisher_;
}; // namespace moving_object_detector

} // namespace moving_object_detector

#endif