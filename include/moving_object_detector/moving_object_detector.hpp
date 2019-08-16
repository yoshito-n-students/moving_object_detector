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
  cv::Ptr< cv::BackgroundSubtractor > createDetector(const std::string &algorithm) {
    ros::NodeHandle pnh(getPrivateNodeHandle(), algorithm);
    if (algorithm == "CNT") {
      return cv::bgsegm::createBackgroundSubtractorCNT(
          pnh.param< int >("min_pixel_stability", 15), pnh.param< bool >("use_history", true),
          pnh.param< int >("max_pixel_stability", 15 * 60), pnh.param< bool >("is_parallel", true));
    } else if (algorithm == "GMG") {
      return cv::bgsegm::createBackgroundSubtractorGMG(
          pnh.param< int >("initialization_frames", 120),
          pnh.param< double >("decision_threshold", 0.8));
    } else if (algorithm == "GSOC") {
      return cv::bgsegm::createBackgroundSubtractorGSOC(
          pnh.param< int >("mc", cv::bgsegm::LSBP_CAMERA_MOTION_COMPENSATION_NONE),
          pnh.param< int >("n_samples", 20), pnh.param< float >("replace_rate", 0.003),
          pnh.param< float >("propagate_rate", 0.01), pnh.param< int >("hits_threshold", 32),
          pnh.param< float >("alpha", 0.01), pnh.param< float >("beta", 0.0022),
          pnh.param< float >("blinking_supression_decay", 0.1),
          pnh.param< float >("blinking_supression_multiplier", 0.1),
          pnh.param< float >("noise_removal_threshold_fac_bg", 0.0004),
          pnh.param< float >("noise_removal_threshold_fac_fg", 0.0008));
    } else if (algorithm == "KNN") {
      return cv::createBackgroundSubtractorKNN(pnh.param< int >("history", 500),
                                               pnh.param< double >("dist2threshold", 400.),
                                               pnh.param< bool >("detect_shadows", true));
    } else if (algorithm == "LSBP") {
      return cv::bgsegm::createBackgroundSubtractorLSBP(
          pnh.param< int >("mc", cv::bgsegm::LSBP_CAMERA_MOTION_COMPENSATION_NONE),
          pnh.param< int >("n_samples", 20), pnh.param< int >("lsbp_radius", 16),
          pnh.param< float >("t_lower", 2.), pnh.param< float >("t_upper", 32.),
          pnh.param< float >("t_inc", 1.), pnh.param< float >("t_dec", 0.05),
          pnh.param< float >("r_scale", 10.), pnh.param< float >("r_incdec", 0.005),
          pnh.param< float >("noise_removal_threshold_fac_bg", 0.0004),
          pnh.param< float >("noise_removal_threshold_fac_fg", 0.0008),
          pnh.param< int >("lsbp_threshold", 8), pnh.param< int >("min_count", 2));
    } else if (algorithm == "MOG") {
      return cv::bgsegm::createBackgroundSubtractorMOG(
          pnh.param< int >("history", 200), pnh.param< int >("nmixtures", 5),
          pnh.param< double >("background_ratio", 0.7), pnh.param< double >("noise_sigma", 0.));
    } else if (algorithm == "MOG2") {
      return cv::createBackgroundSubtractorMOG2(pnh.param< int >("history", 500),
                                                pnh.param< double >("var_threshold", 16.),
                                                pnh.param< bool >("detect_shadows", true));
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
  bool enumerate_objects_, republish_image_;

  cv::Ptr< cv::BackgroundSubtractor > detector_;

  image_transport::Subscriber image_subscriber_;
  image_transport::Publisher image_publisher_;
  ros::Publisher object_publisher_;
}; // namespace moving_object_detector

} // namespace moving_object_detector

#endif