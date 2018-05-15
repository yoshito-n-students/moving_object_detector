# moving_object_detector

## Dependencies
object_detection_msgs
* https://github.com/yoshito-n-students/object_detection_msgs

## Subscribed Topics
image_raw (sensor_msgs/Image)

## Published Topics
image_out (sensor_msgs/Image)
* image in which moving objects are found
* never advertised and published unless ~republish_image is ture

objects_out (object_detection_msgs/Objects)
* data and location of detected objects

## Parameters
~detection_algorithm (string, default: "CNT")
* "CNT", "GMG", "GSOC", "KNN", "LSBP", "MOG", or "MOG2"
* see opencv's documentation for algorithm details

~republish_image (bool, default: false)
* republish image if moving objects are found in it

~enumerate_objects (bool, default: true)
* publish object names by 1-based indices ("1", "2", ...)

~image_transport (string, default: "raw")
* transport type of the subscribed image topic

## Examples
see [launch/test.launch](launch/test.launch)
