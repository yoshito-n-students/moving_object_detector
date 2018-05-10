# moving_object_detector

## Subscribed Topics
image_raw (sensor_msgs/Image)

## Published Topics
image_out (sensor_msgs/Image)
* image in which moving objects are found
* never advertised and published unless ~republish_image is ture

objects_out (object_detection_msgs/Objects)
* data and location of detected objects

## Parameters
~republish_image (bool, default: false)
* republish image if moving objects are found in it

~image_transport (string, default: "raw")
* transport type of the subscribed image topic

## Examples
see [launch/test.launch](launch/test.launch)
