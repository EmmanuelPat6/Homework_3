#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
 
class ImageProcessorNode : public rclcpp::Node {
public:
  ImageProcessorNode() : Node("opencv_image_processor") {
    
    //Subscriber for the simulated image
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/videocamera", 10,
        std::bind(&ImageProcessorNode::image_callback, this, std::placeholders::_1));
 
    //Publisher for the processed image
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/processed_image", 10);
  }
 
private:

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) 
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      //Converts the ROS message to an OpenCV object with BGR8 encoding
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    //Setup SimpleBlobDetector parameters.
    cv::SimpleBlobDetector::Params params;

    // Change Thresholds
    params.minThreshold = 0;
    params.maxThreshold = 255;
    
    //Filter by Color
    params.filterByColor=false;
    params.blobColor=0;

    // Filter by Area
    params.filterByArea = false;
    params.minArea = 0.5;
    
    // Filter by Circularity
    params.filterByCircularity = true;
    params.minCircularity = 0.8;
    
    // Filter by Convexity
    params.filterByConvexity = true;
    params.minConvexity = 0.9;
    
    // Filter by Inertia
    params.filterByInertia = true;
    params.minInertiaRatio = 0.9;
  
    //Set up detector with params
    cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
    
    std::vector<cv::KeyPoint> keypoints;

    //Use the detector to find blobs in the image
    detector->detect(cv_ptr->image,keypoints);

    //Draw the found blobs as red circles on a copy of the original image
    cv::Mat im_with_keypoints;
    cv::drawKeypoints(cv_ptr->image, keypoints, im_with_keypoints, cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    //Show the processed image in a window called 'Sphere Detection'
    cv::imshow("Sphere Detection", im_with_keypoints);
    cv::waitKey(0);

    //Conversion and Publishing
    auto processed_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", im_with_keypoints).toImageMsg();
    publisher_->publish(*processed_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;

};

 
int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImageProcessorNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}