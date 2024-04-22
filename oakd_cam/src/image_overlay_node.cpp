#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "depthai_ros_msgs/msg/spatial_detection_array.hpp"

class ImageOverlayNode : public rclcpp::Node {
public:
    ImageOverlayNode() : Node("image_overlay_node") {
        // Image subscriber
        image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/color/image", 10, std::bind(&ImageOverlayNode::imageCallback, this, std::placeholders::_1));
        
        // Detection subscriber
        detection_subscriber_ = this->create_subscription<depthai_ros_msgs::msg::SpatialDetectionArray>(
            "/color/yolov6_Spatial_detections", 10, std::bind(&ImageOverlayNode::detectionCallback, this, std::placeholders::_1));

        // Image publisher
        image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/overlay", 10);
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        auto cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat image = cv_ptr->image;
        if (!last_detections_.detections.empty()) {
            drawBoxes(image, last_detections_);
        }
        auto overlay_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
        image_publisher_->publish(*overlay_msg);
    }

    void detectionCallback(const depthai_ros_msgs::msg::SpatialDetectionArray::SharedPtr msg) {
        last_detections_ = *msg;
    }

    void drawBoxes(cv::Mat &image, const depthai_ros_msgs::msg::SpatialDetectionArray &detections) {
        for (auto &detection : detections.detections) {
            auto &bbox = detection.bbox;
            int x = int(1.5*(bbox.center.position.x - bbox.size_x / 2));
            int y = int(1.5*(bbox.center.position.y - bbox.size_y / 2));
            int w = int(1.5*bbox.size_x);
            int h = int(1.5*bbox.size_y);
            cv::rectangle(image, cv::Point(x, y), cv::Point(x + w, y + h), cv::Scalar(255, 0, 0), 2);

            // Assuming detection.results[0].class_id is a std::string
            std::string label = "Class: " + detection.results[0].class_id + 
                                ", Score: " + std::to_string(detection.results[0].score);

            cv::putText(image, label, cv::Point(x, y - 15), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 2);

            // 3D position
            std::string position = "x: " + std::to_string(detection.position.x) + 
                                   ", y: " + std::to_string(detection.position.y) + 
                                   ", z: " + std::to_string(detection.position.z);
            cv::putText(image, position, cv::Point(x, y - 35), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 2);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
    rclcpp::Subscription<depthai_ros_msgs::msg::SpatialDetectionArray>::SharedPtr detection_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    depthai_ros_msgs::msg::SpatialDetectionArray last_detections_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageOverlayNode>());
    rclcpp::shutdown();
    return 0;
}
