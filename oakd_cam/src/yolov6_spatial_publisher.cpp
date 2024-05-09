
#include <cstdio>
#include <functional>
#include <iostream>
#include <tuple>

#include "depthai_bridge/BridgePublisher.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai_bridge/ImgDetectionConverter.hpp"
#include "depthai_bridge/SpatialDetectionConverter.hpp"
#include "depthai_bridge/ImuConverter.hpp"
#include "depthai_ros_msgs/msg/spatial_detection_array.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/node.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"

// Inludes common necessary includes for development using depthai library
#include "depthai/device/DataQueue.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/ColorCamera.hpp"
#include "depthai/pipeline/node/DetectionNetwork.hpp"
#include "depthai/pipeline/node/MonoCamera.hpp"
#include "depthai/pipeline/node/SpatialDetectionNetwork.hpp"
#include "depthai/pipeline/node/StereoDepth.hpp"
#include "depthai/pipeline/node/XLinkIn.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"
#include "depthai/pipeline/node/IMU.hpp"

const std::vector<std::string> label_map = {"duplo"};

dai::Pipeline createPipeline(bool syncNN, bool subpixel, std::string nnPath, int confidence, int LRchecktresh, std::string resolution) {
    dai::Pipeline pipeline;
    dai::node::MonoCamera::Properties::SensorResolution monoResolution;
    auto colorCam = pipeline.create<dai::node::ColorCamera>();
    auto spatialDetectionNetwork = pipeline.create<dai::node::YoloSpatialDetectionNetwork>();
    auto monoLeft = pipeline.create<dai::node::MonoCamera>();
    auto monoRight = pipeline.create<dai::node::MonoCamera>();
    auto stereo = pipeline.create<dai::node::StereoDepth>();

    auto imu = pipeline.create<dai::node::IMU>();
    auto xoutImu = pipeline.create<dai::node::XLinkOut>();

    // create xlink connections
    auto xoutRgb = pipeline.create<dai::node::XLinkOut>();
    auto xoutDepth = pipeline.create<dai::node::XLinkOut>();
    auto xoutNN = pipeline.create<dai::node::XLinkOut>();

    xoutImu->setStreamName("imu");
    xoutRgb->setStreamName("preview");
    xoutNN->setStreamName("detections");
    xoutDepth->setStreamName("depth");

    colorCam->setPreviewSize(640, 640);
    colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    colorCam->setInterleaved(false);
    colorCam->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);

    // Imu
    imu->enableIMUSensor(dai::IMUSensor::ACCELEROMETER_RAW, 500);
    imu->enableIMUSensor(dai::IMUSensor::GYROSCOPE_RAW, 400);
    imu->setBatchReportThreshold(5);
    imu->setMaxBatchReports(20);  // Get one message only for now.

    if(resolution == "720p") {
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_720_P;
    } else if(resolution == "400p") {
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_400_P;
    } else if(resolution == "800p") {
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_800_P;
    } else if(resolution == "480p") {
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_480_P;
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid parameter. -> monoResolution: %s", resolution.c_str());
        throw std::runtime_error("Invalid mono camera resolution.");
    }

    monoLeft->setResolution(monoResolution);
    monoLeft->setBoardSocket(dai::CameraBoardSocket::CAM_B);
    monoRight->setResolution(monoResolution);
    monoRight->setBoardSocket(dai::CameraBoardSocket::CAM_C);

    /// setting node configs
    stereo->initialConfig.setConfidenceThreshold(confidence);
    stereo->setRectifyEdgeFillColor(0);  // black, to better see the cutout
    stereo->initialConfig.setLeftRightCheckThreshold(LRchecktresh);
    stereo->setSubpixel(subpixel);
    stereo->setDepthAlign(dai::CameraBoardSocket::CAM_A);
    stereo->initialConfig.setMedianFilter(dai::MedianFilter::KERNEL_3x3);

    spatialDetectionNetwork->setBlobPath(nnPath);
    spatialDetectionNetwork->setConfidenceThreshold(0.5f);
    spatialDetectionNetwork->input.setBlocking(false);
    spatialDetectionNetwork->setBoundingBoxScaleFactor(0.5);
    spatialDetectionNetwork->setDepthLowerThreshold(100);
    spatialDetectionNetwork->setDepthUpperThreshold(5000);

    // yolo specific parameters
    spatialDetectionNetwork->setNumClasses(1);
    spatialDetectionNetwork->setCoordinateSize(4);
    spatialDetectionNetwork->setAnchors({10, 14, 23, 27, 37, 58, 81, 82, 135, 169, 344, 319});
    spatialDetectionNetwork->setAnchorMasks({{"side13", {3, 4, 5}}, {"side26", {1, 2, 3}}});
    spatialDetectionNetwork->setIouThreshold(0.5f);

    // Link plugins CAM -> STEREO -> XLINK
    monoLeft->out.link(stereo->left);
    monoRight->out.link(stereo->right);
    imu->out.link(xoutImu->input);

    // Link plugins CAM -> NN -> XLINK
    colorCam->preview.link(spatialDetectionNetwork->input);
    if(syncNN)
        spatialDetectionNetwork->passthrough.link(xoutRgb->input);
    else
        colorCam->preview.link(xoutRgb->input);

    spatialDetectionNetwork->out.link(xoutNN->input);

    stereo->depth.link(spatialDetectionNetwork->inputDepth);
    spatialDetectionNetwork->passthroughDepth.link(xoutDepth->input);

    return pipeline;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("yolov6_spatial_node");

    std::string tfPrefix, mode, mxId, resourceBaseFolder, nnPath;
    int imuModeParam;
    bool usb2Mode, poeMode;
    double angularVelCovariance, linearAccelCovariance;
    bool enableRosBaseTimeUpdate;
    std::string nnName(BLOB_NAME);
    std::string camera_param_uri;
    bool syncNN, subpixel;
    int confidence = 200, LRchecktresh = 5;
    std::string monoResolution = "400p";

    node->declare_parameter("tf_prefix", "oak");
    node->declare_parameter("camera_param_uri", camera_param_uri);
    node->declare_parameter("sync_nn", true);
    node->declare_parameter("subpixel", true);
    node->declare_parameter("nnName", "");
    node->declare_parameter("confidence", confidence);
    node->declare_parameter("LRchecktresh", LRchecktresh);
    node->declare_parameter("monoResolution", monoResolution);
    node->declare_parameter("resourceBaseFolder", "");
    node->declare_parameter("mxId", "");
    node->declare_parameter("usb2Mode", false);
    node->declare_parameter("poeMode", false);
    node->declare_parameter("imuMode", 1);
    node->declare_parameter("angularVelCovariance", 0.02);
    node->declare_parameter("linearAccelCovariance", 0.0);
    node->declare_parameter("enableRosBaseTimeUpdate", false);

    node->get_parameter("tf_prefix", tfPrefix);
    node->get_parameter("camera_param_uri", camera_param_uri);
    node->get_parameter("sync_nn", syncNN);
    node->get_parameter("subpixel", subpixel);
    node->get_parameter("confidence", confidence);
    node->get_parameter("LRchecktresh", LRchecktresh);
    node->get_parameter("monoResolution", monoResolution);
    node->get_parameter("resourceBaseFolder", resourceBaseFolder);
    node->get_parameter("mxId", mxId);
    node->get_parameter("usb2Mode", usb2Mode);
    node->get_parameter("poeMode", poeMode);
    node->get_parameter("imuMode", imuModeParam);
    node->get_parameter("angularVelCovariance", angularVelCovariance);
    node->get_parameter("linearAccelCovariance", linearAccelCovariance);
    node->get_parameter("enableRosBaseTimeUpdate", enableRosBaseTimeUpdate);

    if(resourceBaseFolder.empty()) {
        throw std::runtime_error("Send the path to the resouce folder containing NNBlob in \'resourceBaseFolder\' ");
    }

    std::string nnParam;
    node->get_parameter("nnName", nnParam);
    if(nnParam != "x") {
        node->get_parameter("nnName", nnName);
    }

    nnPath = resourceBaseFolder + "/" + nnName;
    dai::ros::ImuSyncMethod imuMode = static_cast<dai::ros::ImuSyncMethod>(imuModeParam);
    dai::Pipeline pipeline = createPipeline(syncNN, subpixel, nnPath, confidence, LRchecktresh, monoResolution);
    dai::Device device(pipeline);

    auto colorQueue = device.getOutputQueue("preview", 30, false);
    auto detectionQueue = device.getOutputQueue("detections", 30, false);
    auto depthQueue = device.getOutputQueue("depth", 30, false);
    auto calibrationHandler = device.readCalibration();
    auto imuQueue = device.getOutputQueue("imu", 30, false);

    int width, height;
    if(monoResolution == "720p") {
        width = 1280;
        height = 720;
    } else if(monoResolution == "400p") {
        width = 640;
        height = 400;
    } else if(monoResolution == "800p") {
        width = 1280;
        height = 800;
    } else if(monoResolution == "480p") {
        width = 640;
        height = 480;
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid parameter. -> monoResolution: %s", monoResolution.c_str());
        throw std::runtime_error("Invalid mono camera resolution.");
    }

    auto boardName = calibrationHandler.getEepromData().boardName;
    if(height > 480 && boardName == "OAK-D-LITE") {
        width = 640;
        height = 480;
    }

    dai::rosBridge::ImageConverter rgbConverter(tfPrefix + "_rgb_camera_optical_frame", false);
    auto rgbCameraInfo = rgbConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_A, -1, -1);
    dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> rgbPublish(colorQueue,
                                                                                       node,
                                                                                       std::string("color/image"),
                                                                                       std::bind(&dai::rosBridge::ImageConverter::toRosMsg,
                                                                                                 &rgbConverter,  // since the converter has the same frame name
                                                                                                                 // and image type is also same we can reuse it
                                                                                                 std::placeholders::_1,
                                                                                                 std::placeholders::_2),
                                                                                       30,
                                                                                       rgbCameraInfo,
                                                                                       "color");

    dai::rosBridge::SpatialDetectionConverter detConverter(tfPrefix + "_rgb_camera_optical_frame", 416, 416, false);
    dai::rosBridge::BridgePublisher<depthai_ros_msgs::msg::SpatialDetectionArray, dai::SpatialImgDetections> detectionPublish(
        detectionQueue,
        node,
        std::string("color/yolov6_Spatial_detections"),
        std::bind(&dai::rosBridge::SpatialDetectionConverter::toRosMsg, &detConverter, std::placeholders::_1, std::placeholders::_2),
        30);

    dai::rosBridge::ImageConverter depthConverter(tfPrefix + "_right_camera_optical_frame", true);
    auto rightCameraInfo = depthConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_C, width, height);
    dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> depthPublish(
        depthQueue,
        node,
        std::string("stereo/depth"),
        std::bind(&dai::rosBridge::ImageConverter::toRosMsg, &depthConverter, std::placeholders::_1, std::placeholders::_2),
        30,
        rightCameraInfo,
        "stereo");

    dai::rosBridge::ImuConverter imuConverter(tfPrefix + "_imu_frame", imuMode, linearAccelCovariance, angularVelCovariance);
    if(enableRosBaseTimeUpdate) {
        imuConverter.setUpdateRosBaseTimeOnToRosMsg();
    }
    dai::rosBridge::BridgePublisher<sensor_msgs::msg::Imu, dai::IMUData> imuPublish(
        imuQueue,
        node,
        std::string("imu/data_raw"),
        std::bind(&dai::rosBridge::ImuConverter::toRosMsg, &imuConverter, std::placeholders::_1, std::placeholders::_2),
        30,
        "",
        "imu");

    imuPublish.addPublisherCallback();
    depthPublish.addPublisherCallback();

    detectionPublish.addPublisherCallback();
    rgbPublish.addPublisherCallback();  // addPublisherCallback works only when the dataqueue is non blocking.

    rclcpp::spin(node);

    return 0;
}
