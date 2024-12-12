#include <cstdio>
#include <functional>
#include <iostream>
#include <tuple>

#include "camera_info_manager/camera_info_manager.hpp"
#include "depthai_ros_msgs/msg/spatial_detection_array.hpp"
#include "rclcpp/node.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "stereo_msgs/msg/disparity_image.hpp"

// Inludes common necessary includes for development using depthai library
#include "depthai/device/DataQueue.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/ColorCamera.hpp"
#include "depthai/pipeline/node/IMU.hpp"
#include "depthai/pipeline/node/MonoCamera.hpp"
#include "depthai/pipeline/node/SpatialDetectionNetwork.hpp"
#include "depthai/pipeline/node/StereoDepth.hpp"
#include "depthai/pipeline/node/XLinkIn.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"
#include "depthai_bridge/BridgePublisher.hpp"
#include "depthai_bridge/DisparityConverter.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai_bridge/ImuConverter.hpp"
#include "depthai_bridge/SpatialDetectionConverter.hpp"
#include "depthai_bridge/depthaiUtility.hpp"

std::vector<std::string> usbStrings = {"UNKNOWN", "LOW", "FULL", "HIGH", "SUPER", "SUPER_PLUS"};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("stereo_inertial_node");

  std::string tfPrefix, mode, mxId, resourceBaseFolder, nnPath;
  std::string monoResolution = "720p", rgbResolution = "1080p";
  int badParams = 0, stereo_fps, confidence, LRchecktresh, imuModeParam, detectionClassesCount,
    expTime, sensIso;
  int rgbScaleNumerator, rgbScaleDinominator, previewWidth, previewHeight;
  bool lrcheck, extended, subpixel, enableDepth, rectify, depth_aligned, manualExposure;
  bool enableSpatialDetection, enableDotProjector, enableFloodLight;
  bool usb2Mode, poeMode, syncNN;
  double angularVelCovariance, linearAccelCovariance;
  double dotProjectorIntensity, floodLightIntensity;
  bool enableRosBaseTimeUpdate;
  std::string nnName(BLOB_NAME);    // Set your blob name for the model here

  node->declare_parameter("mxId", "");
  node->declare_parameter("usb2Mode", false);
  node->declare_parameter("poeMode", false);
  node->declare_parameter("resourceBaseFolder", "");

  node->declare_parameter("tf_prefix", "oak");
  node->declare_parameter("mode", "depth");
  node->declare_parameter("imuMode", 1);

  node->declare_parameter("lrcheck", true);
  node->declare_parameter("extended", false);
  node->declare_parameter("subpixel", true);
  node->declare_parameter("rectify", false);

  node->declare_parameter("depth_aligned", true);
  node->declare_parameter("stereo_fps", 30);
  node->declare_parameter("confidence", 200);
  node->declare_parameter("LRchecktresh", 5);
  node->declare_parameter("monoResolution", "720p");
  node->declare_parameter("rgbResolution", "1080p");
  node->declare_parameter("manualExposure", false);
  node->declare_parameter("expTime", 20000);
  node->declare_parameter("sensIso", 800);

  node->declare_parameter("rgbScaleNumerator", 2);
  node->declare_parameter("rgbScaleDinominator", 3);
  node->declare_parameter("previewWidth", 416);
  node->declare_parameter("previewHeight", 416);

  node->declare_parameter("angularVelCovariance", 0.02);
  node->declare_parameter("linearAccelCovariance", 0.0);
  node->declare_parameter("enableSpatialDetection", true);
  node->declare_parameter("detectionClassesCount", 80);
  node->declare_parameter("syncNN", true);
  node->declare_parameter("nnName", "x");

  node->declare_parameter("enableDotProjector", false);
  node->declare_parameter("enableFloodLight", false);
  node->declare_parameter("dotProjectorIntensity", 0.5);
  node->declare_parameter("floodLightIntensity", 0.5);
  node->declare_parameter("enableRosBaseTimeUpdate", false);

  // updating parameters if defined in launch file.

  node->get_parameter("mxId", mxId);
  node->get_parameter("usb2Mode", usb2Mode);
  node->get_parameter("poeMode", poeMode);
  node->get_parameter("resourceBaseFolder", resourceBaseFolder);

  node->get_parameter("tf_prefix", tfPrefix);
  node->get_parameter("mode", mode);
  node->get_parameter("imuMode", imuModeParam);

  node->get_parameter("lrcheck", lrcheck);
  node->get_parameter("extended", extended);
  node->get_parameter("subpixel", subpixel);
  node->get_parameter("rectify", rectify);

  node->get_parameter("depth_aligned", depth_aligned);
  node->get_parameter("stereo_fps", stereo_fps);
  node->get_parameter("confidence", confidence);
  node->get_parameter("LRchecktresh", LRchecktresh);
  node->get_parameter("monoResolution", monoResolution);
  node->get_parameter("rgbResolution", rgbResolution);
  node->get_parameter("manualExposure", manualExposure);
  node->get_parameter("expTime", expTime);
  node->get_parameter("sensIso", sensIso);

  node->get_parameter("rgbScaleNumerator", rgbScaleNumerator);
  node->get_parameter("rgbScaleDinominator", rgbScaleDinominator);
  node->get_parameter("previewWidth", previewWidth);
  node->get_parameter("previewHeight", previewHeight);

  node->get_parameter("angularVelCovariance", angularVelCovariance);
  node->get_parameter("linearAccelCovariance", linearAccelCovariance);
  node->get_parameter("enableSpatialDetection", enableSpatialDetection);
  node->get_parameter("detectionClassesCount", detectionClassesCount);
  node->get_parameter("syncNN", syncNN);

  node->get_parameter("enableDotProjector", enableDotProjector);
  node->get_parameter("enableFloodLight", enableFloodLight);
  node->get_parameter("dotProjectorIntensity", dotProjectorIntensity);
  node->get_parameter("floodLightIntensity", floodLightIntensity);
  node->get_parameter("enableRosBaseTimeUpdate", enableRosBaseTimeUpdate);

  if (resourceBaseFolder.empty()) {
    throw std::runtime_error(
            "Send the path to the resouce folder containing NNBlob in \'resourceBaseFolder\' ");
  }

  std::string nnParam;
  node->get_parameter("nnName", nnParam);
  if (nnParam != "x") {
    node->get_parameter("nnName", nnName);
  }
  nnPath = resourceBaseFolder + "/" + nnName;

  if (mode == "depth") {
    enableDepth = true;
  } else {
    enableDepth = false;
  }

  dai::ros::ImuSyncMethod imuMode = static_cast<dai::ros::ImuSyncMethod>(imuModeParam);

  dai::Pipeline pipeline_dummy;
  int width = 640;
  int height = 480;
  bool isDeviceFound = false;

  // recreate empty pipeline
  dai::Pipeline pipeline;

  std::shared_ptr<dai::Device> device;
  std::vector<dai::DeviceInfo> availableDevices = dai::Device::getAllAvailableDevices();

  std::cout << "Listing available devices..." << std::endl;
  for (auto deviceInfo : availableDevices) {
    std::cout << "Device Mx ID: " << deviceInfo.getMxId() << std::endl;
    if (deviceInfo.getMxId() == mxId) {
      if (deviceInfo.state == X_LINK_UNBOOTED || deviceInfo.state == X_LINK_BOOTLOADER) {
        isDeviceFound = true;
        if (poeMode) {
          device = std::make_shared<dai::Device>(pipeline, deviceInfo);
        } else {
          device = std::make_shared<dai::Device>(pipeline, deviceInfo, usb2Mode);
        }
        break;
      } else if (deviceInfo.state == X_LINK_BOOTED) {
        throw std::runtime_error(
                "\" DepthAI Device with MxId  \"" + mxId +
                "\" is already booted on different process.  \"");
      }
    } else if (mxId == "x") {
      isDeviceFound = true;
      device = std::make_shared<dai::Device>(pipeline);
    }
  }
  if (!isDeviceFound) {
    throw std::runtime_error("\" DepthAI Device with MxId  \"" + mxId + "\" not found.  \"");
  }

  if (!poeMode) {
    std::cout << "Device USB status: " << usbStrings[static_cast<int32_t>(device->getUsbSpeed())] <<
      std::endl;
  }

  std::cout << "Camera ready!" << std::endl;


  return 0;
}
