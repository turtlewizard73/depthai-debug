/* Copyright 2023 Enjoy Robotics Zrt - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Modifications to this file is to be shared with the code owner.
 * Proprietary and confidential
 * Owner: Enjoy Robotics Zrt maintainer@enjoyrobotics.com, 2023
 */

#include <rclcpp/rclcpp.hpp>

#include "depthai_plugins/depthai_custom_pipeline_plugin.hpp"

#include <depthai_ros_driver/pipeline/base_pipeline.hpp>
#include <depthai_ros_driver/dai_nodes/base_node.hpp>
#include <depthai_ros_driver/dai_nodes/sensors/rgb.hpp>
#include <depthai_ros_driver/dai_nodes/sensors/mono.hpp>
#include <depthai/pipeline/Pipeline.hpp>


std::vector<std::unique_ptr<depthai_ros_driver::dai_nodes::BaseNode>>
depthai_plugins::CustomPipeline::createPipeline(
  std::shared_ptr<rclcpp::Node>/*node*/,
  std::shared_ptr<dai::Device>/*device*/,
  std::shared_ptr<dai::Pipeline>/*pipeline*/,
  const std::string & /*nnType*/)
{
  std::vector<std::unique_ptr<depthai_ros_driver::dai_nodes::BaseNode>> daiNodes;

  return daiNodes;
}


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  depthai_plugins::CustomPipeline,
  depthai_ros_driver::pipeline_gen::BasePipeline)
