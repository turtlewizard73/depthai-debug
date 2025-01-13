/* Copyright 2023 Enjoy Robotics Zrt - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Modifications to this file is to be shared with the code owner.
 * Proprietary and confidential
 * Owner: Enjoy Robotics Zrt maintainer@enjoyrobotics.com, 2023
 */

#ifndef DEPTHAI_PLUGINS__CAMERA_SELECTION_TEST_HPP_
#define DEPTHAI_PLUGINS__CAMERA_SELECTION_TEST_HPP_


#include <depthai_ros_driver/pipeline/base_pipeline.hpp>
#include <depthai_ros_driver/dai_nodes/base_node.hpp>

namespace depthai_plugins
{

class CustomPipeline : public depthai_ros_driver::pipeline_gen::BasePipeline
{

public:
  std::vector<std::unique_ptr<depthai_ros_driver::dai_nodes::BaseNode>>
  createPipeline(
    std::shared_ptr<rclcpp::Node> node,
    std::shared_ptr<dai::Device> device,
    std::shared_ptr<dai::Pipeline> pipeline,
    const std::string & nnType);

private:
  std::vector<std::string> camera_sockets_;
  std::vector<std::string> camera_names_;
};

}

#endif  // DEPTHAI_PLUGINS__CAMERA_SELECTION_TEST_HPP_
