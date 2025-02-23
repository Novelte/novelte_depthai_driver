// Copyright (c) [2022] [Adam Serafin]

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#ifndef DEPTHAI_ROS_DRIVER__MM_POINTCLOUD_HPP_
#define DEPTHAI_ROS_DRIVER__MM_POINTCLOUD_HPP_

#include <cv_bridge/cv_bridge.h>

#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "depthai/depthai.hpp"
#include "depthai/pipeline/datatype/ADatatype.hpp"
#include "depthai/pipeline/node/SpatialDetectionNetwork.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"
#include "depthai_ros_driver/base_camera.hpp"
#include "image_transport/image_transport.hpp"
#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"


namespace depthai_ros_driver
{
class NnPointcloud : public BaseCamera
{
public:
  explicit NnPointcloud(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~NnPointcloud() {}
  void on_configure() override;

private:
  void setup_pipeline();
  void setup_publishers();
  void det_cb(const std::string & name, const std::shared_ptr<dai::ADatatype> & data);
  
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_;
  std::shared_ptr<dai::node::NeuralNetwork> nn_;
  std::shared_ptr<dai::node::XLinkIn> xyz_in_;
  std::shared_ptr<dai::node::XLinkOut> xout_nn_, xout_bbdm_, pointsOut_;
  std::shared_ptr<dai::DataOutputQueue> pointcloud_nn_q_, bbdm_q_;
  std::string nn_path_;
  std::vector<std::string> label_map_;
  dai::CalibrationHandler calibData_;

  
  bool sync_nn_ = true;

  
};
}  // namespace depthai_ros_driver

#endif  //  DEPTHAI_ROS_DRIVER__MOBILENET_CAMERA_OBJ_HPP_
