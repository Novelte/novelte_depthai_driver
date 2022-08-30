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

#include "depthai_ros_driver/nn_pointcloud.hpp"

#include <memory>
#include <string>
#include <vector>

#include "depthai-shared/common/CameraBoardSocket.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "depthai/pipeline/datatype/ADatatype.hpp"
#include "depthai_ros_driver/params_rgb.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace depthai_ros_driver
{
NnPointcloud::NnPointcloud(const rclcpp::NodeOptions & options)
: BaseCamera("camera", options)
{
  on_configure();
}

void NnPointcloud::on_configure()
{
  declare_rgb_depth_params();
  std::string default_nn_path = ament_index_cpp::get_package_share_directory("depthai_ros_driver") +
    "/models/out/pointcloud_640x400.blob";
  nn_path_ = this->declare_parameter<std::string>("nn_path", default_nn_path);
  setup_publishers();
  setup_pipeline();
  // this->create_wall_timer(500ms, std::bind(&NnPointcloud::timer_callback, this));

  RCLCPP_INFO(this->get_logger(), "NnPointcloud ready!");
}

// void NnPointcloud::timer_callback()
// {

// }
void NnPointcloud::setup_pipeline()
{
  rgb_params::RGBInitConfig config;
  config.preview_size = 300;
  override_init_rgb_config(config);
  setup_basic_devices();
  setup_all_xout_streams();
  setup_control_config_xin();
  
  auto pipeline = get_pipeline();
  nn_ = pipeline->create<dai::node::NeuralNetwork>();
  nn_->setBlobPath(nn_path_);
  nn_->setNumInferenceThreads(2);
  nn_->input.setBlocking(false);


  stereo_->depth.link(nn_->inputs["depth"]);

  xyz_in_ = pipeline->create<dai::node::XLinkIn>();
  xyz_in_->setMaxDataSize(6144000);
  xyz_in_->setStreamName("xyz_in");
  xyz_in_->out.link(nn_->inputs["xyz"]);


  // Only send xyz data once, and always reuse the message
  nn_->inputs["xyz"].setReusePreviousMessage(true);


  pointsOut_ = pipeline->create<dai::node::XLinkOut>();
  pointsOut_->setStreamName("pcl");
  nn_->out.link(pointsOut_->input);
  






  start_device();
  setup_all_queues();

  setup_nnpointcloud();
  
  pointcloud_nn_q_ = get_output_q("pcl", 8, false);
  pointcloud_nn_q_->get<dai::NNData>();
  pointcloud_nn_q_->addCallback(
    std::bind(&NnPointcloud::det_cb, this, std::placeholders::_1, std::placeholders::_2));
  
}
void NnPointcloud::setup_publishers()
{
  pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("~/points", 10);
}




void NnPointcloud::det_cb(const std::string & name, const std::shared_ptr<dai::ADatatype> & data)
{
  // RCLCPP_INFO(this->get_logger(), "name: %s", name.c_str());
  
  
  auto in_pc = std::dynamic_pointer_cast<dai::NNData>(data);
  
  auto cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
  cloud_msg->header.frame_id = get_frame_id(dai::CameraBoardSocket::RGB);
  cloud_msg->header.stamp = this->get_clock()->now();
  cloud_msg->height = 400;
  cloud_msg->width = 640;
  cloud_msg->is_dense = false;
  cloud_msg->is_bigendian = false;
  auto data_pc = in_pc->getFirstLayerFp16();
  // auto data_raw = in_pc->getRaw()->data;
  // RCLCPP_INFO(this->get_logger(), "size: %d", data_raw.size());

  // RCLCPP_INFO(this->get_logger(), "pc: %f %f %f %f %f %f %f %f %f %f", data_pc[0], data_pc[1], data_pc[2], data_pc[3], data_pc[4], data_pc[5], data_pc[6], data_pc[7], data_pc[8], data_pc[9]);
  // RCLCPP_INFO(this->get_logger(), "pc: %f %f %f %f %f %f %f %f %f %f", 
  // data_pc[data_pc.size()-0], 
  // data_pc[data_pc.size()-1], 
  // data_pc[data_pc.size()-2], 
  // data_pc[data_pc.size()-3], 
  // data_pc[data_pc.size()-4], 
  // data_pc[data_pc.size()-5], 
  // data_pc[data_pc.size()-6], 
  // data_pc[data_pc.size()-7], 
  // data_pc[data_pc.size()-8], 
  // data_pc[data_pc.size()-9]);
  
  // RCLCPP_INFO(this->get_logger(), "data_pc size: %d", data_pc.size());
  sensor_msgs::PointCloud2Modifier pcd_modifier(*cloud_msg);
  pcd_modifier.setPointCloud2FieldsByString(1, "xyz");

  sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_msg, "z");
  float bad_point = std::numeric_limits<float>::quiet_NaN();
  for(int i = 0; i < cloud_msg->height; i++)
  {
    for(int j = 0; j < cloud_msg->width; j++, ++iter_x, ++iter_y, ++iter_z)
    {
      auto x = data_pc[j + i * cloud_msg->width ];
      auto y = data_pc[j + i * cloud_msg->width + cloud_msg->width * cloud_msg->height];
      auto z = data_pc[j + i * cloud_msg->width + cloud_msg->width * cloud_msg->height * 2];
      // RCLCPP_INFO(this->get_logger(), "pc: %f", data_pc[cloud_msg->width * cloud_msg->height + i * cloud_msg->width + j]);
      // if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) )
      // {
      //   *iter_x = *iter_y = *iter_z = bad_point;
      //   continue;
      // }
      *iter_x = x / 1000.0;
      *iter_y = y / 1000.0;
      *iter_z = z / 1000.0;    
    }
  }
  pc_pub_->publish(*cloud_msg);

  return;

}

}  // namespace depthai_ros_driver
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(depthai_ros_driver::NnPointcloud)
