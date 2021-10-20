/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, WHILL, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
#include <ros/ros.h>
#include <nodelet/loader.h>
#include <image_proc/advertisement_checker.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "stereo_image_proc");

  // Check for common user errors
  if(ros::names::remap("camera") != "camera")
  {
    ROS_WARN("Remapping 'camera' has no effect! Start stereo_image_proc in the "
             "stereo namespace instead.\nExample command-line usage:\n"
             "\t$ ROS_NAMESPACE=%s rosrun stereo_image_proc stereo_image_proc",
             ros::names::remap("camera").c_str());
  }
  if(ros::this_node::getNamespace() == "/")
  {
    ROS_WARN("Started in the global namespace! This is probably wrong. Start "
             "stereo_image_proc in the stereo namespace.\nExample command-line usage:\n"
             "\t$ ROS_NAMESPACE=my_stereo rosrun stereo_image_proc stereo_image_proc");
  }

  // Shared parameters to be propagated to nodelet private namespaces
  ros::NodeHandle     private_nh("~");
  XmlRpc::XmlRpcValue shared_params;
  int                 queue_size;
  if(private_nh.getParam("queue_size", queue_size))
    shared_params["queue_size"] = queue_size;

  nodelet::Loader   manager(false);  // Don't bring up the manager ROS API
  nodelet::M_string remappings;
  nodelet::V_string my_argv;

  // Stereo nodelets also need to know the synchronization policy
  bool approx_sync;
  if(private_nh.getParam("approximate_sync", approx_sync))
    shared_params["approximate_sync"] = XmlRpc::XmlRpcValue(approx_sync);

  // Disparity nodelet
  // Inputs: left/image_rect, left/camera_info, right/image_rect, right/camera_info
  // Outputs: disparity
  // NOTE: Using node name for the disparity nodelet because it is the only one using
  // dynamic_reconfigure so far, and this makes us backwards-compatible with cturtle.
  std::string disparity_name = ros::this_node::getName();
  manager.load(disparity_name, "gpu_stereo_image_proc/vx_disparity", remappings, my_argv);

  // PointCloud2 nodelet
  // Inputs: left/image_rect_color, left/camera_info, right/camera_info, disparity
  // Outputs: points2
  std::string point_cloud2_name = ros::this_node::getName() + "_point_cloud2";
  if(shared_params.valid())
    ros::param::set(point_cloud2_name, shared_params);
  manager.load(point_cloud2_name, "stereo_image_proc/point_cloud2", remappings, my_argv);

  // Check for only the original camera topics
  ros::V_string topics;
  topics.push_back(ros::names::resolve("left/image_rect"));
  topics.push_back(ros::names::resolve("left/camera_info"));
  topics.push_back(ros::names::resolve("right/image_rect"));
  topics.push_back(ros::names::resolve("right/camera_info"));
  image_proc::AdvertisementChecker check_inputs(ros::NodeHandle(), ros::this_node::getName());
  check_inputs.start(topics, 60.0);

  ros::spin();
  return 0;
}
