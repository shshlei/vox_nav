// Copyright (c) 2020 Fetullah Atas, Norwegian University of Life Sciences
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef VOX_NAV_UTILITIES__TF_HELPERS_HPP_
#define VOX_NAV_UTILITIES__TF_HELPERS_HPP_

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <memory>
#include <string>

namespace vox_nav_utilities
{
struct RigidBodyTransformation
{
  Eigen::Vector3d translation_{0.0, 0.0, 0.0};

  // intrinsic rotation (opposite from the ROS convention), order X-Y-Z
  Eigen::Vector3d rpyIntrinsic_{0.0, 0.0, 0.0};
};

enum class XYZ : int
{
  X,
  Y,
  Z
};

// Transform a PoseStamped from one frame to another while catching exceptions
bool transformPose(const std::shared_ptr<tf2_ros::Buffer> & tf, const std::string & frame,
  const geometry_msgs::msg::PoseStamped & in_pose, geometry_msgs::msg::PoseStamped & out_pose,
  rclcpp::Duration & transform_tolerance);

// Get the Current Pose object
bool getCurrentPose(geometry_msgs::msg::PoseStamped & global_pose, tf2_ros::Buffer & tf_buffer,
  const std::string & global_frame = "map", const std::string & robot_frame = "base_link",
  const double transform_timeout = 0.1);

// Get the Euclidian Dist Between Poses object
double getEuclidianDistBetweenPoses(const geometry_msgs::msg::Pose & a, const geometry_msgs::msg::Pose & b);

// Get the Euclidian Dist Between Poses object
double getEuclidianDistBetweenPoses(const geometry_msgs::msg::PoseStamped & a, const geometry_msgs::msg::PoseStamped & b);

// Get the Euclidian Dist Between Poses object
double getEuclidianDistBetweenPoints(const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b);

// Provide tf2::Quaternion and get roll pitch yaw
void getRPYfromTFQuaternion(const tf2::Quaternion & q, double & roll, double & pitch, double & yaw);

// Get the Quaternionfrom R P Y object
tf2::Quaternion getTFQuaternionfromRPY(const double roll, const double pitch, const double yaw);

// Provide geometry_msgs::msg::Quaternion and get roll pitch yaw
void getRPYfromMsgQuaternion(const geometry_msgs::msg::Quaternion & q_msg, double & roll, double & pitch, double & yaw);

// Get the Msg Quaternionfrom R P Y object
geometry_msgs::msg::Quaternion getMsgQuaternionfromRPY(const double roll, const double pitch, const double yaw);

// Get the Rotation Matrix object
Eigen::Matrix3d getRotationMatrix(double angle, XYZ axis);

// Get the Rigid Body Transform object
Eigen::Affine3d getRigidBodyTransform(const Eigen::Vector3d & translation, const Eigen::Vector3d & intrinsicRpy);

/*
const double EPSILON = std::numeric_limits<double>::epsilon();

std::tuple<int, int, int> convert_to_rgb(double minval, double maxval, double val,
  const std::vector<std::tuple<int, int, int>> & colors);

double euclidean_distance(const std::tuple<int, int, int> & c1, const std::tuple<int, int, int> & c2);

double convert_to_value(const std::tuple<int, int, int> & rgb, double minval, double maxval,
  const std::vector<std::tuple<int, int, int>> & colors);
*/

}  // namespace vox_nav_utilities

#endif  // VOX_NAV_UTILITIES__TF_HELPERS_HPP_
