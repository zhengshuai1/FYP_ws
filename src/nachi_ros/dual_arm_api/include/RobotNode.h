// Copyright (c) 2020 SMART Lab CUHK 
// Copyright (c) 2020 Zhipeng Dong
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

#ifndef dual_arm_API_ROBOTNODE_H
#define dual_arm_API_ROBOTNODE_H

#include <random>
#include <iostream>
#include <fstream>
#include <stdexcept>
#include <vector>
#include <algorithm>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>
#include <rosgraph_msgs/Clock.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <webots/Node.hpp>
#include <webots/Motor.hpp>
#include <webots/Camera.hpp>
#include <webots/Keyboard.hpp>
#include <webots/Connector.hpp>
#include <webots/Supervisor.hpp>
#include <webots/RangeFinder.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/TouchSensor.hpp>

// webots_ros
#include <webots_ros/set_int.h>

#include <dual_arm_api/MoveAndCapture.h>
#include <dual_arm_api/MoveJointGroup.h>

// Optional roport
#ifdef USE_ROPORT
#include <roport/ExecuteBinaryAction.h>
#include <roport/GetImageData.h>
#endif

using namespace webots;

enum CAMERA_ID{LEFT_BGR, RIGHT_BGR, LEFT_DEPTH, RIGHT_DEPTH};


class RobotNode
{
public:
  RobotNode(const ros::NodeHandle& nh, Supervisor *robot, const std::string& mode,
            std::string js_cmd_topic, std::string js_measured_topic);
  ~RobotNode();

  bool update();

private:
  ros::NodeHandle nh_;

  Supervisor *robot_;
  int time_step_;
  double sec_;

  int long_wait_time_ = 10000;
  int short_wait_time_ = 100;

  bool left_gripper_open_;
  bool right_gripper_open_;

  Keyboard *keyboard_;

  std::string mode_;
  std::string js_cmd_topic_;
  std::string js_measured_topic_;

  /**
   * Names of the joints (motors) in dual_arm's Webots proto
   * If the proto is converted from URDF, the motor name is the joint name
   */
  std::vector<std::string> joint_names_;

  std::vector<Camera> rgb_cameras_;
  std::vector<RangeFinder> depth_cameras_;
  std::vector<image_transport::Publisher> rgb_publishers_;
  std::vector<image_transport::Publisher> depth_publishers_;

//  std::array<Connector, 2> grippers_;
  std::default_random_engine generator_;

  ros::Publisher pub_clock_;
  ros::Publisher pub_js_;

  ros::Subscriber sub_js_cmd_;

  ros::ServiceServer srv_gripper_;
  ros::ServiceServer srv_move_and_capture_;
  ros::ServiceServer srv_move_joint_group_;

  /**
   * Execute the position in the given sensor_msgs::JointState message
   * @param msg sensor_msgs::JointState message
   */
  void joint_position_cb(const sensor_msgs::JointState::ConstPtr& msg);

  /**
   * Given a webots_ros::set_int request, control the gripper based on the int.
   * @param req For value, 1: close left gripper; 2: open left gripper;
   *            3: close right gripper; 4: open right gripper.
   * @param resp For success, true: gripper controlled; false: control error.
   * @return If call succeed
   */
  bool run_gripper_srv(webots_ros::set_int::Request &req, webots_ros::set_int::Response &resp);

#ifdef USE_ROPORT
  ros::ServiceServer srv_binary_gripper_;
  bool binary_gripper_srv(roport::ExecuteBinaryAction::Request &req, roport::ExecuteBinaryAction::Response &resp);
  ros::ServiceServer srv_get_image_data_;
  bool get_image_srv(roport::GetImageData::Request &req, roport::GetImageData::Response &resp);
#endif

  /** move_and_capture
   * This service moves the robot moving group to given goal and captures image(s) at that pose
   * @param req
   * @param resp
   * @return
   */
  bool move_and_capture_srv(dual_arm_api::MoveAndCapture::Request &req, dual_arm_api::MoveAndCapture::Response &resp);
  bool move_joint_group_srv(dual_arm_api::MoveJointGroup::Request &req, dual_arm_api::MoveJointGroup::Response &resp);

  bool captureImages(std::vector<std::string> camera_names, std::vector<sensor_msgs::Image> &images);

  std::vector<double> getRandomCmd(const std::vector<std::string>& j_names);

  bool openGripper(const std::string& name);
  bool closeGripper(const std::string& name, double angle=0.872);

  void publishClock();

  static bool allClose(std::vector<double> a, std::vector<double> b, double tol=0.005);
  static bool forceClosure(std::vector<double> forces, double force_limit=5);

  bool waitExecutionFinish(const std::vector<std::string>& j_names, const std::vector<double>& j_q_cmd, int timeout_cnt);
  bool waitExecutionFinishWithForceFB(const std::vector<std::string>& j_names, const std::vector<double>& j_q_cmd,
                                      std::vector<std::string> sensor_names, int timeout_cnt);

  void publishJointStates();

  void publishRGB();
  void publishDepth();

  void setJointPositions(std::vector<std::string> j_names, std::vector<double> &j_q_cmd);
  bool executeJointPositionCmd(std::vector<std::string> j_names, std::vector<double> &j_q_cmd, int timeout_cnt);
  bool executeJointCmdWithForceFB(std::vector<std::string> j_names, std::vector<double> &j_q_cmd,
                                  std::vector<std::string> sensor_names, int timeout_cnt);

  std::vector<double> getJointPosition(const std::vector<std::string>& j_names);
  std::vector<double> getForces(const std::vector<std::string>& sensor_names);
  void getJointStates(std::vector<double> &q, std::vector<double> &dq, std::vector<double> &tau_J);

  bool getBGR(Camera *camera, cv::Mat &bgr);
  bool getBGR8Msg(Camera *camera, sensor_msgs::ImagePtr &bgr8_msg);

  bool getDepth(RangeFinder *range_finder, cv::Mat &depth);
  bool getDepthMsg(RangeFinder *range_finder, sensor_msgs::ImagePtr &depth_msg);

  void onKeyboard();

  void baseSetWheelVelocity(std::vector<double> vel);
  void baseForward(double vel=1.8);
  void baseBackward(double vel=1.8);
  void baseSlideLeft(double vel=1.8);
  void baseSlideRight(double vel=1.8);
  void baseTurnLeft(double vel=1.8);
  void baseTurnRight(double vel=1.8);
  void baseStop();
};


#endif //dual_arm_API_ROBOTNODE_H
