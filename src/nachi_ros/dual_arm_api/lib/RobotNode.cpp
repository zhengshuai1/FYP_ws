// Copyright (c) 2020 ULTRA Lab CUHK
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

#include "RobotNode.h"

#include <utility>

RobotNode::RobotNode(const ros::NodeHandle& nh, Supervisor *robot, const std::string& mode,
                     std::string js_cmd_topic, std::string js_measured_topic) :
  nh_(nh), robot_(robot), sec_(0), mode_(mode),
  js_cmd_topic_(std::move(js_cmd_topic)), js_measured_topic_(js_measured_topic)
{
  if (mode == "leader") {
    pub_clock_ = nh_.advertise<rosgraph_msgs::Clock>("/clock", 1);
    pub_js_ = nh_.advertise<sensor_msgs::JointState>(js_measured_topic_, 1);
    sub_js_cmd_ = nh_.subscribe(js_cmd_topic_, 1, &RobotNode::joint_position_cb, this);
  } else if (mode == "follower") {
    sub_js_cmd_ = nh_.subscribe(js_cmd_topic_, 1, &RobotNode::joint_position_cb, this);
  } else {
    throw std::runtime_error("Unknown mode: <" + mode + ">");
  }

  srv_gripper_ = nh_.advertiseService("/simulation/gripper/run", &RobotNode::run_gripper_srv, this);
  srv_move_and_capture_ = nh_.advertiseService("/simulation/move_and_capture", &RobotNode::move_and_capture_srv, this);
//  srv_move_joint_group_ = nh_.advertiseService("/simulation/move_joint_group", &RobotNode::move_joint_group_srv, this);

#ifdef USE_ROPORT
  srv_binary_gripper_ = nh_.advertiseService("execute_gripper_action", &RobotNode::binary_gripper_srv, this);
  srv_get_image_data_ = nh_.advertiseService("get_image_data", &RobotNode::get_image_srv, this);
#endif

  time_step_ = (int)robot_->getBasicTimeStep();

  /* The following initialization aims to be agnostic to the robot definition */
  // Initialize devices (motors, position sensors, cameras)
  for (int i = 0; i < robot_->getNumberOfDevices(); ++i) {
    Device *tag = robot_->getDeviceByIndex(i);
    int type = tag->getNodeType();
    std::string name = tag->getName();

    if (type == Node::ROTATIONAL_MOTOR || type == Node::LINEAR_MOTOR) {
      Motor *motor = robot_->getMotor(name);
      if (motor == nullptr) continue;
      PositionSensor *ps = motor->getPositionSensor();
      if (ps == nullptr) continue;

      motor->enableTorqueFeedback(time_step_);
      ps->enable(time_step_);
      joint_names_.push_back(name);
      ROS_INFO("Initialized motor #%d %s\n", i, name.c_str());
    } else if (type == Node::CAMERA) {
      Camera *cam = robot_->getCamera(name);
      if (cam != nullptr) {
        cam->enable(time_step_);
        image_transport::ImageTransport it(nh_);
        image_transport::Publisher publisher = it.advertise("/simulation/" + name + "/rgb", 1);
        rgb_cameras_.push_back(*cam);
        rgb_publishers_.push_back(publisher);
        ROS_INFO("Initialized rgb camera #%d %s", i, name.c_str());
      }
    } else if (type == Node::RANGE_FINDER) {
      RangeFinder *cam = robot_->getRangeFinder(name);
      if (cam != nullptr) {
        cam->enable(time_step_);
        image_transport::ImageTransport it(nh_);
        image_transport::Publisher publisher = it.advertise("/simulation/" + name + "/depth", 1);
        depth_cameras_.push_back(*cam);
        depth_publishers_.push_back(publisher);
        ROS_INFO("Initialized depth camera #%d %s", i, name.c_str());
      }
    } else if (type == Node::TOUCH_SENSOR) {
      TouchSensor *touchSensor = robot_->getTouchSensor(name);
      if (touchSensor != nullptr) {
        touchSensor->enable(time_step_);
        ROS_INFO("Initialized touch sensor #%d %s", i, name.c_str());
      }
    }
  }

  // Initialize gripper states
  left_gripper_open_ = true;
  right_gripper_open_ = true;
}

RobotNode::~RobotNode() {
  delete robot_;
}

void RobotNode::joint_position_cb(const sensor_msgs::JointState::ConstPtr &msg) {
  assert(msg->name.size() == msg->position.size());
  std::vector<std::string> j_names;
  std::vector<double> j_q;
  for (size_t i = 0; i < msg->name.size(); i++) {
    // Ignore command to the gripper drivers, they can only be controlled via services
    if (msg->name[i].find("hand") != std::string::npos) continue;
    j_names.push_back(msg->name[i]);
    j_q.push_back(msg->position[i]);
  }
  if (!j_names.empty() && j_q.size() == j_names.size()) {
    executeJointPositionCmd(j_names, j_q, long_wait_time_);
  }
}

bool RobotNode::run_gripper_srv(webots_ros::set_int::Request &req, webots_ros::set_int::Response &resp) {
  if (req.value == 1) {
    resp.success = closeGripper("L");
  } else if (req.value == 2) {
    resp.success = openGripper("L");
  } else if (req.value == 3) {
    resp.success = closeGripper("R");
  } else if (req.value == 4) {
    resp.success = openGripper("R");
  } else {
    ROS_ERROR("Gripper control value should be: \n"
              "1: close left gripper; 2: open left gripper; 3: close right gripper; 4: open right gripper\n"
              "But got %d", req.value);
    return false;
  }
  return true;
}

#ifdef USE_ROPORT
bool RobotNode::binary_gripper_srv(roport::ExecuteBinaryAction::Request &req,
                                   roport::ExecuteBinaryAction::Response &resp) {
  std::string gripper;
  if (req.device_id == 0) {
    gripper = "L";
  } else {
    gripper = "R";
  }
  if (req.enable) {
    closeGripper(gripper, req.value);
  } else {
    openGripper(gripper);
  }
  resp.result_status = resp.SUCCEEDED;
  return true;
}

bool RobotNode::get_image_srv(roport::GetImageData::Request &req, roport::GetImageData::Response &resp) {
  if (captureImages(req.device_names, resp.images)) {
    resp.result_status = resp.SUCCEEDED;
  } else {
    resp.result_status = resp.FAILED;
  }
}
#endif

bool RobotNode::move_and_capture_srv(dual_arm_api::MoveAndCapture::Request &req,
                                     dual_arm_api::MoveAndCapture::Response &resp) {
  std::vector<std::string> j_names = req.joint_names;
  std::vector<double> j_q_cmd;
  if (req.random) {
    j_q_cmd = getRandomCmd(j_names);
  } else {
    j_q_cmd = req.joint_positions;
  }
  if (!j_names.empty() && !j_q_cmd.empty() && j_names.size() == j_q_cmd.size()) {
    executeJointPositionCmd(j_names, j_q_cmd, long_wait_time_);
  }

  if (captureImages(req.camera_names, resp.images)) {
    resp.result_status = resp.SUCCEEDED;
  } else {
    resp.result_status = resp.FAILED;
  }
  return true;
}

//bool RobotNode::move_joint_group_srv(dual_arm_api::MoveJointGroup::Request &req,
//                                     dual_arm_api::MoveJointGroup::Response &resp) {
//  std::vector<std::string> j_names;
//  std::vector<double> js_cmd = req.position;
//  if (req.group_id == req.LEFT_ARM) {
//    j_names = {
//      "arm_L_joint1", "arm_L_joint2", "arm_L_joint3", "arm_L_joint4", "arm_L_joint5", "arm_L_joint6", "arm_L_joint7"
//    };
//  } else if (req.group_id == req.RIGHT_ARM) {
//    j_names = {
//      "arm_R_joint1", "arm_R_joint2", "arm_R_joint3", "arm_R_joint4", "arm_R_joint5", "arm_R_joint6", "arm_R_joint7"
//    };
//  } else if (req.group_id == req.TORSO) {
//    j_names = {"waist_joint_0A_0B", "waist_joint_1_L12A", "waist_joint_1_crank_A"};
//  } else {
//    resp.result_status = resp.FAILED;
//    return true;
//  }
//
//  if (executeJointPositionCmd(j_names, js_cmd, long_wait_time_)) {
//    resp.result_status = resp.SUCCEEDED;
//  } else {
//    resp.result_status = resp.FAILED;
//  }
//  return true;
//}

bool RobotNode::captureImages(std::vector<std::string> camera_names, std::vector<sensor_msgs::Image> &images) {
  for (size_t i = 0; i < camera_names.size(); ++i) {
    Camera *rgb_cam = robot_->getCamera(camera_names[i]);
    RangeFinder *depth_cam = robot_->getRangeFinder(camera_names[i]);

    sensor_msgs::ImagePtr img_msg;
    if (rgb_cam != nullptr && depth_cam == nullptr) {
      if (getBGR8Msg(rgb_cam, img_msg)) {
        images.push_back(*img_msg);
      } else {
        ROS_ERROR("Get BGR image failed");
        return false;
      }
    } else if (rgb_cam == nullptr && depth_cam != nullptr) {
      if (getDepthMsg(depth_cam, img_msg)) {
        images.push_back(*img_msg);
      } else {
        ROS_ERROR("Get depth image failed");
        return false;
      }
    } else {
      ROS_ERROR("The camera %s was not found", camera_names[i].c_str());
      return false;
    }
  }
  return true;
}

std::vector<double> RobotNode::getRandomCmd(const std::vector<std::string>& j_names) {
  std::vector<double> j_q_cmd;
  std::uniform_real_distribution<double> distribution(0., 1.0);
  for (auto &name : j_names) {
    double seed = distribution(generator_);
    Motor *motor = robot_->getMotor(name);
    auto min_p = motor->getMinPosition();
    auto max_p = motor->getMaxPosition();
    auto cmd = min_p + (max_p - min_p) * seed;
    j_q_cmd.push_back(cmd);
  }
  return j_q_cmd;
}

bool RobotNode::allClose(std::vector<double> a, std::vector<double> b, double tol)
{
  for (size_t i = 0; i < a.size(); ++i) {
    if (fabs(a[i] - b[i]) > tol) {
      return false;
    }
  }
  return true;
}

bool RobotNode::forceClosure(std::vector<double> forces, double force_limit) {
  if (!forces.empty()) {
    bool reached_limit = true;
    for (auto & f : forces) {
      if (f < force_limit) {
        reached_limit = false;
        break;
      }
    }
    if (reached_limit) {
      ROS_INFO("Force limit %f reached", force_limit);
      return true;
    } else {
      return false;
    }
  }
  return false;
}

bool RobotNode::waitExecutionFinish(const std::vector<std::string>& j_names,
                                    const std::vector<double>& j_q_cmd, int timeout_cnt) {
  bool need_timeout = true;
  if (timeout_cnt <= 0) {
    need_timeout = false;
  }
  bool target_reached = false;
  while (robot_->step(time_step_) != -1) {
    auto j_q_curr = getJointPosition(j_names);
    if (allClose(j_q_cmd, j_q_curr)) {
      target_reached = true;
      break;
    }
    if (need_timeout) {
      if (timeout_cnt <= 0) break;
      else timeout_cnt--;
    }
  }
  return target_reached;
}

bool RobotNode::waitExecutionFinishWithForceFB(const std::vector<std::string>& j_names, const std::vector<double>& j_q_cmd,
                                               std::vector<std::string> sensor_names, int timeout_cnt)
{
  bool need_timeout = true;
  if (timeout_cnt <= 0) {
    need_timeout = false;
  }
  bool target_reached = false;
  while (robot_->step(time_step_) != -1) {
    // Check force criterion
    auto force_curr = getForces(sensor_names);
    auto j_q_curr = getJointPosition(j_names);
    if (forceClosure(force_curr)) {
      // Stop the movement at current joint position
      return executeJointPositionCmd(j_names, j_q_curr, 100);
    }
    // Check position criterion
    if (allClose(j_q_cmd, j_q_curr)) {
      target_reached = true;
      break;
    }
    // Check timeout
    if (need_timeout) {
      if (timeout_cnt <= 0) break;
      else timeout_cnt--;
    }
  }
  return target_reached;
}

void RobotNode::setJointPositions(std::vector<std::string> j_names, std::vector<double> &j_q_cmd) {
  assert(j_names.size() == j_q_cmd.size());
  for (size_t i = 0; i < j_q_cmd.size(); ++i) {
    if (j_names[i] == "waist_joint_2_R23A") {
      j_names[i] = "waist_joint_1_crank_A";
    }
    if (std::find(std::begin(joint_names_), std::end(joint_names_), j_names[i]) == std::end(joint_names_))
      continue;
    Motor *motor = robot_->getMotor(j_names[i]);
    if (j_q_cmd[i] > motor->getMaxPosition()) {
      j_q_cmd[i] = motor->getMaxPosition();
    }
    else if (j_q_cmd[i] < motor->getMinPosition()) {
      j_q_cmd[i] = motor->getMinPosition();
    }
    motor->setPosition(j_q_cmd[i]);
  }
}

bool RobotNode::executeJointPositionCmd(std::vector<std::string> j_names, std::vector<double> &j_q_cmd, int timeout_cnt)
{
  setJointPositions(j_names, j_q_cmd);
  return waitExecutionFinish(j_names, j_q_cmd, timeout_cnt);
}

bool RobotNode::executeJointCmdWithForceFB(std::vector<std::string> j_names, std::vector<double> &j_q_cmd,
                                           std::vector<std::string> sensor_names, int timeout_cnt)
{
  setJointPositions(j_names, j_q_cmd);
  return waitExecutionFinishWithForceFB(j_names, j_q_cmd, sensor_names, timeout_cnt);
}

std::vector<double> RobotNode::getJointPosition(const std::vector<std::string>& j_names) {
  std::vector<double> j_q;
  for (auto & j_name : j_names) {
    double p = 0.;
    if (std::find(std::begin(joint_names_), std::end(joint_names_), j_name) != std::end(joint_names_)) {
      auto motor = robot_->getMotor(j_name);
      p = motor->getPositionSensor()->getValue();
    }
    j_q.push_back(p);
  }
  return j_q;
}

std::vector<double> RobotNode::getForces(const std::vector<std::string>& sensor_names) {
  std::vector<double> forces;
  for (auto & s_name : sensor_names) {
    double f = 0.;
    auto sensor = robot_->getTouchSensor(s_name);
    if (sensor != nullptr) {
      // The unit of the force is 0.1N if default lookUpTable is used
      f = sensor->getValue();
    }
    forces.push_back(f);
  }
  return forces;
}

void RobotNode::getJointStates(std::vector<double> &q, std::vector<double> &dq, std::vector<double> &tau_J) {
  for (auto & j_name : joint_names_) {
    double j_q, j_dq, j_tau;
    auto motor = robot_->getMotor(j_name);
    j_q = motor->getPositionSensor()->getValue();
    // FIXME this velocity is not the instant velocity
    j_dq = motor->getVelocity();
    j_tau = motor->getTorqueFeedback();
    q.push_back(j_q);
    dq.push_back(j_dq);
    tau_J.push_back(j_tau);
  }
}

bool RobotNode::openGripper(const std::string& name) {
  assert(name == "L" || name == "R");
  std::string gripper_name = "hand_" + name;
  auto connector = robot_->getConnector(gripper_name + "_connector");
  if (connector != nullptr) {
    connector->unlock();
    ROS_INFO("UnLocked");
  }

  std::vector<std::string> j_names = {gripper_name + "_right_driver_joint", gripper_name + "_left_driver_joint"};
  std::vector<double> j_q_cmd = {0., 0.};
  executeJointPositionCmd(j_names, j_q_cmd, short_wait_time_);

  if (name == "L") left_gripper_open_ = true;
  else right_gripper_open_ = true;
  return true;
}

bool RobotNode::closeGripper(const std::string& name, double angle) {
  assert(name == "L" || name == "R");
  std::string gripper_name = "hand_" + name;
  if (name == "R"){
  std::vector<std::string> j_names = {gripper_name + "_right_driver_joint", gripper_name + "_left_driver_joint"};
  std::vector<double> j_q_cmd = {angle, angle};
  std::vector<std::string> sensor_names = {gripper_name + "_right_touch_sensor",};
  executeJointCmdWithForceFB(j_names, j_q_cmd, sensor_names, short_wait_time_);
  }
  auto connector = robot_->getConnector(gripper_name + "_connector");
  if (connector != nullptr) {
    connector->lock();
    ROS_INFO("%s Locked",gripper_name.c_str());
  }

  if (name == "L") left_gripper_open_ = false;
  else right_gripper_open_ = false;
  return true;
}

void RobotNode::publishClock() {
  rosgraph_msgs::Clock msg;
  msg.clock.fromSec(sec_);
  pub_clock_.publish(msg);
  sec_ += double(time_step_) / 1000.0;
}

void RobotNode::publishJointStates() {
  std::vector<double> q;  // joint positions
  std::vector<double> dq;  // joint velocities
  std::vector<double> tau_J;  // joint torque
  getJointStates(q, dq, tau_J);

  sensor_msgs::JointState msg;
  // Using now() here could cause crash!
  msg.header.stamp.fromSec(sec_);
  msg.name.insert(msg.name.end(), joint_names_.begin(), joint_names_.end());
  msg.position.insert(msg.position.end(), q.begin(), q.end());
  msg.velocity.insert(msg.velocity.end(), dq.begin(), dq.end());
  msg.effort.insert(msg.effort.end(), tau_J.begin(), tau_J.end());
  pub_js_.publish(msg);
}

void RobotNode::publishRGB() {
  sensor_msgs::ImagePtr rgb_msg;
  for (size_t i = 0; i < rgb_cameras_.size(); ++i) {
    if (getBGR8Msg(&rgb_cameras_[i], rgb_msg)) {
      rgb_publishers_[i].publish(rgb_msg);
    }
  }
}

void RobotNode::publishDepth() {
  sensor_msgs::ImagePtr depth_msg;
  for (size_t i = 0; i < depth_cameras_.size(); ++i) {
    if (getDepthMsg(&depth_cameras_[i], depth_msg)) {
      depth_publishers_[i].publish(depth_msg);
    }
  }
}

bool RobotNode::getBGR(Camera *camera, cv::Mat &bgr) {
  if (camera != nullptr) {
    cv::Mat bgra(camera->getHeight(), camera->getWidth(), CV_8UC4);
    bgra.data = (uchar *)camera->getImage();
    cv::cvtColor(bgra, bgr, cv::COLOR_BGRA2BGR);
    return true;
  } else {
    return false;
  }
}

bool RobotNode::getBGR8Msg(Camera *camera, sensor_msgs::ImagePtr &bgr8_msg) {
  cv::Mat bgr;
  if (getBGR(camera, bgr)) {
    bgr8_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", bgr).toImageMsg();
    return true;
  } else {
    return false;
  }
}

bool RobotNode::getDepth(RangeFinder *range_finder, cv::Mat &depth) {
  if (range_finder != nullptr) {
    cv::Mat depth_f(range_finder->getHeight(), range_finder->getWidth(), CV_32FC1);
    depth_f.data = (uchar *)range_finder->getRangeImage();
    // 16UC1 is the supported depth image format of ROS.
    // The float values are in meters, while the 16U values are in mm.
    depth_f.convertTo(depth, CV_16UC1, 1000.);
    return true;
  } else {
    return false;
  }
}

bool RobotNode::getDepthMsg(RangeFinder *range_finder, sensor_msgs::ImagePtr &depth_msg) {
  cv::Mat depth;
  if (getDepth(range_finder, depth)) {
    // Do not use mono16 to replace sensor_msgs::image_encodings::TYPE_16UC1
    depth_msg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::TYPE_16UC1, depth).toImageMsg();
    return true;
  } else {
    return false;
  }
}
bool RobotNode::update() {
  if (robot_->step(time_step_) != -1) {
    if (mode_ == "leader") {
      publishJointStates();
      publishClock();
      // Publishing images after step the simulation
//      publishRGB();
      // FIXME publishing depth images will make the simulation slow
      //publishDepth();
    }
    return true;
  } else {
    return false;
  }
}
