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

#include <ros/ros.h>

#include <webots/Supervisor.hpp>

#include "include/RobotNode.h"

using namespace webots;


int main(int argc, char **argv) {
  ros::init(argc, argv, "dual_arm_api");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  /*
   * This API takes 3 arguments in string type.
   * The first arg indicates the running mode,
   * 'leader' (default) is for leader mode, where the simulator will publish the /clock and
   * measured_joint_states messages.
   * 'follower' is for follower mode, where the simulator will not publish the robot's joint
   * states but update them based on the js_cmd_topic.
   * If the first arg is empty or not be 'leader' or 'follower', the api will exit.
   *
   * The second arg is for the js_cmd_topic. It is `cartesian/solution` by default.
   * If it is given '', default value will be used. If not given, the api will exit.
   *
   * The third arg is for the js_measured_topic. It is `measured_joint_states` by default.
   * If it is given '', default value will be used. If not given, the api will exit.
   */
  if (argc != 4) {
    ROS_ERROR("The controller takes exact 3 string arguments: mode, js_cmd_topic, js_measured_topic, "
              "but only %d are given", argc-1);
    exit(-1);
  }
  // Check the args
  std::string mode = argv[1];
  if (mode == "leader" || mode == "follower") {
    ROS_INFO("Robot simulation run in %s mode", mode.c_str());
  } else {
    ROS_ERROR("Robot simulation mode should be either 'leader' or 'follower', but got %s", mode.c_str());
    exit(0);
  }
  std::string js_cmd_topic = "cartesian/solution";
  if (!std::string(argv[2]).empty()) {
    js_cmd_topic = std::string(argv[2]);
  }
  ROS_INFO("External joint states command topic: %s", js_cmd_topic.c_str());

  std::string js_measured_topic = "measured_joint_states";
  if (!std::string(argv[3]).empty()) {
    js_measured_topic = std::string(argv[3]);
  }
  ROS_INFO("Publish measured joint states topic: %s", js_measured_topic.c_str());

  auto *robot = new Supervisor();
  RobotNode robot_node(nh, robot, mode, js_cmd_topic, js_measured_topic);

  while (robot_node.update()) {
    ros::spinOnce();
  }

  return 0;
}


