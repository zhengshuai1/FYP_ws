search_mode=OPTIMIZE_MAX_JOINT
srdf_filename=mz04_gripper.srdf
robot_name_in_srdf=mz04_gripper
moveit_config_pkg=mz04_gripper_moveit_config
robot_name=mz04_arm
planning_group_name=arm
ikfast_plugin_pkg=mz04_arm_ikfast_arm_plugin
base_link_name=mz04_link0
eef_link_name=mz04_link6
ikfast_output_path=/home/hp/nachi_ws/src/mz04_arm_ikfast_arm_plugin/src/mz04_arm_arm_ikfast_solver.cpp

rosrun moveit_kinematics create_ikfast_moveit_plugin.py\
  --search_mode=$search_mode\
  --srdf_filename=$srdf_filename\
  --robot_name_in_srdf=$robot_name_in_srdf\
  --moveit_config_pkg=$moveit_config_pkg\
  $robot_name\
  $planning_group_name\
  $ikfast_plugin_pkg\
  $base_link_name\
  $eef_link_name\
  $ikfast_output_path
