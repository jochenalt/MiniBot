search_mode=OPTIMIZE_MAX_JOINT
srdf_filename=minibot.srdf
robot_name_in_srdf=minibot
moveit_config_pkg=minibot_moveit_config
robot_name=minibot
planning_group_name=minibot_arm
ikfast_plugin_pkg=minibot_ikfast_plugin
base_link_name=base_link
eef_link_name=tool0_link
ikfast_output_path=/home/jochenalt/catkin_ws/src/minibot_ikfast_plugin/src/minibot_minibot_arm_ikfast_solver.cpp

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
