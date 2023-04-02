cmake_minimum_required(VERSION 3.8)
project(ros2basilisk)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# python + cpp dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

# for interfaces
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AttGuid.msg"
  "msg/CmdTorque.msg"
  "msg/CurrentFrame.msg"
  "msg/PointTimed.msg"
  "msg/ObjectCount.msg"
  "msg/QuaternionTimed.msg"
  "msg/RWTorque.msg"
  "msg/SCStates.msg"
  "msg/ThrustCmd.msg"
  # ADD NEW MESSAGES HERE:
  DEPENDENCIES std_msgs
)

# for cpp nodes
# include_directories(include)
# add_executable(cpp_executable src/cpp_node.cpp)
# ament_target_dependencies(cpp_executable rclcpp)
# install(TARGETS
#   cpp_executable
#   DESTINATION lib/${PROJECT_NAME}
# )

# for python nodes/files
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  ros2basilisk/__init__.py
  ros2basilisk/Simulation.py
  ros2basilisk/IncrementalSimulationClass.py
  ros2basilisk/RosBasilisk.py
  ros2basilisk/RosBskTask.py
  # algorithms
  algorithm_base/AttitudeCtrlAllocNode.py
  algorithm_base/FormationControl.py
  algorithm_base/SimpleNav.py
  algorithm_base/utilitiy.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  # algorithms helper
  algorithm_base/helper_fns/attitude_ctrl.py
  algorithm_base/helper_fns/Constants.py
  algorithm_base/helper_fns/__init__.py
  algorithm_base/helper_fns/lqr.py
  algorithm_base/helper_fns/OrbitalMechanics.py
  algorithm_base/helper_fns/OrbitPropagation.py
  algorithm_base/helper_fns/PropagationInterface.py
  algorithm_base/helper_fns/PropagationUtil.py
  algorithm_base/helper_fns/RelativeOrbitalMechanics.py
  algorithm_base/helper_fns/TimeTable.py
  algorithm_base/helper_fns/utility.py
  algorithm_base/helper_fns/LinAlg.py
  algorithm_base/helper_fns/Hcw.py
  algorithm_base/helper_fns/AttitudeUtil.py
  DESTINATION lib/${PROJECT_NAME}/helper_fns
)

# install the launch directory
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
