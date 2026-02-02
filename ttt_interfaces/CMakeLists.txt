cmake_minimum_required(VERSION 3.8)
project(ttt_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 2. Generate the interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BoardState.msg"
  "srv/GetBoardState.srv"
  "action/ExecuteMove.action"
  "action/TextToSpeech.action"
  "srv/GetRobotMove.srv"
  "srv/GetGameStatus.srv"
  "srv/GetHumanMove.srv"
  "srv/ManualUpdate.srv"
  
)

# 3. Export dependencies for other packages
ament_export_dependencies(rosidl_default_runtime)

ament_package()
