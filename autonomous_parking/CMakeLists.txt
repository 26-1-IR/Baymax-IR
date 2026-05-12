cmake_minimum_required(VERSION 3.8)
project(autonomous_parking)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# 설치 대상 폴더들
install(DIRECTORY
  launch
  worlds
  models
  media
  config
  maps
  urdf
  DESTINATION share/${PROJECT_NAME}
)

# Python 스크립트 (ros2 run으로 실행 가능하게 lib/ 에 설치)
install(PROGRAMS
  scripts/parking_node.py
  scripts/vision_node.py
  scripts/decision_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
