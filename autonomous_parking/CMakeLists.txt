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

# Python 스크립트가 있으면
install(DIRECTORY
  scripts
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

ament_package()
