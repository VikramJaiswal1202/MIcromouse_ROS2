cmake_minimum_required(VERSION 3.5)
project(micromouse_gazebo)

find_package(ament_cmake REQUIRED)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install worlds
install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

