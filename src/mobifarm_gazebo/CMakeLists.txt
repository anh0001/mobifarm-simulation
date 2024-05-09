cmake_minimum_required(VERSION 2.8.3)
project(mobifarm_gazebo)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(launch/test_world.launch)
endif()
