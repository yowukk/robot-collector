cmake_minimum_required(VERSION 3.0.2)
project(detection_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  message_generation
)

add_message_files(
  FILES
  YoloRect.msg
  AnnotationWithPose2D.msg
  CompressedImageWithAnnotationAndPose2D.msg
)

generate_messages(
  DEPENDENCIES geometry_msgs sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs sensor_msgs message_generation
)
