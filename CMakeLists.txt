cmake_minimum_required(VERSION 3.0.2)
project(cable_analysis)


find_package(catkin REQUIRED COMPONENTS
    cv_bridge
    image_transport
    roscpp
    sensor_msgs
    std_msgs
    )

find_package( OpenCV REQUIRED )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES MaskPicker
  CATKIN_DEPENDS roscpp sensor_msgs image_transport
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(MaskPicker src/mask_picker.cpp)
    add_dependencies(MaskPicker
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    )
    target_link_libraries(MaskPicker
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    )

add_executable(cv_bridge_node src/cv_bridge_node.cpp)
    target_link_libraries(cv_bridge_node
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
    )

add_executable(vertical_line_detection_node src/vertical_line_detection_node.cpp)
    target_link_libraries(vertical_line_detection_node
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
    )

add_executable(canny_edge_detection_node src/canny_edge_detection_node.cpp)
    target_link_libraries(canny_edge_detection_node
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
    )

add_executable(canny_edge_video_detection_node src/canny_edge_video_detection_node.cpp)
    target_link_libraries(canny_edge_video_detection_node
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
    )

add_executable(vertical_lines_video_extractor_node src/vertical_lines_video_extractor_node.cpp)
    target_link_libraries(vertical_lines_video_extractor_node
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
    )

add_executable(countors_node src/countors_node.cpp)
    target_link_libraries(countors_node
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
    )