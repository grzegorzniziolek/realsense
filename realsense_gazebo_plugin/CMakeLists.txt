cmake_minimum_required(VERSION 3.5)
project(realsense_gazebo_plugin)

# Znalezienie wymaganych pakietów ROS 2 i Gazebo
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo REQUIRED)
find_package(realsense2 REQUIRED)

# Dodanie katalogów z plikami nagłówkowymi
include_directories(
    ${GAZEBO_INCLUDE_DIRS}
)

# Tworzenie biblioteki wtyczki
add_library(${PROJECT_NAME} SHARED
    src/gazebo_ros_realsense.cpp
    src/RealSensePlugin.cpp
)

# Linkowanie bibliotek
ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    sensor_msgs
    image_transport
    camera_info_manager
    gazebo_ros
    gazebo
    realsense2
)

target_link_libraries(${PROJECT_NAME}
    ${GAZEBO_LIBRARIES}
)

target_include_directories(${PROJECT_NAME} PRIVATE include)


# Instalacja plików binarnych
install(
    TARGETS ${PROJECT_NAME}
    DESTINATION lib/${PROJECT_NAME}
)

# Instalacja plików nagłówkowych
install(
    DIRECTORY include/
    DESTINATION include/${PROJECT_NAME}
)
install(
    DIRECTORY models/
    DESTINATION share/${PROJECT_NAME}/models
)


# Rejestracja paczki w ament_cmake
ament_package()


