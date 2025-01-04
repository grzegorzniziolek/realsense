#ifndef _GAZEBO_ROS_REALSENSE_PLUGIN_
#define _GAZEBO_ROS_REALSENSE_PLUGIN_

#include "realsense_gazebo_plugin/RealSensePlugin.h"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <image_transport/image_transport.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <gazebo/physics/physics.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/transport/transport.hh>
#include <sdf/sdf.hh>
#include <string>
#include <memory>
#include <map>

namespace gazebo {

/// \brief Klasa symulująca kamerę RealSense w Gazebo
class GazeboRosRealsense : public RealSensePlugin {
public:
  /// \brief Konstruktor
  GazeboRosRealsense();

  /// \brief Destruktor
  ~GazeboRosRealsense();

  /// \brief Funkcja ładowania wtyczki
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

  /// \brief Funkcja obsługująca nowe ramki głębi
  virtual void OnNewDepthFrame();

  /// \brief Funkcja obsługująca nowe ramki obrazu
  virtual void OnNewFrame(const rendering::CameraPtr cam, const transport::PublisherPtr pub);

protected:
  /// \brief Wskaźnik na węzeł ROS 2
  std::shared_ptr<rclcpp::Node> node_;

  /// \brief Menedżer informacji o kamerze
  std::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;

  /// \brief Publikator chmury punktów
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;

  /// \brief Publikatory obrazów
  image_transport::CameraPublisher color_pub_;
  image_transport::CameraPublisher depth_pub_;

protected:
  /// \brief Wiadomości obrazów i chmury punktów
  sensor_msgs::msg::Image image_msg_;
  sensor_msgs::msg::Image depth_msg_;
  sensor_msgs::msg::PointCloud2 pointcloud_msg_;

  /// \brief Mapa parametrów kamery
  std::map<std::string, CameraParams> cameraParamsMap_;

private:
  /// \brief Nazwa tematu dla chmury punktów
  std::string pointCloudTopic_;

  /// \brief Czy chmura punktów jest włączona
  bool pointCloud_;
};

}  // namespace gazebo

namespace {

/// \brief Funkcja pomocnicza do wyodrębniania nazwy kamery
std::string extractCameraName(const std::string &name);

/// \brief Funkcja pomocnicza do generowania wiadomości CameraInfo
sensor_msgs::msg::CameraInfo cameraInfo(const sensor_msgs::msg::Image &image, float horizontal_fov);

}  // namespace

#endif /* _GAZEBO_ROS_REALSENSE_PLUGIN_ */


