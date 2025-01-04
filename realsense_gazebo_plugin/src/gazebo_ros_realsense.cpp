#include "realsense_gazebo_plugin/gazebo_ros_realsense.h"

#include <algorithm>  // std::copy
#include <string>

namespace gazebo {

void fillImage(sensor_msgs::msg::Image &image_msg,
               const std::string &encoding, uint32_t height, uint32_t width,
               uint32_t step, const void *data) {
    RCLCPP_DEBUG(rclcpp::get_logger("GazeboRosRealsense"), "Filling image with encoding: %s", encoding.c_str());
    image_msg.encoding = encoding;
    image_msg.height = height;
    image_msg.width = width;
    image_msg.step = step;
    image_msg.data.resize(step * height);

    const uint8_t *data_ptr = reinterpret_cast<const uint8_t *>(data);
    std::copy(data_ptr, data_ptr + step * height, image_msg.data.begin());
    RCLCPP_DEBUG(rclcpp::get_logger("GazeboRosRealsense"), "Image filled successfully.");
}

// Rejestracja wtyczki w Gazebo
GZ_REGISTER_MODEL_PLUGIN(GazeboRosRealsense)

GazeboRosRealsense::GazeboRosRealsense() {
    RCLCPP_DEBUG(rclcpp::get_logger("GazeboRosRealsense"), "Realsense plugin constructor called.");
}

GazeboRosRealsense::~GazeboRosRealsense() {
    RCLCPP_DEBUG(this->node_->get_logger(), "Realsense plugin unloaded.");
}

void GazeboRosRealsense::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
    RCLCPP_INFO(rclcpp::get_logger("GazeboRosRealsense"), "Loading Realsense plugin...");

    if (!rclcpp::ok()) {
        RCLCPP_FATAL(rclcpp::get_logger("GazeboRosRealsense"),
                     "ROS 2 node not initialized. Unable to load plugin.");
        return;
    }

    RCLCPP_INFO(rclcpp::get_logger("GazeboRosRealsense"), "ROS 2 node initialized.");

    this->node_ = rclcpp::Node::make_shared("realsense_plugin_node");

    // Inicjalizacja menedżera informacji o kamerze
    RCLCPP_INFO(this->node_->get_logger(), "Initializing camera info manager...");
    this->camera_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(
        this->node_.get(), "realsense_camera");

    this->color_pub_ = image_transport::create_camera_publisher(
        this->node_.get(), "camera/color/image_raw");
    RCLCPP_INFO(this->node_->get_logger(), "Color camera publisher created.");

    this->depth_pub_ = image_transport::create_camera_publisher(
        this->node_.get(), "camera/depth/image_raw");
    RCLCPP_INFO(this->node_->get_logger(), "Depth camera publisher created.");

    this->pointcloud_pub_ = this->node_->create_publisher<sensor_msgs::msg::PointCloud2>(
        "camera/points", 10);
    RCLCPP_INFO(this->node_->get_logger(), "PointCloud publisher created.");

    RCLCPP_INFO(this->node_->get_logger(), "Realsense plugin successfully loaded.");
}

void GazeboRosRealsense::OnNewFrame(const rendering::CameraPtr cam, const transport::PublisherPtr pub) {
    RCLCPP_DEBUG(this->node_->get_logger(), "OnNewFrame called.");
    common::Time current_time = this->world->SimTime();

    // Identyfikacja kamery
    RCLCPP_DEBUG(this->node_->get_logger(), "Extracting camera name...");
    std::string camera_id = extractCameraName(cam->Name());
    RCLCPP_INFO(this->node_->get_logger(), "Camera ID: %s", camera_id.c_str());

    const std::map<std::string, image_transport::CameraPublisher> camera_publishers = {
        {"color", this->color_pub_},
        {"depth", this->depth_pub_},
    };

    if (camera_publishers.find(camera_id) == camera_publishers.end()) {
        RCLCPP_ERROR(this->node_->get_logger(), "Camera ID '%s' not found in publishers.", camera_id.c_str());
        return;
    }

    const auto &image_pub = camera_publishers.at(camera_id);

    // Kopiowanie danych do obrazu
    this->image_msg_.header.frame_id = this->cameraParamsMap_[camera_id].optical_frame;
    this->image_msg_.header.stamp = rclcpp::Time(current_time.sec, current_time.nsec);

    // Ustawienie kodowania obrazu
    const std::map<std::string, std::string> supported_image_encodings = {
        {"RGB_INT8", sensor_msgs::image_encodings::RGB8},
        {"L_INT8", sensor_msgs::image_encodings::TYPE_8UC1}};
    const auto pixel_format = supported_image_encodings.at(cam->ImageFormat());

    RCLCPP_DEBUG(this->node_->get_logger(), "Copying image data to ROS message...");
    gazebo::fillImage(
        this->image_msg_, pixel_format, cam->ImageHeight(),
        cam->ImageWidth(), cam->ImageDepth() * cam->ImageWidth(),
        reinterpret_cast<const void *>(cam->ImageData()));

    // Publikacja obrazu
    auto camera_info_msg = cameraInfo(this->image_msg_, cam->HFOV().Radian());
    RCLCPP_INFO(this->node_->get_logger(), "Publishing image and camera info for '%s'.", camera_id.c_str());
    image_pub.publish(this->image_msg_, camera_info_msg);
}

}  // namespace gazebo

namespace {

std::string extractCameraName(const std::string &name) {
    if (name.find("color") != std::string::npos) {
        return "color";
    }
    if (name.find("depth") != std::string::npos) {
        return "depth";
    }

    RCLCPP_ERROR(rclcpp::get_logger("GazeboRosRealsense"), "Unknown camera name in '%s'.", name.c_str());
    return "color";
}

sensor_msgs::msg::CameraInfo cameraInfo(const sensor_msgs::msg::Image &image, float horizontal_fov) {
    sensor_msgs::msg::CameraInfo info_msg;

    info_msg.header = image.header;
    info_msg.distortion_model = "plumb_bob";
    info_msg.height = image.height;
    info_msg.width = image.width;

    float focal = 0.5 * image.width / tan(0.5 * horizontal_fov);

    info_msg.k[0] = focal;
    info_msg.k[4] = focal;
    info_msg.k[2] = info_msg.width * 0.5;
    info_msg.k[5] = info_msg.height * 0.5;
    info_msg.k[8] = 1.;

    info_msg.p[0] = info_msg.k[0];
    info_msg.p[5] = info_msg.k[4];
    info_msg.p[2] = info_msg.k[2];
    info_msg.p[6] = info_msg.k[5];
    info_msg.p[10] = info_msg.k[8];

    RCLCPP_DEBUG(rclcpp::get_logger("GazeboRosRealsense"), "CameraInfo message created.");
    return info_msg;
}

}  // namespace



