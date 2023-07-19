#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <fstream>
#include <iostream>
#include <vector>
#include <csignal>


class MapPoseListenerNode : public rclcpp::Node
{
public:
  MapPoseListenerNode() : Node("map_pose_listener")
  {
    csv_properties.csv_filename = "/home/fran/TFM_turtlebot4/tfm_ws/map.csv";
    csv_properties.csv_file = std::ofstream(csv_properties.csv_filename);
    if(csv_properties.csv_file.is_open())
      csv_properties.csv_file << csv_properties.csv_headers << std::endl;

    // Create a TF2 buffer and listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Create a timer to execute the callback at a fixed rate
    timer_ = create_wall_timer(std::chrono::milliseconds(500),
                               std::bind(&MapPoseListenerNode::tfCallback, this));
  }

  struct CSV_Struct{
    std::string csv_filename;
    const std::string csv_headers ="x y";
    std::vector<std::array<float, 2>> poses;
    std::ofstream csv_file;
  } csv_properties;

private:

  void tfCallback()
  {
    try
    {
      // Get the latest transform from "map" to "base_link"
      geometry_msgs::msg::TransformStamped transform_stamped;
      transform_stamped = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
    
      float pos_x = transform_stamped.transform.translation.x,
            pos_y = transform_stamped.transform.translation.y;

      // Print the translation and rotation
      RCLCPP_INFO(get_logger(), "Transform - Translation: [%f, %f]", pos_x, pos_y);

      csv_properties.poses.push_back({pos_x, pos_y});
      if(csv_properties.csv_file.is_open()) {
        csv_properties.csv_file << pos_x << " " << pos_y << std::endl;
      }
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_ERROR(get_logger(), "Failed to lookup transform: %s", ex.what());
    }
  }

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
};

// std::shared_ptr<MapPoseListenerNode> globalNodePtr = nullptr; // Global pointer to MyNode object

int main(int argc, char **argv)
{
  std::vector<std::string> ros_args = rclcpp::init_and_remove_ros_arguments(argc, argv);
  
  auto node = std::make_shared<MapPoseListenerNode>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
