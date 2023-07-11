#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

class MapSubscriberNode : public rclcpp::Node
{
public:
  MapSubscriberNode() : Node("occupacy_mean_node")
  {
    // Create a subscription to the "/map" topic
    map_subscription_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map",
        rclcpp::QoS(rclcpp::KeepLast(10)),
        std::bind(&MapSubscriberNode::mapCallback, this, std::placeholders::_1));
  }

private:
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    // Process the received map data here
    // You can access the map data using 'msg->data', its width and height using 'msg->info.width' and 'msg->info.height', etc.
    int sum = 0;
    int s = 0;
    for (const auto& value : msg->data)
    {
      if(value >=  0){
        sum += value;
        s++;
      }
    }
    double mean = static_cast<double>(sum) / s;
    // Print the mean occupancy value
    RCLCPP_INFO(get_logger(), "Mean Occupancy: %.2f", mean);
    
  }

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MapSubscriberNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
