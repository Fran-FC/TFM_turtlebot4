#include <chrono>
#include <fstream>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

class PoseToCSVNode : public rclcpp::Node
{
public:
  explicit PoseToCSVNode(const std::string& poseTopic, const std::string& outputFile)
      : Node("pose_to_csv_node"), poseTopic_(poseTopic), outputFile_(outputFile)
  {
    // Create the publisher and subscriber
    subscriber_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        poseTopic_, 10, std::bind(&PoseToCSVNode::poseCallback, this, std::placeholders::_1));

    // Open the CSV file for writing
    csvFile_.open(outputFile_, std::ios::out);
    if (!csvFile_.is_open())
    {
      RCLCPP_ERROR(get_logger(), "Failed to open CSV file: %s", outputFile_.c_str());
    }
    csvFile_ << "x y" << "\n";
  }

  ~PoseToCSVNode()
  {
    // Close the CSV file
    if (csvFile_.is_open())
    {
      csvFile_.close();
    }
  }

private:
  void poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    RCLCPP_INFO(get_logger(), "New pose registered: [%f, %f]", 
      msg->pose.pose.position.x, 
      msg->pose.pose.position.y
    );
    // Write the x and y poses to the CSV file
    csvFile_ << msg->pose.pose.position.x << " " << msg->pose.pose.position.y << "\n";
  }

  std::string poseTopic_;
  std::string outputFile_;
  std::ofstream csvFile_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscriber_;
};

int main(int argc, char** argv)
{
  if (argc < 3)
  {
    std::cerr << "usage: " << argv[0] << " <pose_topic> <output_file>" << std::endl;
    return 1;
  }

  rclcpp::init(argc, argv);
  auto node = std::make_shared<PoseToCSVNode>(argv[1], argv[2]);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
