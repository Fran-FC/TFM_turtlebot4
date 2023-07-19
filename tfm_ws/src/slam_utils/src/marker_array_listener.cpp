// Inside your_package_name/src/trajectory_subscriber.cpp
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <fstream>

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof(arr[0]))

class TrajectorySubscriber : public rclcpp::Node
{
public:
    TrajectorySubscriber(const std::string& poseTopic, const std::string& outputFile) 
        : Node("trajectory_subscriber"), file_name_(outputFile)
    {
        // Subscribe to the "/trajectory_node_list" topic using the message type visualization_msgs::msg::MarkerArray
        subscription_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
            poseTopic,
            10,
            std::bind(&TrajectorySubscriber::trajectoryCallback, this, std::placeholders::_1));
        
        // Open the CSV file for writing
        csv_file_.open(outputFile);
        if (!csv_file_.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file.");
        }
        csv_file_ << "x y" << "\n";
    }

    ~TrajectorySubscriber()
    {
        // Close the CSV file
        if (csv_file_.is_open())
        {
            for(const auto& point: last_marker_.points) {
                if(!existsPoint(point.x, point.y)) {
                    list_of_points_.push_back({point.x, point.y});
                    csv_file_ << point.x << " " << point.y << "\n";
                    RCLCPP_INFO(get_logger(), "New point: %f, %f", 
                        point.x, point.y
                    );
                }
            }
            csv_file_.close();
        }
    }

private:
    bool areEqual(double a, double b, double epsilon = 1e-3) {
        return std::abs(a - b) < epsilon;
    }

    bool existsPoint(float x, float y) {
        for(const auto& point : list_of_points_) 
            if(areEqual(x, point[0]) && areEqual(y, point[1])) 
                return true;
        
        return false;
    }

    void trajectoryCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
    {
        if(!msg->markers.empty())
            last_marker_ = msg->markers.back();
    }

    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr subscription_;
    std::ofstream csv_file_;
    std::string file_name_;
    std::vector<std::array<double, 2>> list_of_points_;
    visualization_msgs::msg::Marker last_marker_;
};

int main(int argc, char** argv)
{
    if (argc < 3) {
        std::cerr << "usage: " << argv[0] << " <pose_topic> <output_file>" << std::endl;
        return 1;
    }
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectorySubscriber>(argv[1], argv[2]));
    rclcpp::shutdown();
    return 0;
}
