#include "rclcpp/rclcpp.hpp"
#include "yolo_lidar/people_mapper_node.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<yolo_lidar::PeopleMapperNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
