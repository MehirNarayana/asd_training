#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_
 
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "costmap_core.hpp"
#include <vector>

class CostmapNode : public rclcpp::Node {
  public:
    CostmapNode();
    
    // Place callback function here
    void publishMessage();
    void receiveMessage(sensor_msgs::msg::LaserScan::SharedPtr msg);
  private:
    robot::CostmapCore costmap_;
    std::vector <std::vector<double>> gridLocal;
    // Place these constructs here
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr string_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub;
    rclcpp::TimerBase::SharedPtr timer_;
    void convertToGrid(float &range, float &angle, int &x_grid, int &y_grid);
    void markObstacle(int &x_grid, int &y_grid);
    void inflateObstacles();
    void setCost(int x, int y, double cost);
};
 
#endif 