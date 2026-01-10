#include <chrono>
#include <memory>
#include <functional>
 
#include "costmap_node.hpp"
#include <cmath>
CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // Initialize the constructs and their parameters
  string_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
  lidar_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
  "/lidar",
  rclcpp::SensorDataQoS(),
  std::bind(&CostmapNode::receiveMessage, this, std::placeholders::_1)
);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&CostmapNode::publishMessage, this));
  gridLocal = std::vector<std::vector<double>>(300, std::vector<double>(300, 0));
  
}
 
// Define the timer to publish a message every 500ms
void CostmapNode::publishMessage() {
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.stamp = this->now();
  grid.header.frame_id = "base_link";

  grid.info.resolution = 0.1;  // 10cm/cell
  grid.info.width = 300;
  grid.info.height = 300;
  grid.info.origin.position.x = 0.0;
  grid.info.origin.position.y = 0.0;
  grid.info.origin.orientation.w = 1.0;

  grid.data.assign(grid.info.width * grid.info.height, 0); // unknown

  // Mark a small occupied block
  for (int y = 0; y < 300; y++) {
    for (int x = 0; x < 300; x++) {
      grid.data[y * grid.info.width + x] = static_cast<int8_t>(gridLocal[y][x]);
    }
  }

  string_pub_->publish(grid);
}



void CostmapNode::receiveMessage(sensor_msgs::msg::LaserScan::SharedPtr msg){
  for (auto &row : gridLocal) std::fill(row.begin(), row.end(), 0.0);

  for (size_t i = 0; i < msg->ranges.size(); ++i) {
        float angle = msg->angle_min + i * msg->angle_increment;
        float range = msg->ranges[i];
        if (range < msg->range_max && range > msg->range_min) {
            // Calculate grid coordinates
            int x_grid, y_grid;
            convertToGrid(range, angle, x_grid, y_grid);
            markObstacle(x_grid, y_grid);
        }
      }
  inflateObstacles();
 
    // Step 4: Publish costmap
  publishMessage();    

}

void CostmapNode::convertToGrid(float &range, float &angle, int &x_grid, int &y_grid){
  x_grid = 150 + range*std::cos(angle) / 0.1;
  y_grid = 150+ range*std::sin(angle) / 0.1;
}

void CostmapNode::markObstacle(int &x_grid, int &y_grid){
  
  setCost(x_grid, y_grid, 100);

}

void CostmapNode::inflateObstacles(){
  for (int i = 0; i<300; i++){
    for (int j=0; j<300; j++){
      if (gridLocal[i][j]<100){
        continue;
      }

      int step = std::ceil(0.5 / 0.1);
      for (int dy = -step; dy<=step; dy++){
        for (int dx=-step; dx<=step; dx++){
          int curr_x = j+dx;
          int curr_y = i+dy; 

          if (curr_x < 0 || curr_x >= 300 || curr_y < 0 || curr_y >= 300) {
            continue;
          }

          double currDist = std::hypot(dx, dy) * 0.1;
          //need to check cuz its a circle
          if (currDist<0.5){
            float currCost = 100*(1-currDist/0.1);
            setCost(curr_x, curr_y, currCost);

          }
        }
      }

      


    }
  }
}

void CostmapNode::setCost(int x, int y, double cost){
  if (0<=x && x<=299 && 0<=y && y<=299){
    if (cost > gridLocal[y][x]){
      gridLocal[y][x] = cost;
    }
  }
}


 
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}