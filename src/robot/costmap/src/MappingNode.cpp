#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include <cmath>

class MappingNode : public rclcpp::Node {
public:
    MappingNode() : Node("mapping_node"), last_x(0.0), last_y(0.0), distance_threshold(1.5) {
        // Initialize subscribers
        costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/costmap", 10, std::bind(&MappingNode::costmapCallback, this, std::placeholders::_1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom/filtered", 10, std::bind(&MappingNode::odomCallback, this, std::placeholders::_1));
 
        // Initialize publisher
        map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
 
        // Initialize timer
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&MappingNode::updateMap, this));
    }
 
private:
    // Subscribers and Publisher
    
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
 
    // Global map and robot position
    nav_msgs::msg::OccupancyGrid global_map_;
    double last_x, last_y;
    double last_robot_theta = 0;
    
    const double distance_threshold;
    bool costmap_updated_ = false;
 
    // Callback for costmap updates
    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        // Store the latest costmap
        latest_costmap_ = *msg;
        costmap_updated_ = true;
    }
    
   

    // Callback for odometry updates
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;
        double robot_theta = tf2::getYaw(msg->pose.pose.orientation);
        
// Compute distance traveled
        double distance = std::sqrt(std::pow(x - last_x, 2) + std::pow(y - last_y, 2));
        if (distance >= distance_threshold) {
            last_x = x;
            last_y = y;
            last_robot_theta = robot_theta;
            should_update_map_ = true;
        }
    }
 
    // Timer-based map update
    void updateMap() {
        auto now = this->get_clock()->now();
        if (should_update_map_ && costmap_updated_) {
            integrateCostmap();
            global_map_.header.stamp = this->now();
            map_pub_->publish(global_map_);
            should_update_map_ = false;
            
        }
    }
 
    // Integrate the latest costmap into the global map
    void integrateCostmap() {

        if (global_map_.data.empty()) {
            
            global_map_.header.frame_id = "sim_world";   
            global_map_.info.width = 300;
            global_map_.info.height = 300;
            global_map_.info.resolution = latest_costmap_.info.resolution;
            global_map_.info.origin.position.x = -15.0;
            global_map_.info.origin.position.y = -15.0;
            
            
            global_map_.data.assign(global_map_.info.width * global_map_.info.height, 0);
        }

        

        
        int size = static_cast<int> (latest_costmap_.info.width);
        float resolution =  (latest_costmap_.info.resolution);
        


        for (int yLocal=0;yLocal<size; yLocal++){
            for (int xLocal=0;xLocal<size;xLocal++){
                double xMeters = resolution*(xLocal-size/2.0);
                double yMeters = resolution*(yLocal-size/2.0);
                double rotatedX = xMeters*std::cos(last_robot_theta) - yMeters*std::sin(last_robot_theta);
                double rotatedY = xMeters*std::sin(last_robot_theta) + yMeters*std::cos(last_robot_theta);
                double translatedX = last_x+rotatedX;
                double translatedY = last_y+rotatedY;
                int gridX  = translatedX/resolution + 150;
                int gridY = translatedY/resolution + 150;
                // double xMeters = resolution * (xLocal - size / 2.0);
                // double yMeters = resolution * (yLocal - size / 2.0);
                // double translatedX = last_x + xMeters;
                // double translatedY = last_y + yMeters;
                // int gridX = translatedX / resolution + 150;
                // int gridY = translatedY / resolution + 150;
                if (gridX < 0 || gridX >= 300 || gridY < 0 || gridY >= 300) continue;

                int index = gridY*300+gridX;
                int indexTwo = yLocal * size + xLocal;
                global_map_.data[index] = std::max(global_map_.data[index],latest_costmap_.data[indexTwo]);


            }
        }


        

        // Transform and merge the latest costmap into the global map
        // (Implementation would handle grid alignment and merging logic)
    }





 
    // Flags
    nav_msgs::msg::OccupancyGrid latest_costmap_;
    bool should_update_map_ = false;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MappingNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}