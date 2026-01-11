#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include <cmath>

class MappingNode : public rclcpp::Node {
public:
    MappingNode() : Node("mapping_node"), last_x(0.0), last_y(0.0), distance_threshold(0) {
        // Initialize subscribers
        costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/costmap", 10, std::bind(&MappingNode::costmapCallback, this, std::placeholders::_1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom/filtered", 10, std::bind(&MappingNode::odomCallback, this, std::placeholders::_1));
 
        // Initialize publisher
        map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
 
        // Initialize timer
        timer_ = this->create_wall_timer(
            std::chrono::seconds(0), std::bind(&MappingNode::updateMap, this));

        global_map_.header.frame_id = "sim_world";
        global_map_.info.origin.position.x = -15.0;
        global_map_.info.origin.position.y = -15.0;
        global_map_.info.origin.position.z = 0;
        global_map_.info.width = 300; 
        global_map_.info.height = 300; 
        global_map_.info.resolution = 0.1;
        global_map_.data.resize(300 * 300, 0);
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

       

        

        
        int size = static_cast<int> (global_map_.info.width);
        float resolution =  (global_map_.info.resolution);
        


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

                global_map_.data[index] = latest_costmap_.data[indexTwo];
                //std::max(global_map_.data[index],latest_costmap_.data[indexTwo]);
                

                


            }
        }


        

        // Transform and merge the latest costmap into the global map
        // (Implementation would handle grid alignment and merging logic)
    }

//     void integrateCostmap() {
  
  

//   // Validate incoming costmap
//   if (latest_costmap_.data.empty()) return;

//   // Ensure global map initialized (if you initialize elsewhere, this is harmless)
//   if (global_map_.data.empty()) {
//     RCLCPP_WARN(this->get_logger(), "Global map not initialized; skipping integration");
//     return;
//   }

//   // Check resolution compatibility
//   const double in_res = latest_costmap_.info.resolution;
//   const double glob_res = global_map_.info.resolution;
//   if (!std::isfinite(in_res) || !std::isfinite(glob_res)) {
//     RCLCPP_WARN(this->get_logger(), "Invalid resolution(s); skipping integration");
//     return;
//   }
//   if (std::abs(in_res - glob_res) > 1e-9) {
//     RCLCPP_WARN(this->get_logger(), "Resolution mismatch: skipping integration");
//     return;
//   }

//   // Aliases
//   const int in_size = static_cast<int>(latest_costmap_.info.width); // square map
//   const double half_local = (in_size * in_res) / 2.0;
//   const double glob_ox = global_map_.info.origin.position.x;
//   const double glob_oy = global_map_.info.origin.position.y;
//   const int glob_w = static_cast<int>(global_map_.info.width);
//   const int glob_h = static_cast<int>(global_map_.info.height);

//   // Robot pose (world)
//   double rx = last_x;
//   double ry = last_y;
//   double rtheta = last_robot_theta;
//   double cos_t = std::cos(rtheta);
//   double sin_t = std::sin(rtheta);

//   // Iterate incoming (robot-relative) costmap cells
//   for (int yLocal = 0; yLocal < in_size; ++yLocal) {
//     for (int xLocal = 0; xLocal < in_size; ++xLocal) {
//       int inIdx = yLocal * in_size + xLocal;
//       int8_t inVal = latest_costmap_.data[inIdx];

//       // Skip unknowns in incoming costmap
//       if (inVal < 0) continue;

//       // Local coordinates of cell center (robot frame)
//       double lx = (xLocal + 0.5) * in_res - half_local; // x_local (meters)
//       double ly = (yLocal + 0.5) * in_res - half_local; // y_local (meters)

//       // Rotate then translate to world coordinates
//       double wx = rx + (lx * cos_t - ly * sin_t);
//       double wy = ry + (lx * sin_t + ly * cos_t);

//       // World -> global grid indices (use floor for consistent mapping)
//       int gx = static_cast<int>(std::floor((wx - glob_ox) / glob_res));
//       int gy = static_cast<int>(std::floor((wy - glob_oy) / glob_res));

//       if (gx < 0 || gx >= glob_w || gy < 0 || gy >= glob_h) continue;

//       int gidx = gy * glob_w + gx;

//       // Merge policy: preserve unknown in global map, otherwise take max after clamping
//       int existing = static_cast<int>(global_map_.data[gidx]);
//       int incoming = std::clamp<int>(static_cast<int>(inVal), 0, 100);

//       if (existing < 0) {
//         global_map_.data[gidx] = static_cast<int8_t>(incoming);
//       } else {
//         global_map_.data[gidx] = static_cast<int8_t>(std::max(existing, incoming));
//       }
//     }
//   }

//   // Update header stamp
//   global_map_.header.stamp = this->now();
// }






 
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