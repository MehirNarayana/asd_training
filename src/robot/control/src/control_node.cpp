#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>
#include <optional>
 
class PurePursuitController : public rclcpp::Node {
public:
    PurePursuitController() : Node("pure_pursuit_controller") {
        // Initialize parameters

        lookahead_distance_ = 1.0;  // Lookahead distance
        goal_tolerance_ = 0.5;     // Distance to consider the goal reached
        linear_speed_ = 0.5;       // Constant forward speed
        lastIndex=0;
        // Subscribers and Publishers
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/path", 10, [this](const nav_msgs::msg::Path::SharedPtr msg) { lastIndex=0; 
              current_path_ = msg; });
 
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom/filtered", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) { robot_odom_ = msg; });
 
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
 
        // Timer
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), [this]() { controlLoop(); });
    }
 
private:
    void controlLoop() {
        // Skip control if no path or odometry data is available
        if (!current_path_ || !robot_odom_) {
            return;
        }
 
        // Find the lookahead point
        auto lookahead_point = findLookaheadPoint();
        if (!lookahead_point) {
            
            return;  // No valid lookahead point found
        }
 
        // Compute velocity command
        auto cmd_vel = computeVelocity(*lookahead_point);
 
        // Publish the velocity command
        cmd_vel_pub_->publish(cmd_vel);
    }
 
    std::optional<geometry_msgs::msg::PoseStamped> findLookaheadPoint() {
      // std::vector<geometry_msgs::msg::PoseStamped> poses = current_path_->poses;
      //   int currIndex=0;
      //   for (const auto& currpose : poses){
          
      //     if ((computeDistance(currpose.pose.position, robot_odom_->pose.pose.position))>=lookahead_distance_ && currIndex>lastIndex){
      //       lastIndex=currIndex;
      //       return currpose;
      //     }
      //     currIndex+=1;
      //   }
      // return std::nullopt;  // Replace with a valid point when implemented  
      
      // TODO: Implement logic to find the lookahead point on the path
// if (!current_path_ || current_path_->poses.empty()) {
//         return std::nullopt;
//     }

//     auto robot_position = robot_odom_->pose.pose.position;
//     auto robot_yaw = extractYaw(robot_odom_->pose.pose.orientation);

//     double min_distance = std::numeric_limits<double>::max();
//     std::optional<geometry_msgs::msg::PoseStamped> best_point = std::nullopt;

//     for (const auto& currpose : current_path_->poses) {
//         double dx = currpose.pose.position.x - robot_position.x;
//         double dy = currpose.pose.position.y - robot_position.y;
//         double distance = std::hypot(dx, dy);

//         if (distance < lookahead_distance_) {
//             continue; // skip too close
//         }

//         double angle_to_point = std::atan2(dy, dx);
//         double angle_diff = angle_to_point - robot_yaw;
//         if (angle_diff > M_PI) angle_diff -= 2*M_PI;
//         if (angle_diff < -M_PI) angle_diff += 2*M_PI;

//         // Only consider points roughly in front
//         if (std::abs(angle_diff) < M_PI/2) {
//             if (distance < min_distance) {
//                 min_distance = distance;
//                 best_point = currpose;
//             }
//         }
//     }

    // return best_point;
      //   std::vector<geometry_msgs::msg::PoseStamped> curr_poses = current_path_->poses;
      //   for (const auto& currpose : current_path_->poses) {
      //     double dx = currpose.pose.position.x - robot_odom_->pose.pose.position.x;
      //     double dy = currpose.pose.position.y - robot_odom_->pose.pose.position.y;
      //     double distance = std::hypot(dx, dy);

      //     if (distance < lookahead_distance_) {
      //         continue; // skip too-close points
      //     }

      //     // Angle to point
      //     double angle_to_point = std::atan2(dy, dx);
      //     double robot_yaw = extractYaw(robot_odom_->pose.pose.orientation);

      //     // Normalize difference
      //     double angle_diff = angle_to_point - robot_yaw;
      //     if (angle_diff > M_PI) angle_diff -= 2*M_PI;
      //     if (angle_diff < -M_PI) angle_diff += 2*M_PI;

      //     // Only accept points roughly in front (±90°)
      //     if (std::abs(angle_diff) < M_PI/2) {
      //         return currpose; // first valid forward point
      //     }
      // }
      // return std::nullopt;

        
        //Replace with a valid point when implemented
      if (!current_path_ || current_path_->poses.empty()) {
        return std::nullopt; // Return no lookahead point if the path is empty
      }

    auto robot_position = robot_odom_->pose.pose.position;
    auto robot_yaw = extractYaw(robot_odom_->pose.pose.orientation);

    double min_distance = std::numeric_limits<double>::max();
    int lookahead_index = 0;
    bool found_forward = false;

    
    for (size_t i = 0; i < current_path_->poses.size(); ++i) {
        double dx = current_path_->poses[i].pose.position.x - robot_position.x;
        double dy = current_path_->poses[i].pose.position.y - robot_position.y;
        double distance = std::sqrt(dx * dx + dy * dy);

        
        if (distance < lookahead_distance_) {
            continue;
        }

        
        double angle_to_point = std::atan2(dy, dx);

        
        double angle_diff = angle_to_point - robot_yaw;

        
        if (angle_diff > M_PI) angle_diff -= 2 * M_PI;
        if (angle_diff < -M_PI) angle_diff += 2 * M_PI;

        
        if (std::abs(angle_diff) < M_PI / 2) {
            
            if (distance < min_distance) {
                min_distance = distance;
                lookahead_index = i;
                found_forward = true;
            }
        }
   }

    
    if (!found_forward) {
        
        for (size_t i = 0; i < current_path_->poses.size(); ++i) {
            double dx = current_path_->poses[i].pose.position.x - robot_position.x;
            double dy = current_path_->poses[i].pose.position.y - robot_position.y;
            double distance = std::sqrt(dx * dx + dy * dy);

            
            if (distance < lookahead_distance_) {
                continue;
            }

            
            if (distance < min_distance) {
                min_distance = distance;
                lookahead_index = i;
            }
        }
    }

    
    if (lookahead_index < current_path_->poses.size()) {
        return current_path_->poses[lookahead_index];
    }

    return std::nullopt; // Return no lookahead point if none was found
  }
 
    geometry_msgs::msg::Twist computeVelocity(const geometry_msgs::msg::PoseStamped &target) {
        // TODO: Implement logic to compute velocity commands
        double dx = target.pose.position.x - robot_odom_->pose.pose.position.x;

        double dy = target.pose.position.y - robot_odom_->pose.pose.position.y;
        double target_yaw = atan2(dy, dx);
        double curr_yaw = extractYaw(robot_odom_->pose.pose.orientation);
        double angle_error = atan2(sin(target_yaw - curr_yaw), cos(target_yaw - curr_yaw));
        
        double dist_error = std::hypot(dx, dy);
        geometry_msgs::msg::Twist cmd_vel;
        double angular_velocity_p = 0.20;
        double linear_velocity_p = 0.5;
        double forward_dot = dx * cos(curr_yaw) + dy * sin(curr_yaw); // Dot product with the forward direction
        double forward_dir = std::copysign(1.0, forward_dot);
        cmd_vel.linear.x = std::clamp(linear_velocity_p * dist_error, -0.5, 0.5) * forward_dir;

        cmd_vel.angular.z = std::clamp(angular_velocity_p * angle_error, -0.5, 0.5);
        if (dist_error < goal_tolerance_) {
            return geometry_msgs::msg::Twist{};
            
        }
        return cmd_vel;



        

    }
 
    double computeDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b) {
        
        double dist = std::sqrt((a.x - b.x)*(a.x - b.x) + (a.y-b.y)*(a.y-b.y));
        return dist;
    }
 
    double extractYaw(const geometry_msgs::msg::Quaternion &quat) {
      
      double yaw = std::atan2(2 * (quat.w * quat.z + quat.x * quat.y), 1 - 2 * (quat.y * quat.y + quat.z * quat.z));
      return yaw;
    }
 
    // Subscribers and Publishers
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
 
    // Timer
    rclcpp::TimerBase::SharedPtr control_timer_;
 
    // Data
    nav_msgs::msg::Path::SharedPtr current_path_;
    nav_msgs::msg::Odometry::SharedPtr robot_odom_;
 
    // Parameters
    int lastIndex;
    double lookahead_distance_;
    double goal_tolerance_;
    double linear_speed_;
};
 
// Main function
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PurePursuitController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

