#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <unordered_map>
#include <queue>
#include <limits>
#include <algorithm>
#include <cmath>
#include <vector>




class PlannerNode : public rclcpp::Node {
struct CellIndex
{
  int x;
  int y;
 
  CellIndex(int xx, int yy) : x(xx), y(yy) {}
  CellIndex() : x(0), y(0) {}
 
  bool operator==(const CellIndex &other) const
  {
    return (x == other.x && y == other.y);
  }
 
  bool operator!=(const CellIndex &other) const
  {
    return (x != other.x || y != other.y);
  }
};
 
// Hash function for CellIndex so it can be used in std::unordered_map
struct CellIndexHash
{
  std::size_t operator()(const CellIndex &idx) const
  {
    // A simple hash combining x and y
    return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
  }
};
 
// Structure representing a node in the A* open set
struct AStarNode
{
  CellIndex index;
  double f_score;  // f = g + h
 
  AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};
 
// Comparator for the priority queue (min-heap by f_score)
struct CompareF
{
  bool operator()(const AStarNode &a, const AStarNode &b)
  {
    // We want the node with the smallest f_score on top
    return a.f_score > b.f_score;
  }
};

public:
    PlannerNode() : Node("planner_node"), state_(State::WAITING_FOR_GOAL) {
        // Subscribers
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));
 
        // Publisher
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
 
        // Timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallback, this));
    }
 
private:
    enum class State { WAITING_FOR_GOAL, WAITING_FOR_ROBOT_TO_REACH_GOAL };
    State state_;
    
    // Subscribers and Publisher
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
 
    // Data Storage
    nav_msgs::msg::OccupancyGrid current_map_;
    geometry_msgs::msg::PointStamped goal_;
    geometry_msgs::msg::Pose robot_pose_;
 
    bool goal_received_ = false;
 
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        current_map_ = *msg;
        if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
            planPath();
        }
    }
 
    void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        goal_ = *msg;
        goal_received_ = true;
        state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
        planPath();
    }
 
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        robot_pose_ = msg->pose.pose;
    }
 
    void timerCallback() {
        if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
            if (goalReached()) {
                RCLCPP_INFO(this->get_logger(), "Goal reached!");
                state_ = State::WAITING_FOR_GOAL;
            } else {
                RCLCPP_INFO(this->get_logger(), "Replanning due to timeout or progress...");
                planPath();
            }
        }
    }
 
    bool goalReached() {
        double dx = goal_.point.x - robot_pose_.position.x;
        double dy = goal_.point.y - robot_pose_.position.y;
        return std::sqrt(dx * dx + dy * dy) < 0.5; // Threshold for reaching the goal
    }
 
    void planPath() {
        if (!goal_received_ || current_map_.data.empty()) {
            RCLCPP_WARN(this->get_logger(), "Cannot plan path: Missing map or goal!");
            return;
        }
 
        // A* Implementation (pseudo-code)
        nav_msgs::msg::Path path;
        path.header.stamp = this->get_clock()->now();
        path.header.frame_id = "sim_world";
        calculatePath(path);
        // Compute path using A* on current_map_
        // Fill path.poses with the resulting waypoints.
        
        path_pub_->publish(path);
    }

    void calculatePath(nav_msgs::msg::Path &path){
      int goal_grid_x = std::floor((goal_.point.x - current_map_.info.origin.position.x) / current_map_.info.resolution);
      int goal_grid_y = std::floor((goal_.point.y - current_map_.info.origin.position.y) / current_map_.info.resolution);

      
      if (goal_grid_x < 0 || goal_grid_x >= static_cast<int>(current_map_.info.width) ||
          goal_grid_y < 0 || goal_grid_y >= static_cast<int>(current_map_.info.height)) {
          RCLCPP_WARN(this->get_logger(), "Goal is outside the map!");
          return;
      }

      int robot_grid_x = std::floor((robot_pose_.position.x - current_map_.info.origin.position.x) / current_map_.info.resolution);
      int robot_grid_y = std::floor((robot_pose_.position.y - current_map_.info.origin.position.y) / current_map_.info.resolution);
      std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_q;
      std::unordered_map<CellIndex, double, CellIndexHash> gScore;
      
      std::unordered_map<CellIndex, double, CellIndexHash> fScore;
      std::unordered_map<CellIndex, CellIndex, CellIndexHash> cameFrom;
      
      auto distHeuristic = [goal_grid_x, goal_grid_y](double x, double y){
        return std::hypot((x-goal_grid_x), (y-goal_grid_y));

      };


      auto isBlocked = [this](const CellIndex& idx) -> bool {
        int map_index = idx.y * current_map_.info.width + idx.x;
        if (map_index < 0 || map_index >= static_cast<int>(current_map_.data.size())) {
          return true;  
        }
        return current_map_.data[map_index] >= 65;
      };


      
      
      for (uint32_t y = 0; y < current_map_.info.height; ++y) {
        for (uint32_t x = 0; x < current_map_.info.width; ++x) {
          CellIndex idx(static_cast<int>(x), static_cast<int>(y));
          gScore[idx] = std::numeric_limits<double>::infinity();
        }
      
      }

      gScore[CellIndex(robot_grid_x, robot_grid_y)] = 0;
      open_q.push(AStarNode(CellIndex(robot_grid_x, robot_grid_y), distHeuristic(robot_grid_x, robot_grid_y)));
      
      while (!open_q.empty()){
        AStarNode curr = open_q.top();
        open_q.pop();
        
        

        if (curr.index.x == goal_grid_x && curr.index.y == goal_grid_y) {
          constructPath(path, cameFrom, curr.index);
          return;
          //call create path
        }

        const int dx[8] = { -1, -1, -1,  0, 0, 1, 1, 1 };
        const int dy[8] = { -1,  0,  1, -1, 1,-1, 0, 1 };

        for (int i=0; i<8; i++){
          CellIndex curr_neighbor = CellIndex(curr.index.x + dx[i], curr.index.y+dy[i]);
          if (isBlocked(curr_neighbor)){
            continue;
          }
          double tentative_g = gScore.at(curr.index) + std::hypot(curr.index.x-curr_neighbor.x, curr.index.y-curr_neighbor.y);
          
          
          if (tentative_g < gScore.at(curr_neighbor)){
            cameFrom[curr_neighbor] = curr.index;
            gScore[curr_neighbor] = tentative_g;
            open_q.push(AStarNode(curr_neighbor, tentative_g+distHeuristic(curr_neighbor.x, curr_neighbor.y)));

          }
        }

      }
      RCLCPP_WARN(this->get_logger(), "A* failed to find a path to (%d,%d)", goal_grid_x, goal_grid_y);





    }

  


    void constructPath(nav_msgs::msg::Path &path, 
                      const std::unordered_map<CellIndex, CellIndex, CellIndexHash>& cameFrom, 
                      CellIndex goal) {
      // Reconstruct path from goal back to start
        std::vector<CellIndex> waypoints;
        CellIndex current = goal;
        
        while (cameFrom.find(current) != cameFrom.end()) {
          waypoints.push_back(current);
          current = cameFrom.at(current);
        }
        waypoints.push_back(current);  // Add start
        std::reverse(waypoints.begin(), waypoints.end());  // Reverse to start→goal
      
      
        
      
        for (auto& cell : waypoints) {
          geometry_msgs::msg::PoseStamped pose;
          pose.header.frame_id = "sim_world";
          pose.header.stamp = this->now();
          
          
          pose.pose.position.x = current_map_.info.origin.position.x + (cell.x + 0.5) * current_map_.info.resolution;
          pose.pose.position.y = current_map_.info.origin.position.y + (cell.y + 0.5) * current_map_.info.resolution;
          pose.pose.position.z = 0.0;
          pose.pose.orientation.w = 1.0;  
          
          path.poses.push_back(pose);
        }
    }
};




int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto planner_node = std::make_shared<PlannerNode>();
  rclcpp::spin(planner_node);
  rclcpp::shutdown();
  return 0;
}