/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Robert Bosch LLC.
 *  Copyright (c) 2015-2016, Jiri Horner.
 *  Copyright (c) 2021, Carlos Alvarez, Juan Galvis.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Jiri Horner nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

#include <explore/explore.h>

#include <thread>
#include <chrono>

/// @brief Checks whether two points are within 20cm of eachother
/// @param one The first point to check
/// @param two The second point to check
/// @return Boolean value - True if points are proximal
inline static bool same_point(const geometry_msgs::msg::Point& one,
                              const geometry_msgs::msg::Point& two)
{
  double dx = one.x - two.x;
  double dy = one.y - two.y;
  double dist = sqrt(dx * dx + dy * dy);
  return dist < 0.2;
}

namespace explore
{

/// @brief Constructor for the Explore class
Explore::Explore()
  : Node("explore_node")
  , logger_(this->get_logger())
  , tf_buffer_(this->get_clock())
  , tf_listener_(tf_buffer_)
  , costmap_client_(*this, &tf_buffer_)
  , prev_distance_(0)
  , last_markers_count_(0)
{
  double timeout;
  double min_frontier_size;
  this->declare_parameter<float>("planner_frequency", 1.0);
  this->declare_parameter<float>("progress_timeout", 30.0);
  this->declare_parameter<bool>("visualize", false);
  this->declare_parameter<float>("potential_scale", 1e-3);
  this->declare_parameter<float>("orientation_scale", 0.0);
  this->declare_parameter<float>("gain_scale", 1.0);
  this->declare_parameter<float>("min_frontier_size", 0.5);
  this->declare_parameter<std::string>("node_status", "idle");
  this->declare_parameter<double>("recovery_delta", 0.3);
  this->declare_parameter<double>("target_prox_lim", 0.2);
  this->declare_parameter<double>("goalpause_timeout", 2.0);

  this->get_parameter("planner_frequency", planner_frequency_);
  this->get_parameter("progress_timeout", timeout);
  this->get_parameter("visualize", visualize_);
  this->get_parameter("potential_scale", potential_scale_);
  this->get_parameter("orientation_scale", orientation_scale_);
  this->get_parameter("gain_scale", gain_scale_);
  this->get_parameter("min_frontier_size", min_frontier_size);
  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("recovery_delta", recovery_delta_);
  this->get_parameter("target_prox_lim", target_prox_lim_);
  this->get_parameter("goalpause_timeout", goalpause_timeout_);
  progress_timeout_ = timeout;

  // Client accessing move base server for goal setting and feedback
  move_base_client_ =
      rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
          this, ACTION_NAME);
        
  // Subscription to local costmap
  local_costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/local_costmap/costmap", 10,
    [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        local_costmap_ = msg;
    });

  // Subscription to global costmap
  global_costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/global_costmap/costmap", 10,
    [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        global_costmap_ = msg;
    });
  
  // Frontier search object used to generate frontiers
  search_ = frontier_exploration::FrontierSearch(costmap_client_.getCostmap(),
                                                 potential_scale_, gain_scale_,
                                                 min_frontier_size, logger_);
                                          
  // Service exposed to allow controlling nodes to restart frontier exploration  
  make_plan_service_ = this->create_service<std_srvs::srv::Empty>(
    "/make_plan",
    std::bind(&Explore::makePlanCallback, this, std::placeholders::_1, std::placeholders::_2));                                          
  
  // Frontier visualisation publisher
  if (visualize_) {
    marker_array_publisher_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>("explore/"
                                                                     "frontier"
                                                                     "s",
                                                                     10);
  }

  // Publisher for the status of the explore_lite node
  explore_status_publisher = 
    this->create_publisher<std_msgs::msg::String>("explore/status", 10);

  // Publisher for the list of blacklisted goals (published on exit for post-processing)
  explore_blacklist_publisher =
    this->create_publisher<geometry_msgs::msg::PoseArray>("explore/blacklist", 10);

  // Timer for status publishing
  status_timer = this->create_wall_timer(
      std::chrono::seconds(1),
      [this]() { statusCallback(); });
  
  // Subscription to resume or stop exploration
  resume_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
      "explore/resume", 10,
      std::bind(&Explore::resumeCallback, this, std::placeholders::_1));

  RCLCPP_INFO(logger_, "Waiting to connect to move_base nav2 server");
  move_base_client_->wait_for_action_server();
  RCLCPP_INFO(logger_, "Connected to move_base nav2 server");

  // Timer that triggers the planning loop 
  exploring_timer_ = this->create_wall_timer(
      std::chrono::milliseconds((uint16_t)(1000.0 / planner_frequency_)),
      [this]() { makePlan(); });
  // Start exploration right away
  this->set_parameter(rclcpp::Parameter("node_status", "exploring"));
  makePlan();
}

/// @brief Destructor for the Explore class
Explore::~Explore()
{
  stop();
}

/// @brief Callback function that publishes the status of the node
void Explore::statusCallback() 
{
  std::string status = this->get_parameter("node_status").as_string();

  auto msg = std_msgs::msg::String();
  msg.data = status;
  explore_status_publisher->publish(msg);
}

/// @brief Callback function that re-initialises the search and re-starts the planning loop
/// @param ROS2 Request type
/// @param  ROS2 Response type
void Explore::makePlanCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/)
{
  // Immediately publish status back to the controlling node
  statusCallback();

  // Reset critical values
  last_progress_ = this->now();
  this->set_parameter(rclcpp::Parameter("node_status", "exploring"));
  RCLCPP_INFO(this->get_logger(), "makePlan service called, triggering replanning.");
  exploring_timer_->reset();
  frontier_blacklist_.clear();
  first_pass_ = true;

  // Re-start the planning loop
  this->makePlan();
}

/// @brief Callback to manage pause/resume feature
/// @param msg Whether or not to resume search
void Explore::resumeCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data) {
    resume();
  } else {
    stop();
  }
}

/// @brief Function to visualise frontiers to relevant ROS2 topics
/// @param frontiers Vector of frontier vlasses to visualise
void Explore::visualizeFrontiers(
    const std::vector<frontier_exploration::Frontier>& frontiers)
{
  std_msgs::msg::ColorRGBA blue;
  blue.r = 0;
  blue.g = 0;
  blue.b = 1.0;
  blue.a = 1.0;
  std_msgs::msg::ColorRGBA red;
  red.r = 1.0;
  red.g = 0;
  red.b = 0;
  red.a = 1.0;
  std_msgs::msg::ColorRGBA green;
  green.r = 0;
  green.g = 1.0;
  green.b = 0;
  green.a = 1.0;

  RCLCPP_DEBUG(logger_, "visualising %lu frontiers", frontiers.size());
  visualization_msgs::msg::MarkerArray markers_msg;
  std::vector<visualization_msgs::msg::Marker>& markers = markers_msg.markers;
  visualization_msgs::msg::Marker m;

  m.header.frame_id = costmap_client_.getGlobalFrameID();
  m.header.stamp = this->now();
  m.ns = "frontiers";
  m.scale.x = 1.0;
  m.scale.y = 1.0;
  m.scale.z = 1.0;
  m.color.r = 0;
  m.color.g = 0;
  m.color.b = 255;
  m.color.a = 255;
  // lives forever
#ifdef ELOQUENT
  m.lifetime = rclcpp::Duration(0);  // deprecated in galactic warning
#elif DASHING
  m.lifetime = rclcpp::Duration(0);  // deprecated in galactic warning
#else
  m.lifetime = rclcpp::Duration::from_seconds(0);  // foxy onwards
#endif
  // m.lifetime = rclcpp::Duration::from_nanoseconds(0); // suggested in
  // galactic
  m.frame_locked = true;

  // weighted frontiers are always sorted
  double min_cost = frontiers.empty() ? 0. : frontiers.front().cost;

  m.action = visualization_msgs::msg::Marker::ADD;
  size_t id = 0;
  for (auto& frontier : frontiers) {
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.id = int(id);
    m.pose.position = frontier.initial;
    // scale frontier according to its cost (costier frontiers will be smaller)
    double scale = std::min(std::abs(min_cost * 0.4 / frontier.cost), 0.5);
    m.scale.x = scale;
    m.scale.y = scale;
    m.scale.z = scale;
    m.points = {};
    m.color = green;
    markers.push_back(m);
    ++id;
  }
  size_t current_markers_count = markers.size();

  // delete previous markers, which are now unused
  m.action = visualization_msgs::msg::Marker::DELETE;
  for (; id < last_markers_count_; ++id) {
    m.id = int(id);
    markers.push_back(m);
  }

  last_markers_count_ = current_markers_count;
  marker_array_publisher_->publish(markers_msg);
}

/// @brief Get the value of a point in the world frame from a given costmap
/// @param wx x-coord of test point
/// @param wy y-coord of test point
/// @param costmap occupancy grid to query
/// @return value (0-255) representing the cost of the point
int Explore::costmapVal(double wx, double wy, nav_msgs::msg::OccupancyGrid::SharedPtr costmap) {
    if (!costmap) return -1;
    double origin_x = costmap->info.origin.position.x;
    double origin_y = costmap->info.origin.position.y;
    double resolution = costmap->info.resolution;
    int width = costmap->info.width;
    int height = costmap->info.height;

    int mx = static_cast<int>((wx - origin_x) / resolution);
    int my = static_cast<int>((wy - origin_y) / resolution);

    if (mx < 0 || my < 0 || mx >= width || my >= height) return -1;
    int idx = my * width + mx;
    int value = costmap->data[idx];
    return value;
}

/// @brief Main planning loop of the node - selects next frontier and publishes 
void Explore::makePlan()
{
  // Reset timer on first pass
  if (first_pass_) {
    last_progress_ = this->now();
  }

  // black list if we've made no progress for a long time
  if ((this->now() - last_progress_ >
      tf2::durationFromSec(progress_timeout_)) && !resuming_) {
    frontier_blacklist_.push_back(prev_centroid_);
    last_progress_ = this->now();
    RCLCPP_INFO(logger_, "Timeout: Adding current goal to black list");
    move_base_client_->async_cancel_all_goals();
    return;
  }

  // Uninitialised or not enough time has elapsed between goals 
  if (last_goal_.nanoseconds() != 0 && this->now() - last_goal_ < 
      tf2::durationFromSec(goalpause_timeout_)) {
        return;
      }

  // find frontiers
  auto pose = costmap_client_.getRobotPose();
  
  // get frontiers sorted according to cost
  auto frontiers = search_.searchFrom(pose.position);
  
  if (!goal_active_) {
    RCLCPP_INFO(logger_, "found %lu frontiers", frontiers.size());
    for (size_t i = 0; i < frontiers.size(); ++i) {
      RCLCPP_DEBUG(logger_, "frontier %zd cost: %f", i, frontiers[i].cost);
    }
  }
  

  if (frontiers.empty()) {
    RCLCPP_WARN(logger_, "No frontiers found, stopping.");
    stop(true);
    return;
  }

  // publish frontiers as visualization markers
  if (visualize_) {
    visualizeFrontiers(frontiers);
  }

  // find non blacklisted frontier
  auto frontier =
      std::find_if_not(frontiers.begin(), frontiers.end(),
                       [this](const frontier_exploration::Frontier& f) {
                         return goalOnBlacklist(f.centroid);
                       });
  if (frontier == frontiers.end()) {
    RCLCPP_WARN(logger_, "All frontiers traversed/tried out, stopping.");
    stop(true);
    return;
  }

  // Track the target position for the robot
  geometry_msgs::msg::Point target_position = frontier->centroid;

  // Track the current frontier centroid being targeted 
  geometry_msgs::msg::Point target_centroid = target_position;

  // check whether proposed target is at the robot position
  bool null_target =
    (std::abs(target_position.x - pose.position.x) < target_prox_lim_) &&
    (std::abs(target_position.y - pose.position.y) < target_prox_lim_);

  // If target is ontop of the robot, project along the frontier direction
  if (null_target) {
    if (!goal_active_) {
      RCLCPP_INFO(logger_, "Frontier target set to current robot position: projecting goal along frontier direction");
    }
    
    
    // Get direction to frontier
    double dx = target_position.x - pose.position.x;
    double dy = target_position.y - pose.position.y;
    
    // Normalise
    double len = std::sqrt(dx*dx + dy*dy);
    dx /= len; dy /= len;

    // Project away from the robot
    target_position.x = pose.position.x + dx * recovery_delta_;
    target_position.y = pose.position.y + dy * recovery_delta_;
  }

  
  // Get local and global costmap values for proposed position
  int local_costmap_val = costmapVal(target_position.x, target_position.y, local_costmap_);
  int global_costmap_val = costmapVal(target_position.x, target_position.y, global_costmap_);
  if (!goal_active_) {
    RCLCPP_INFO(logger_, "Local costmap val at target is: %d.", local_costmap_val);
    RCLCPP_INFO(logger_, "Global costmap val at target is: %d.", global_costmap_val);  
  }
  

  int max_costmap_target_val = 75;
  int max_costmap_val = std::max<int>(local_costmap_val, global_costmap_val);
  
  // If target position is too close to a wall, attempt to move it to a lower cost position
  if (max_costmap_val >= max_costmap_target_val) {

    // Search parameters
    double search_radius = 6;
    double search_resolution = 0.05;

    bool found = false;
    for (int dx = -search_radius; dx <= search_radius && !found; ++dx) {
      for (int dy = -search_radius; dy <= search_radius && !found; ++dy) {

        // Calculate candidate position and find max costmap value
        double test_x = target_position.x + static_cast<double>(dx) * search_resolution;
        double test_y = target_position.y + static_cast<double>(dy) * search_resolution;
        
        // Do not send trivial targets
        if (std::abs(test_x - pose.position.x) < target_prox_lim_ &&
            std::abs(test_y - pose.position.y) < target_prox_lim_) {
              continue;
            }
        
        int global_cost = costmapVal(test_x, test_y, global_costmap_);
        int local_cost = costmapVal(test_x, test_y, local_costmap_);
        int max_costmap_val = std::max<int>(local_cost, global_cost);

        // Terminate search upon finding a valid target
        if (max_costmap_val < max_costmap_target_val) {
          RCLCPP_INFO(logger_, "Moved target from obstacle: (%.2f, %.2f) -> (%.2f, %.2f)", 
            target_position.x, target_position.y, test_x, test_y);
          target_position.x = test_x;
          target_position.y = test_y;
          found = true;
        }
      }
    }

    // If never found a safe place to send robot, blacklist and return
    if (found == false) {
      RCLCPP_INFO(logger_, "Could not find low-cost neighbour. Blacklisting.");
      frontier_blacklist_.push_back(target_centroid);
      return;
    }
  }

  // Test for whether we are pursuing the same fontier
  // and whether we are pursuing the same goal
  bool same_centroid = same_point(prev_centroid_, target_centroid);
  bool same_goal = same_point(prev_goal_, target_position);
  prev_goal_ = target_position;
  prev_centroid_ = target_centroid;

  // If we have changed frontiers or we are making progress to a goal, reset
  if (first_pass_ || !same_centroid || (same_goal && (prev_distance_ > frontier->min_distance))) {
    // we have different goal or we made some progress
    last_progress_ = this->now();
    prev_distance_ = frontier->min_distance;
  }

  // Ensure only first call of makePlan was set resuming to true
  if (resuming_) {
    resuming_ = false;
  }

  if (goal_active_) {
    // Robot is already pursuing a goal, don't send a new one
    return;
  }
  
  // Compute yaw angle between robot and target
  double dx = target_position.x - pose.position.x;
  double dy = target_position.y - pose.position.y;
  double yaw = std::atan2(dy, dx);

  // Convert yaw to quaternion
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);

  // Populate target goal if all checks have passed
  auto goal = nav2_msgs::action::NavigateToPose::Goal();
  goal.pose.pose.position = target_position;
  goal.pose.pose.orientation = tf2::toMsg(q);
  goal.pose.header.frame_id = costmap_client_.getGlobalFrameID();
  goal.pose.header.stamp = this->now();

  RCLCPP_INFO(logger_, "Sending frontier target: ({%.2f}, {%.2f})", target_position.x, target_position.y);

  // Log goal 
  last_goal_ = this->now();
  auto send_goal_options =
      rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();

  // Attach a callback for when goal is achieved
  send_goal_options.result_callback =
      [this,
       target_position](const NavigationGoalHandle::WrappedResult& result) {
        reachedGoal(result, target_position);
      };
  
  // Set flags and send goal
  goal_active_ = true;
  move_base_client_->async_send_goal(goal, send_goal_options);
  first_pass_ = false;
}

/// @brief Check whether a goal is blacklisted
/// @param goal x-y coordinated of the goal
/// @return boolean - whether the goal is blacklisted
bool Explore::goalOnBlacklist(const geometry_msgs::msg::Point& goal)
{

  // check if a goal is on the blacklist for goals that we're pursuing
  for (auto& frontier_goal : frontier_blacklist_) {
    double x_diff = fabs(goal.x - frontier_goal.x);
    double y_diff = fabs(goal.y - frontier_goal.y);

    if (sqrt(x_diff * x_diff + y_diff * y_diff) < 0.35) {
      return true;
    }
  }
  return false;
}
/// @brief Callback for when a goal is reached
/// @param result result from move base server
/// @param frontier_goal the goal attempted
void Explore::reachedGoal(const NavigationGoalHandle::WrappedResult& result,
                          const geometry_msgs::msg::Point& frontier_goal)
{
  // Enable a new goal to be sent
  goal_active_ = false; 

  // Check result code
  switch (result.code) {

    // No action on success - wait for next planning loop
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_DEBUG(logger_, "Goal was successful. Pausing for 2s");
      std::this_thread::sleep_for(std::chrono::seconds(2));
      break;

    // If aborted, blacklist the attempted point
    case rclcpp_action::ResultCode::ABORTED:
    {
      RCLCPP_INFO(logger_, "Goal was aborted");

      // Get the robot's current pose
      auto current_pose = costmap_client_.getRobotPose();

      // Compute distance to the frontier goal
      double dx = frontier_goal.x - current_pose.position.x;
      double dy = frontier_goal.y - current_pose.position.y;
      double distance = std::sqrt(dx*dx + dy*dy);

      RCLCPP_INFO(logger_, "Robot aborted %.2f meters away from goal", distance);
      RCLCPP_INFO(logger_, "Robot aborted attempting to get to: {%.2f, %.2f}", frontier_goal.x, frontier_goal.y);
      if (distance > 0.5) {
        frontier_blacklist_.push_back(frontier_goal);
        RCLCPP_INFO(logger_, "Adding current goal to black list");
      }

      return;
    }

    // If cancelled just wait for the next planning loop. 
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_INFO(logger_, "Goal was canceled");
      return;
    default:
      RCLCPP_WARN(logger_, "Unknown result code from move base nav2");
      break;
  }
  
  makePlan();
}

/// @brief Publishes the blacklisted goals 
void Explore::publishBlacklist()
{
  // Construct array of poses
  geometry_msgs::msg::PoseArray msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = costmap_client_.getGlobalFrameID();

  for (const auto &p : frontier_blacklist_) {
    geometry_msgs::msg::Pose pose;
    pose.position = p;
    pose.orientation.w = 1.0;  // neutral orientation
    msg.poses.push_back(pose);
  }

  // Publish to topic
  explore_blacklist_publisher->publish(msg);
  RCLCPP_INFO(logger_, "Published blacklist with %zu poses.", frontier_blacklist_.size());
}

/// @brief Placeholder function for start-up
void Explore::start()
{
  RCLCPP_INFO(logger_, "Exploration started.");
}

/// @brief Cleanup function for the planning loop terminating
/// @param finished_exploring whether all available frontiers were explored
void Explore::stop(bool finished_exploring)
{
  // Cancel goals and timer
  RCLCPP_INFO(logger_, "Exploration stopped.");
  move_base_client_->async_cancel_all_goals();
  exploring_timer_->cancel();

  // Publish results based on whether all available frontiers were explored
  if (finished_exploring) {
    this->set_parameter(rclcpp::Parameter("node_status", "finished"));
    publishBlacklist();
  } else {
    this->set_parameter(rclcpp::Parameter("node_status", "stopped"));
  }
}

/// @brief Resets values when resuming
void Explore::resume()
{
  resuming_ = true;
  RCLCPP_INFO(logger_, "Exploration resuming.");
  // Reactivate the timer
  exploring_timer_->reset();
  // Resume immediately
  makePlan();
}

}  // namespace explore

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
      std::make_shared<explore::Explore>());  // std::move(std::make_unique)?
  rclcpp::shutdown();
  return 0;
}
