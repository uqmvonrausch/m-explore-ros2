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
  this->declare_parameter<bool>("return_to_init", false);
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
  this->get_parameter("return_to_init", return_to_init_);
  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("recovery_delta", recovery_delta_);
  this->get_parameter("target_prox_lim", target_prox_lim_);
  this->get_parameter("goalpause_timeout", goalpause_timeout_);

  progress_timeout_ = timeout;
  move_base_client_ =
      rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
          this, ACTION_NAME);
        
  local_costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/local_costmap/costmap", 10,
    [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        local_costmap_ = msg;
    });

  global_costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/global_costmap/costmap", 10,
    [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        global_costmap_ = msg;
    });

  search_ = frontier_exploration::FrontierSearch(costmap_client_.getCostmap(),
                                                 potential_scale_, gain_scale_,
                                                 min_frontier_size, logger_);
                                          
  make_plan_service_ = this->create_service<std_srvs::srv::Empty>(
    "/make_plan",
    std::bind(&Explore::makePlanCallback, this, std::placeholders::_1, std::placeholders::_2));                                          

  if (visualize_) {
    marker_array_publisher_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>("explore/"
                                                                     "frontier"
                                                                     "s",
                                                                     10);
  }

  explore_status_publisher = 
    this->create_publisher<std_msgs::msg::String>("explore/status", 10);

  explore_blacklist_publisher =
    this->create_publisher<geometry_msgs::msg::PoseArray>("explore/blacklist", 10);

  
  status_timer = this->create_wall_timer(
      std::chrono::seconds(1),
      [this]() { statusCallback(); });
  
  goal_pose_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 10);

  // Subscription to resume or stop exploration
  resume_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
      "explore/resume", 10,
      std::bind(&Explore::resumeCallback, this, std::placeholders::_1));

  RCLCPP_INFO(logger_, "Waiting to connect to move_base nav2 server");
  move_base_client_->wait_for_action_server();
  RCLCPP_INFO(logger_, "Connected to move_base nav2 server");

  if (return_to_init_) {
    RCLCPP_INFO(logger_, "Getting initial pose of the robot");
    geometry_msgs::msg::TransformStamped transformStamped;
    std::string map_frame = costmap_client_.getGlobalFrameID();
    try {
      transformStamped = tf_buffer_.lookupTransform(
          map_frame, robot_base_frame_, tf2::TimePointZero);
      initial_pose_.position.x = transformStamped.transform.translation.x;
      initial_pose_.position.y = transformStamped.transform.translation.y;
      initial_pose_.orientation = transformStamped.transform.rotation;
    } catch (tf2::TransformException& ex) {
      RCLCPP_ERROR(logger_, "Couldn't find transform from %s to %s: %s",
                   map_frame.c_str(), robot_base_frame_.c_str(), ex.what());
      return_to_init_ = false;
    }
  }

  exploring_timer_ = this->create_wall_timer(
      std::chrono::milliseconds((uint16_t)(1000.0 / planner_frequency_)),
      [this]() { makePlan(); });
  // Start exploration right away
  this->set_parameter(rclcpp::Parameter("node_status", "exploring"));
  makePlan();
}

Explore::~Explore()
{
  stop();
}

void Explore::statusCallback() 
{
  std::string status = this->get_parameter("node_status").as_string();

  auto msg = std_msgs::msg::String();
  msg.data = status;
  explore_status_publisher->publish(msg);
}

void Explore::makePlanCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/)
{
  statusCallback();
  last_progress_ = this->now();
  this->set_parameter(rclcpp::Parameter("node_status", "exploring"));
  RCLCPP_INFO(this->get_logger(), "makePlan service called, triggering replanning.");
  exploring_timer_->reset();
  frontier_blacklist_.clear();
  first_pass_ = true;
  this->makePlan();
}

void Explore::resumeCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data) {
    resume();
  } else {
    stop();
  }
}

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
    m.type = visualization_msgs::msg::Marker::POINTS;
    m.id = int(id);
    // m.pose.position = {}; // compile warning
    m.scale.x = 0.1;
    m.scale.y = 0.1;
    m.scale.z = 0.1;
    m.points = frontier.points;
    if (goalOnBlacklist(frontier.centroid)) {
      m.color = red;
    } else {
      m.color = blue;
    }
    markers.push_back(m);
    ++id;
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

void Explore::makePlan()
{

  if (goal_active_) {
    // Robot is already pursuing a goal, don't send a new one
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
  
  RCLCPP_INFO(logger_, "found %lu frontiers", frontiers.size());
  for (size_t i = 0; i < frontiers.size(); ++i) {
    RCLCPP_DEBUG(logger_, "frontier %zd cost: %f", i, frontiers[i].cost);
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
  geometry_msgs::msg::Point target_position = frontier->centroid;
  geometry_msgs::msg::Point target_centroid = target_position;

  // check whether proposed target is at the robot position
  bool null_target =
    (std::abs(target_position.x - pose.position.x) < target_prox_lim_) &&
    (std::abs(target_position.y - pose.position.y) < target_prox_lim_);

  if (null_target) {
    // RCLCPP_INFO(logger_, "Frontier target set to current robot position: Pausing for map update cycle (5s)");
    // std::this_thread::sleep_for(std::chrono::seconds(5));
    RCLCPP_INFO(logger_, "Frontier target set to current robot position: projecting goal along frontier direction");
    
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
  RCLCPP_INFO(logger_, "Local costmap val at target is: %d.", local_costmap_val);
  RCLCPP_INFO(logger_, "Global costmap val at target is: %d.", global_costmap_val);

  int max_costmap_target_val = 80;
  int max_costmap_val = std::max<int>(local_costmap_val, global_costmap_val);
  
  // If target position is too close to a wall, attempt to move it to a lower cost position
  if (max_costmap_val >= max_costmap_target_val) {

    // Search parameters
    double search_radius = 8;
    double search_resolution = 0.05;

    bool found = false;
    for (int dx = -search_radius; dx <= search_radius && !found; ++dx) {
      for (int dy = -search_radius; dy <= search_radius && !found; ++dy) {

        // Calculate candidate position and find max costmap value
        double test_x = target_position.x + static_cast<double>(dx) * search_resolution;
        double test_y = target_position.y + static_cast<double>(dy) * search_resolution;

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
  if (first_pass_ || !same_centroid || (same_goal && prev_distance_ > frontier->min_distance)) {
    // we have different goal or we made some progress
    last_progress_ = this->now();
    prev_distance_ = frontier->min_distance;
  }
  
  // black list if we've made no progress for a long time
  if ((this->now() - last_progress_ >
      tf2::durationFromSec(progress_timeout_)) && !resuming_) {
    frontier_blacklist_.push_back(target_centroid);
    RCLCPP_INFO(logger_, "Timeout: Adding current goal to black list");
    makePlan();
    return;
  }

  // Ensure only first call of makePlan was set resuming to true
  if (resuming_) {
    resuming_ = false;
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

  last_goal_ = this->now();
  auto send_goal_options =
      rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();

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

// void Explore::returnToInitialPose()
// {
//   RCLCPP_INFO(logger_, "Returning to initial pose.");
//   auto goal = nav2_msgs::action::NavigateToPose::Goal();
//   goal.pose.pose.position = initial_pose_.position;
//   goal.pose.pose.orientation = initial_pose_.orientation;
//   goal.pose.header.frame_id = costmap_client_.getGlobalFrameID();
//   goal.pose.header.stamp = this->now();

//   auto send_goal_options =
//       rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
//   move_base_client_->async_send_goal(goal, send_goal_options);
// }

// bool Explore::goalOnBlacklist(const geometry_msgs::msg::Point& goal)
// {
//   // constexpr static size_t tolerance = 2;
//   // nav2_costmap_2d::Costmap2D* costmap2d = costmap_client_.getCostmap();

//   // check if a goal is on the blacklist for goals that we're pursuing
//   for (auto& frontier_goal : frontier_blacklist_) {
//     double x_diff = fabs(goal.x - frontier_goal.x);
//     double y_diff = fabs(goal.y - frontier_goal.y);

//     if (sqrt(x_diff * x_diff + y_diff * y_diff) < 0.35) {
//       return true;
//     }
//   }
//   return false;
// }

void Explore::reachedGoal(const NavigationGoalHandle::WrappedResult& result,
                          const geometry_msgs::msg::Point& frontier_goal)
{
  goal_active_ = false; 
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_DEBUG(logger_, "Goal was successful. Pausing for 2s");
      std::this_thread::sleep_for(std::chrono::seconds(2));
      break;
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
      
      // If it was aborted probably because we've found another frontier goal,
      // so just return and don't make plan again
      return;
    }
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_INFO(logger_, "Goal was canceled");
      // If goal canceled might be because exploration stopped from topic. Don't make new plan.
      return;
    default:
      RCLCPP_WARN(logger_, "Unknown result code from move base nav2");
      break;
  }
  // find new goal immediately regardless of planning frequency.
  // execute via timer to prevent dead lock in move_base_client (this is
  // callback for sendGoal, which is called in makePlan). the timer must live
  // until callback is executed.
  // oneshot_ = relative_nh_.createTimer(
  //     ros::Duration(0, 0), [this](const ros::TimerEvent&) { makePlan(); },
  //     true);

  // Because of the 1-thread-executor nature of ros2 I think timer is not
  // needed.

  makePlan();
}

void Explore::publishBlacklist()
{
  geometry_msgs::msg::PoseArray msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = costmap_client_.getGlobalFrameID();

  for (const auto &p : frontier_blacklist_) {
    geometry_msgs::msg::Pose pose;
    pose.position = p;
    pose.orientation.w = 1.0;  // neutral orientation
    msg.poses.push_back(pose);
  }

  explore_blacklist_publisher->publish(msg);
  RCLCPP_INFO(logger_, "Published blacklist with %zu poses.", frontier_blacklist_.size());
}

void Explore::start()
{
  RCLCPP_INFO(logger_, "Exploration started.");
}

void Explore::stop(bool finished_exploring)
{
  RCLCPP_INFO(logger_, "Exploration stopped.");
  move_base_client_->async_cancel_all_goals();
  exploring_timer_->cancel();

  if (finished_exploring) {
    this->set_parameter(rclcpp::Parameter("node_status", "finished"));
    publishBlacklist();
  } else {
    this->set_parameter(rclcpp::Parameter("node_status", "stopped"));
  }
}

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
