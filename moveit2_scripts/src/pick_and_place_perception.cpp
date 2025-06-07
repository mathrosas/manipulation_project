#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <custom_msgs/msg/detected_objects.hpp>

#include <chrono>
#include <cmath>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <vector>

// program variables
static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_node");
static const std::string PLANNING_GROUP_ROBOT = "ur_manipulator";
static const std::string PLANNING_GROUP_GRIPPER = "gripper";

class PickAndPlaceTrajectory {
public:
  PickAndPlaceTrajectory(rclcpp::Node::SharedPtr base_node_)
      : base_node_(base_node_), object_received_(false), object_x_(0.0),
        object_y_(0.0), object_z_(0.0) {
    RCLCPP_INFO(LOGGER, "Initializing Class: Pick And Place Trajectory…");

    // 1) Create subscription on base_node_, not this‐>create_subscription
    object_sub_ =
        base_node_->create_subscription<custom_msgs::msg::DetectedObjects>(
            "/object_detected", rclcpp::QoS(10),
            std::bind(&PickAndPlaceTrajectory::objectCallback, this,
                      std::placeholders::_1));

    // configure node options
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    // initialize move_group node as a separate node
    move_group_node_ =
        rclcpp::Node::make_shared("move_group_node", node_options);
    executor_.add_node(move_group_node_);
    std::thread([this]() { executor_.spin(); }).detach();

    // initialize MoveGroupInterface objects
    move_group_robot_ = std::make_shared<MoveGroupInterface>(
        move_group_node_, PLANNING_GROUP_ROBOT);
    move_group_gripper_ = std::make_shared<MoveGroupInterface>(
        move_group_node_, PLANNING_GROUP_GRIPPER);

    // get initial state of robot and gripper
    joint_model_group_robot_ =
        move_group_robot_->getCurrentState()->getJointModelGroup(
            PLANNING_GROUP_ROBOT);
    joint_model_group_gripper_ =
        move_group_gripper_->getCurrentState()->getJointModelGroup(
            PLANNING_GROUP_GRIPPER);

    // print out basic system information
    RCLCPP_INFO(LOGGER, "Planning Frame: %s",
                move_group_robot_->getPlanningFrame().c_str());
    RCLCPP_INFO(LOGGER, "End Effector Link: %s",
                move_group_robot_->getEndEffectorLink().c_str());
    RCLCPP_INFO(LOGGER, "Available Planning Groups:");
    std::vector<std::string> group_names =
        move_group_robot_->getJointModelGroupNames();
    for (size_t i = 0; i < group_names.size(); ++i) {
      RCLCPP_INFO(LOGGER, "  Group %zu: %s", i, group_names[i].c_str());
    }

    // grab current state into our vectors
    current_state_robot_ = move_group_robot_->getCurrentState(10);
    current_state_robot_->copyJointGroupPositions(joint_model_group_robot_,
                                                  joint_group_positions_robot_);

    current_state_gripper_ = move_group_gripper_->getCurrentState(10);
    current_state_gripper_->copyJointGroupPositions(
        joint_model_group_gripper_, joint_group_positions_gripper_);

    // set start state to current
    move_group_robot_->setStartStateToCurrentState();
    move_group_gripper_->setStartStateToCurrentState();

    RCLCPP_INFO(LOGGER, "Class Initialized: Pick And Place Trajectory");
  }

  ~PickAndPlaceTrajectory() {
    RCLCPP_INFO(LOGGER, "Class Terminated: Pick And Place Trajectory");
  }

  void run() {
    // Wait up to 10 seconds for the first /object_detected message
    RCLCPP_INFO(LOGGER, "Waiting for a block detection on /object_detected …");
    auto start = base_node_->now();
    rclcpp::Rate rate(10);
    while (rclcpp::ok() && !object_received_) {
      if ((base_node_->now() - start).seconds() > 10.0) {
        RCLCPP_ERROR(LOGGER, "Timeout: no /object_detected received!");
        return;
      }
      rclcpp::spin_some(base_node_->get_node_base_interface());
      rate.sleep();
    }

    RCLCPP_INFO(
        LOGGER,
        "Block detected at [%.3f, %.3f, %.3f]. Proceeding with pick‐and‐place.",
        object_x_, object_y_, object_z_);

    // Now that we have the block’s (x,y,z), execute the full trajectory
    execute_trajectory_plan();
  }

  void objectCallback(const custom_msgs::msg::DetectedObjects::SharedPtr msg) {
    // only take the very first /object_detected message
    if (!object_received_) {
      object_x_ = msg->position.x;
      object_y_ = msg->position.y;
      object_z_ = msg->position.z;
      object_received_ = true;
      RCLCPP_INFO(LOGGER,
                  "Received /object_detected → x: %.3f, y: %.3f, z: %.3f",
                  object_x_, object_y_, object_z_);
    }
  }

  void execute_trajectory_plan() {
    RCLCPP_INFO(LOGGER, "Planning and Executing Pick & Place Trajectory…");

    // 1) Go to Home
    RCLCPP_INFO(LOGGER, "Going to Home Position…");
    setup_joint_value_target(0.0000, -2.5000, +1.5000, -1.5000, -1.5000,
                             0.0000);
    plan_trajectory_kinematics();
    execute_trajectory_kinematics();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // 2) Pre-grasp: use object_x_, object_y_, object_z_
    RCLCPP_INFO(LOGGER, "Going to Pregrasp Position…");
    setup_goal_pose_target(object_x_ + 0.012, object_y_ - 0.012,
                           object_z_ + 0.20, -1.000, 0.000, 0.000, 0.000);
    plan_trajectory_kinematics();
    execute_trajectory_kinematics();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // 3) Open Gripper
    RCLCPP_INFO(LOGGER, "Opening Gripper…");
    setup_named_pose_gripper("open");
    plan_trajectory_gripper();
    execute_trajectory_gripper();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // 4) Approach down 6 cm
    RCLCPP_INFO(LOGGER, "Approaching…");
    setup_waypoints_target(0.000, 0.000, -0.060);
    plan_trajectory_cartesian();
    execute_trajectory_cartesian();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // 5) Close Gripper
    RCLCPP_INFO(LOGGER, "Closing Gripper…");
    setup_named_pose_gripper("close");
    plan_trajectory_gripper();
    execute_trajectory_gripper();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // 6) Retreat up 6 cm
    RCLCPP_INFO(LOGGER, "Retreating…");
    setup_waypoints_target(0.000, 0.000, +0.100);
    plan_trajectory_cartesian();
    execute_trajectory_cartesian();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // 7) Go to Place position (hard‐coded)
    RCLCPP_INFO(LOGGER, "Going to Place Position…");
    RCLCPP_INFO(LOGGER, "Preparing Joint Value Trajectory...");
    setup_joint_value_target(3.0000, -1.373942, 1.536554, -1.732970, -1.571293,
                             -1.568870);
    // plan and execute the trajectory
    RCLCPP_INFO(LOGGER, "Planning Joint Value Trajectory...");
    plan_trajectory_kinematics();
    RCLCPP_INFO(LOGGER, "Executing Joint Value Trajectory...");
    execute_trajectory_kinematics();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // 8) Open gripper to drop
    RCLCPP_INFO(LOGGER, "Opening Gripper…");
    setup_named_pose_gripper("open");
    plan_trajectory_gripper();
    execute_trajectory_gripper();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // 9) Go home again
    RCLCPP_INFO(LOGGER, "Going to Home Position…");
    setup_joint_value_target(0.0000, -2.5000, +1.5000, -1.5000, -1.5000,
                             0.0000);
    plan_trajectory_kinematics();
    execute_trajectory_kinematics();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    RCLCPP_INFO(LOGGER, "Pick & Place Trajectory Execution Complete");
  }

private:
  // shorthand typedefs
  using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
  using JointModelGroup = moveit::core::JointModelGroup;
  using RobotStatePtr = moveit::core::RobotStatePtr;
  using Plan = MoveGroupInterface::Plan;
  using Pose = geometry_msgs::msg::Pose;
  using RobotTrajectory = moveit_msgs::msg::RobotTrajectory;

  // user‐provided Node handle
  rclcpp::Node::SharedPtr base_node_;

  // move_group node and executor
  rclcpp::Node::SharedPtr move_group_node_;
  rclcpp::executors::SingleThreadedExecutor executor_;

  // MoveGroupInterfaces
  std::shared_ptr<MoveGroupInterface> move_group_robot_;
  std::shared_ptr<MoveGroupInterface> move_group_gripper_;

  // Subscription to /object_detected
  rclcpp::Subscription<custom_msgs::msg::DetectedObjects>::SharedPtr
      object_sub_;

  // Joint‐group pointers
  const JointModelGroup *joint_model_group_robot_;
  const JointModelGroup *joint_model_group_gripper_;

  // Stored joint‐values
  std::vector<double> joint_group_positions_robot_{6, 0.0};
  std::vector<double> joint_group_positions_gripper_{3, 0.0};

  // Current robot states
  RobotStatePtr current_state_robot_;
  RobotStatePtr current_state_gripper_;

  // Trajectory planning variables
  Plan kinematics_trajectory_plan_;
  bool plan_success_robot_{false};
  Plan gripper_trajectory_plan_;
  bool plan_success_gripper_{false};
  std::vector<Pose> cartesian_waypoints_;
  RobotTrajectory cartesian_trajectory_plan_;
  const double jump_threshold_{0.0};
  const double end_effector_step_{0.01};
  double plan_fraction_robot_{0.0};

  // Object detection storage
  bool object_received_;
  double object_x_, object_y_, object_z_;

  // ─── Helper methods
  // ─────────────────────────────────────────────────────────

  void setup_joint_value_target(float a0, float a1, float a2, float a3,
                                float a4, float a5) {
    joint_group_positions_robot_[0] = a0;
    joint_group_positions_robot_[1] = a1;
    joint_group_positions_robot_[2] = a2;
    joint_group_positions_robot_[3] = a3;
    joint_group_positions_robot_[4] = a4;
    joint_group_positions_robot_[5] = a5;
    move_group_robot_->setJointValueTarget(joint_group_positions_robot_);
  }

  void setup_goal_pose_target(float px, float py, float pz, float qx, float qy,
                              float qz, float qw) {
    Pose target;
    target.position.x = px;
    target.position.y = py;
    target.position.z = pz;
    target.orientation.x = qx;
    target.orientation.y = qy;
    target.orientation.z = qz;
    target.orientation.w = qw;
    move_group_robot_->setPoseTarget(target);
  }

  void plan_trajectory_kinematics() {
    plan_success_robot_ =
        (move_group_robot_->plan(kinematics_trajectory_plan_) ==
         moveit::core::MoveItErrorCode::SUCCESS);
  }

  void execute_trajectory_kinematics() {
    if (plan_success_robot_) {
      move_group_robot_->execute(kinematics_trajectory_plan_);
      RCLCPP_INFO(LOGGER, "Robot Kinematics Trajectory Success!");
    } else {
      RCLCPP_WARN(LOGGER, "Robot Kinematics Trajectory Failed!");
    }
  }

  void setup_waypoints_target(float dx, float dy, float dz) {
    Pose current = move_group_robot_->getCurrentPose().pose;
    cartesian_waypoints_.clear();
    cartesian_waypoints_.push_back(current);
    current.position.x += dx;
    current.position.y += dy;
    current.position.z += dz;
    cartesian_waypoints_.push_back(current);
  }

  void plan_trajectory_cartesian() {
    plan_fraction_robot_ = move_group_robot_->computeCartesianPath(
        cartesian_waypoints_, end_effector_step_, jump_threshold_,
        cartesian_trajectory_plan_);
  }

  void execute_trajectory_cartesian() {
    if (plan_fraction_robot_ >= 0.0) {
      move_group_robot_->execute(cartesian_trajectory_plan_);
      RCLCPP_INFO(LOGGER, "Robot Cartesian Trajectory Success!");
    } else {
      RCLCPP_WARN(LOGGER, "Robot Cartesian Trajectory Failed!");
    }
    cartesian_waypoints_.clear();
  }

  void setup_named_pose_gripper(const std::string &pose_name) {
    move_group_gripper_->setNamedTarget(pose_name);
  }

  void plan_trajectory_gripper() {
    plan_success_gripper_ =
        (move_group_gripper_->plan(gripper_trajectory_plan_) ==
         moveit::core::MoveItErrorCode::SUCCESS);
  }

  void execute_trajectory_gripper() {
    if (plan_success_gripper_) {
      move_group_gripper_->execute(gripper_trajectory_plan_);
      RCLCPP_INFO(LOGGER, "Gripper Action Command Success!");
    } else {
      RCLCPP_WARN(LOGGER, "Gripper Action Command Failed!");
    }
  }

}; // end class PickAndPlaceTrajectory

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  // Create a “base node” that we will use only to spin and accept callbacks
  auto base_node =
      std::make_shared<rclcpp::Node>("pick_and_place_perception_node");

  PickAndPlaceTrajectory pick_and_place(base_node);

  // Wait for /object_detected and then run the trajectory
  pick_and_place.run();

  rclcpp::shutdown();
  return 0;
}
