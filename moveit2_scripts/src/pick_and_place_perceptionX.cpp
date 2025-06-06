#include <custom_msgs/msg/detected_objects.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <cmath>
#include <memory>
#include <thread>
#include <vector>

static const rclcpp::Logger LOGGER =
    rclcpp::get_logger("pick_and_place_perception");
static const std::string PLANNING_GROUP_ROBOT = "ur_manipulator";
static const std::string PLANNING_GROUP_GRIPPER = "gripper";

class PickAndPlaceTrajectory : public rclcpp::Node {
public:
  PickAndPlaceTrajectory()
      : Node("pick_and_place_perception"), object_received_(false),
        object_x_(0.0), object_y_(0.0), object_z_(0.0) {
    RCLCPP_INFO(LOGGER, "Initializing PickAndPlaceTrajectory node...");

    // 1) Subscription to /object_detected
    object_sub_ = this->create_subscription<custom_msgs::msg::DetectedObjects>(
        "/object_detected", 10,
        std::bind(&PickAndPlaceTrajectory::objectCallback, this,
                  std::placeholders::_1));

    // 2) Spawn a separate MoveGroupInterface node and spin it on its own thread
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    move_group_node_ = rclcpp::Node::make_shared("move_group_node", options);

    executor_.add_node(move_group_node_);
    std::thread([this]() { executor_.spin(); }).detach();

    // 3) Initialize MoveGroupInterfaces
    move_group_robot_ =
        std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            move_group_node_, PLANNING_GROUP_ROBOT);
    move_group_gripper_ =
        std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            move_group_node_, PLANNING_GROUP_GRIPPER);

    // 4) Cache joint‐model groups
    joint_model_group_robot_ =
        move_group_robot_->getCurrentState()->getJointModelGroup(
            PLANNING_GROUP_ROBOT);
    joint_model_group_gripper_ =
        move_group_gripper_->getCurrentState()->getJointModelGroup(
            PLANNING_GROUP_GRIPPER);

    // 5) Cache current joint positions (robot + gripper)
    auto state_robot = move_group_robot_->getCurrentState(10);
    state_robot->copyJointGroupPositions(joint_model_group_robot_,
                                         joint_group_positions_robot_);
    auto state_gripper = move_group_gripper_->getCurrentState(10);
    state_gripper->copyJointGroupPositions(joint_model_group_gripper_,
                                           joint_group_positions_gripper_);

    // 6) Set start state to “current” for both planning groups
    move_group_robot_->setStartStateToCurrentState();
    move_group_gripper_->setStartStateToCurrentState();

    RCLCPP_INFO(LOGGER, "PickAndPlaceTrajectory initialized.");
  }

  ~PickAndPlaceTrajectory() {
    RCLCPP_INFO(LOGGER, "Shutting down PickAndPlaceTrajectory node.");
  }

  void run() {
    // Wait up to 10 seconds for the first /object_detected message
    RCLCPP_INFO(LOGGER,
                "Waiting for a block detection on /object_detected ...");
    auto start = this->now();
    rclcpp::Rate rate(10);

    while (rclcpp::ok() && !object_received_) {
      if ((this->now() - start).seconds() > 10.0) {
        RCLCPP_ERROR(LOGGER, "Timeout: no /object_detected received!");
        return;
      }
      rclcpp::spin_some(this->get_node_base_interface());
      rate.sleep();
    }

    RCLCPP_INFO(
        LOGGER,
        "Block detected at [%.3f, %.3f, %.3f]. Proceeding with pick-and-place.",
        object_x_, object_y_, object_z_);

    // Now that we have (x,y,z), execute the full pick‐and‐place
    executeTrajectoryPlan();
  }

private:
  //───────────────────────────────────────────────────────────────────────────
  // 1) Callback for /object_detected
  //───────────────────────────────────────────────────────────────────────────
  void objectCallback(const custom_msgs::msg::DetectedObjects::SharedPtr msg) {
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

  //───────────────────────────────────────────────────────────────────────────
  // 2) Full pick‐and‐place sequence (using the detected (x,y,z))
  //───────────────────────────────────────────────────────────────────────────
  void executeTrajectoryPlan() {
    RCLCPP_INFO(LOGGER, "Planning and Executing Pick & Place Trajectory...");

    // A) Go to “Home” joint position
    RCLCPP_INFO(LOGGER, "→ Moving to HOME joint pose...");
    setJointTarget(0.0000, -2.5000, 1.5000, -1.5000, -1.5000, 0.0000);
    planAndExecuteRobot();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // B) Move to Pregrasp (hover 5 cm above detected block)
    RCLCPP_INFO(LOGGER, "→ Moving to PREGRASP above the block...");
    auto current_pose = move_group_robot_->getCurrentPose().pose;
    current_pose.position.x = object_x_;
    current_pose.position.y = object_y_;
    current_pose.position.z = object_z_ + 0.05; // 5 cm above
    move_group_robot_->setPoseTarget(current_pose);
    planAndExecuteRobot();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // C) Open the gripper
    RCLCPP_INFO(LOGGER, "→ Opening gripper...");
    move_group_gripper_->setNamedTarget("open");
    planAndExecuteGripper();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // D) Approach straight down 6 cm
    RCLCPP_INFO(LOGGER, "→ Approaching DOWN to block...");
    {
      std::vector<geometry_msgs::msg::Pose> waypoints;
      auto wpose = move_group_robot_->getCurrentPose().pose;
      waypoints.push_back(wpose);
      wpose.position.z -= 0.06; // 6 cm down
      waypoints.push_back(wpose);
      move_group_robot_->computeCartesianPath(waypoints, 0.01, 0.0,
                                              cartesian_plan_);
      move_group_robot_->execute(cartesian_plan_);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // E) Close the gripper (to pick)
    RCLCPP_INFO(LOGGER, "→ Closing gripper to pick...");
    move_group_gripper_->setNamedTarget("close");
    planAndExecuteGripper();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // F) Retract straight up 6 cm
    RCLCPP_INFO(LOGGER, "→ Retracting UP with block...");
    {
      std::vector<geometry_msgs::msg::Pose> waypoints;
      auto wpose = move_group_robot_->getCurrentPose().pose;
      waypoints.push_back(wpose);
      wpose.position.z += 0.06; // 6 cm up
      waypoints.push_back(wpose);
      move_group_robot_->computeCartesianPath(waypoints, 0.01, 0.0,
                                              cartesian_plan_);
      move_group_robot_->execute(cartesian_plan_);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // G) Move to Place joint pose (hardcoded)
    RCLCPP_INFO(LOGGER, "→ Moving to PLACE joint pose...");
    setJointTarget(0.0000, -1.6956, -1.7875, -1.2292, 1.5702, -1.2365);
    planAndExecuteRobot();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // H) Open the gripper to release the block
    RCLCPP_INFO(LOGGER, "→ Opening gripper to RELEASE...");
    move_group_gripper_->setNamedTarget("open");
    planAndExecuteGripper();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // I) Return to Home
    RCLCPP_INFO(LOGGER, "→ Returning to HOME...");
    setJointTarget(0.0000, -2.5000, 1.5000, -1.5000, -1.5000, 0.0000);
    planAndExecuteRobot();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    RCLCPP_INFO(LOGGER, "Pick & Place with Perception: COMPLETE.");
  }

  //───────────────────────────────────────────────────────────────────────────
  // 3) Planning/execution helpers
  //───────────────────────────────────────────────────────────────────────────
  void setJointTarget(double a0, double a1, double a2, double a3, double a4,
                      double a5) {
    joint_group_positions_robot_[0] = a0;
    joint_group_positions_robot_[1] = a1;
    joint_group_positions_robot_[2] = a2;
    joint_group_positions_robot_[3] = a3;
    joint_group_positions_robot_[4] = a4;
    joint_group_positions_robot_[5] = a5;
    move_group_robot_->setJointValueTarget(joint_group_positions_robot_);
  }

  void planAndExecuteRobot() {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_robot_->plan(plan) ==
                    moveit::core::MoveItErrorCode::SUCCESS);
    if (!success) {
      RCLCPP_ERROR(LOGGER, "Robot planning failed!");
    }
    move_group_robot_->execute(plan);
  }

  void planAndExecuteGripper() {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_gripper_->plan(plan) ==
                    moveit::core::MoveItErrorCode::SUCCESS);
    if (!success) {
      RCLCPP_ERROR(LOGGER, "Gripper planning failed!");
    }
    move_group_gripper_->execute(plan);
  }

private:
  // (A) Subscription to /object_detected
  rclcpp::Subscription<custom_msgs::msg::DetectedObjects>::SharedPtr
      object_sub_;
  bool object_received_;
  double object_x_, object_y_, object_z_;

  // (B) MoveIt “inner” node & executor
  rclcpp::Node::SharedPtr move_group_node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface>
      move_group_robot_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface>
      move_group_gripper_;
  const moveit::core::JointModelGroup *joint_model_group_robot_;
  const moveit::core::JointModelGroup *joint_model_group_gripper_;
  std::vector<double> joint_group_positions_robot_{6, 0.0};
  std::vector<double> joint_group_positions_gripper_{3, 0.0};

  // (C) Cartesian planning storage
  moveit_msgs::msg::RobotTrajectory cartesian_plan_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PickAndPlaceTrajectory>();

  // Give ROS a brief moment to start everything
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  node->run();

  rclcpp::shutdown();
  return 0;
}
