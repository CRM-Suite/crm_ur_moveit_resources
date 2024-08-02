#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>("planner", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  auto const logger = rclcpp::get_logger("planner");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]()
              { executor.spin(); })
      .detach();

  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "manipulator");

  move_group_interface.setPlanningPipelineId("pilz_industrial_motion_planner");
  move_group_interface.setPlannerId("LIN");

  move_group_interface.setMaxVelocityScalingFactor(0.1);
  move_group_interface.setMaxAccelerationScalingFactor(0.1);

  geometry_msgs::msg::PoseStamped start_pose = move_group_interface.getCurrentPose();
  RCLCPP_INFO(logger, "Current Pose Position    X %f", start_pose.pose.position.x);
  RCLCPP_INFO(logger, "Current Pose Position    Y %f", start_pose.pose.position.y);
  RCLCPP_INFO(logger, "Current Pose Position    Z %f", start_pose.pose.position.z);
  RCLCPP_INFO(logger, "Current Pose Orientation X %f", start_pose.pose.orientation.x);
  RCLCPP_INFO(logger, "Current Pose Orientation Y %f", start_pose.pose.orientation.y);
  RCLCPP_INFO(logger, "Current Pose Orientation Z %f", start_pose.pose.orientation.z);
  RCLCPP_INFO(logger, "Current Pose Orientation W %f", start_pose.pose.orientation.w);

  rclcpp::shutdown();
  return 0;
}