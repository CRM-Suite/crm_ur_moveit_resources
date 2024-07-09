#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

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
    auto move_group_interface = MoveGroupInterface(node, "right_manipulator");

    move_group_interface.setPlanningPipelineId("pilz_industrial_motion_planner");
    move_group_interface.setPlannerId("LIN");

    move_group_interface.setMaxVelocityScalingFactor(0.1);
    move_group_interface.setMaxAccelerationScalingFactor(0.1);

    geometry_msgs::msg::Pose target_pose = move_group_interface.getCurrentPose().pose;
    target_pose.position.x += .1; // Move 50 cm in the positive Z direction
    target_pose.orientation.w = -.258819; 
    target_pose.orientation.x = .9659258; 
        
    // // Set orientation to a 30-degree angle off Z-axis towards Y-axis
    // double angle = M_PI / 6.0; // 30 degrees in radians

    // // Create a quaternion from the axis-angle representation
    // tf2::Quaternion orientation_quat;
    // orientation_quat.setRPY(0, angle, 0); // Roll: 0, Pitch: angle, Yaw: 0

    // // Convert tf2::Quaternion to geometry_msgs::msg::Quaternion
    // geometry_msgs::msg::Quaternion orientation_msg;
    // tf2::convert(orientation_quat, orientation_msg);

    // // Set the orientation of target_pose
    // target_pose.orientation = orientation_msg;
    
    // RCLCPP_INFO(logger, "Target Pose Orientation: [x=%f, y=%f, z=%f, w=%f]",
    //         target_pose.orientation.x,
    //         target_pose.orientation.y,
    //         target_pose.orientation.z,
    //         target_pose.orientation.w);

    move_group_interface.setPoseTarget(target_pose);
    auto const [success, plan] = [&move_group_interface]()
    {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok, msg);
    }();

    if (success)
    {
        move_group_interface.execute(plan);
    }
    else
    {
        RCLCPP_ERROR(logger, "Planning failed!");
    }

    rclcpp::shutdown();
    return 0;
}
