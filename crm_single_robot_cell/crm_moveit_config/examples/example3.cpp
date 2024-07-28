#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

class PlannerNode : public rclcpp::Node
{
public:
    PlannerNode() : Node("planner") {}

    void run()
    {
        // Plan and execute for right manipulator
        planAndExecuteForManipulator("alice_manipulator");

        // Then plan and execute for left manipulator
        planAndExecuteForManipulator("bob_manipulator");
    }

private:
    void planAndExecuteForManipulator(const std::string& manipulator_name)
    {
        RCLCPP_INFO(this->get_logger(), "Planning for %s", manipulator_name.c_str());

        auto move_group_interface = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), manipulator_name);

        move_group_interface->setPlanningPipelineId("pilz_industrial_motion_planner");
        move_group_interface->setPlannerId("LIN");
        move_group_interface->setMaxVelocityScalingFactor(0.1);
        move_group_interface->setMaxAccelerationScalingFactor(0.1);

        geometry_msgs::msg::Pose target_pose = move_group_interface->getCurrentPose().pose;
        target_pose.position.z -= 0.01; // Move 1 cm in the negative Z direction
        move_group_interface->setPoseTarget(target_pose);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = static_cast<bool>(move_group_interface->plan(plan));

        if (success)
        {
            move_group_interface->execute(plan);
            RCLCPP_INFO(this->get_logger(), "Planning and execution successful for %s", manipulator_name.c_str());
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Planning failed for %s", manipulator_name.c_str());
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PlannerNode>();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    std::thread executor_thread([&executor]() { executor.spin(); });

    node->run();

    executor.cancel();
    executor_thread.join();

    rclcpp::shutdown();
    return 0;
}