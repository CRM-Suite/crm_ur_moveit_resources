#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

class PlannerNode : public rclcpp::Node
{
public:
    PlannerNode() : Node("planner")
    {
        // The node is constructed, but we don't do anything else here
    }

    void run()
    {
        // This method will be called after construction, when it's safe to use shared_from_this()
        move_group_interface_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "manipulator");
        
        move_group_interface_->setPlanningPipelineId("pilz_industrial_motion_planner");
        move_group_interface_->setPlannerId("LIN");
        move_group_interface_->setMaxVelocityScalingFactor(0.1);
        move_group_interface_->setMaxAccelerationScalingFactor(0.1);
        
        planAndExecute();
    }

private:
    void planAndExecute()
    {
        geometry_msgs::msg::Pose target_pose = move_group_interface_->getCurrentPose().pose;
        target_pose.position.z -= 0.01; // Move 1 cm in the negative Z direction
        move_group_interface_->setPoseTarget(target_pose);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = static_cast<bool>(move_group_interface_->plan(plan));

        if (success)
        {
            move_group_interface_->execute(plan);
            RCLCPP_INFO(this->get_logger(), "Planning and execution successful");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Planning failed!");
        }
    }

    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<PlannerNode>();
    
    // Create a single-threaded executor
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    // Spin the executor in a separate thread
    std::thread executor_thread([&executor]() { executor.spin(); });

    // Run the planning and execution
    node->run();

    // Stop the executor and join the thread
    executor.cancel();
    executor_thread.join();

    rclcpp::shutdown();
    return 0;
}