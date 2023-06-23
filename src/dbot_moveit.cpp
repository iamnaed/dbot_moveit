#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char ** argv)
{
  // Initialize node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "dbot_moveit_node", 
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Logger
  auto const logger = rclcpp::get_logger("dbot_moveit_node");

  // Spinup a thread for the MoveItVisualTools
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor](){executor.spin();});

  // Create MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "dbot_arm");

  // Construct and initialize MoveItVisualTools
  auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
    node,
    "base_link",
    rviz_visual_tools::RVIZ_MARKER_TOPIC,
    move_group_interface.getRobotModel()
  };
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();

  // Create closures for visualization
  auto const draw_title = [&moveit_visual_tools](auto text){
    auto const text_pose = []{
      auto msg = Eigen::Isometry3d::Identity();
      msg.translation().z() = 1.5;
      return msg;
    }();
    moveit_visual_tools.publishText(text_pose, text, rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  };
  auto const prompt = [&moveit_visual_tools](auto text){
    moveit_visual_tools.prompt(text);
  };
  auto const draw_trajectory_tool_path = [&moveit_visual_tools, jmg = move_group_interface.getRobotModel()->getJointModelGroup("dbot_arm")](auto trajectory){
    moveit_visual_tools.publishTrajectoryLine(trajectory, jmg);
  };  

  // Test
  auto default_planner_id = move_group_interface.getDefaultPlannerId("dbot_arm");
  RCLCPP_INFO(logger, "Default Planner ID: '%s'", default_planner_id.c_str());

  auto default_planning_pipeline_id = move_group_interface.getDefaultPlanningPipelineId();
  RCLCPP_INFO(logger, "Default Planning Pipeline ID: '%s'", default_planning_pipeline_id.c_str());

  auto planner_id = move_group_interface.getPlannerId();
  RCLCPP_INFO(logger, "Planner ID: '%s'", planner_id.c_str());

  auto planning_frame = move_group_interface.getPlanningFrame();
  RCLCPP_INFO(logger, "Planning Frame: '%s'", planning_frame.c_str());

  auto planning_pipeline_id = move_group_interface.getPlanningPipelineId();
  RCLCPP_INFO(logger, "Planning Pipeline ID: '%s'", planning_pipeline_id.c_str());

  auto planning_time = move_group_interface.getPlanningTime();
  RCLCPP_INFO(logger, "Planning Time: '%f's", planning_time);  

  moveit_msgs::msg::PlannerInterfaceDescription desc;
  move_group_interface.getInterfaceDescription(desc);
  RCLCPP_INFO(logger, "Interface Description loaded by action server: '%s'", desc.name.c_str());  

  std::vector<moveit_msgs::msg::PlannerInterfaceDescription> descs;
  move_group_interface.getInterfaceDescriptions(descs);
  RCLCPP_INFO(logger, "All Interface Description loaded by action server");
  for (auto &d : descs)
  {
      RCLCPP_INFO(logger, "\tInterface Description: '%s'", d.name.c_str());  
  }
  
  // Target
  auto const target_pose = []{
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = 1.0;
    msg.position.x = 0.28;
    msg.position.y = -0.2;
    msg.position.z = 0.5;
    return msg;
  }();
  move_group_interface.setPoseTarget(target_pose);

  // Create a plan to that target pose
  prompt("Press 'Next' in the RvizVisualToolsGui window to plan");
  draw_title("Planning");
  auto const [success, plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto const ok = static_cast<bool>(move_group_interface.plan(plan));
    return std::make_pair(ok, plan);
  }();

  // Execute the plan
  if(success)
  {
    draw_trajectory_tool_path(plan.trajectory_);
    moveit_visual_tools.trigger();
    prompt("Press 'Next' in the RvizVisualToolsGui window to execute");
    draw_title("Executing");
    moveit_visual_tools.trigger();
    move_group_interface.execute(plan);
  }
  else
  {
    draw_title("Planning Failed!");
    moveit_visual_tools.trigger();
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  // Shutdown
  rclcpp::shutdown();
  spinner.join();
  return 0;
}