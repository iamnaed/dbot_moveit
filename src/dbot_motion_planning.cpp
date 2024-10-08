#include <pluginlib/class_loader.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/planning_scene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/move_group_interface/move_group_interface.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("dbot_motion_planning_node");

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    std::shared_ptr<rclcpp::Node> motion_planning_api_tutorial_node =
        rclcpp::Node::make_shared("dbot_motion_planning_node", node_options);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(motion_planning_api_tutorial_node);
    std::thread([&executor]() { executor.spin(); }).detach();

    // Setup Planners
    const std::string PLANNING_GROUP = "dbot_arm";
    robot_model_loader::RobotModelLoader robot_model_loader(motion_planning_api_tutorial_node, "robot_description");
    const moveit::core::RobotModelPtr& robot_model = robot_model_loader.getModel();
    moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(robot_model));
    const moveit::core::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);

    // Using the
    // :moveit_codedir:`RobotModel<moveit_core/robot_model/include/moveit/robot_model/robot_model.h>`,
    // we can construct a
    // :moveit_codedir:`PlanningScene<moveit_core/planning_scene/include/moveit/planning_scene/planning_scene.h>`
    // that maintains the state of the world (including the robot).
    planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));
    planning_scene->getCurrentStateNonConst().setToDefaultValues(joint_model_group, "zero");

    // We will now construct a loader to load a planner, by name.
    // Note that we are using the ROS pluginlib library here.
    std::unique_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
    planning_interface::PlannerManagerPtr planner_instance;
    std::string planner_plugin_name;

    // We will get the name of planning plugin we want to load
    // from the ROS parameter server, and then load the planner
    // making sure to catch all exceptions.
    if(!motion_planning_api_tutorial_node->get_parameter("ompl.planning_plugin", planner_plugin_name))
        RCLCPP_FATAL(LOGGER, "Could not find planner plugin name: %s", planner_plugin_name.c_str());
        
    try
    {
        planner_plugin_loader.reset(
            new pluginlib::ClassLoader<planning_interface::PlannerManager>(
                "moveit_core", 
                "planning_interface::PlannerManager"
            )
        );
    }
    catch(pluginlib::PluginlibException& ex)
    {
        RCLCPP_FATAL(LOGGER, "Exception while creating planning plugin loader %s", ex.what());
    }

    try
    {
        planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
        if(!planner_instance->initialize(
                robot_model, 
                motion_planning_api_tutorial_node, 
                motion_planning_api_tutorial_node->get_namespace()
            )
        )
        {
            RCLCPP_FATAL(LOGGER, "Could not initialize planner instance");
        }
        RCLCPP_INFO(LOGGER, "Using planning interface '%s'", planner_instance->getDescription().c_str());
    }
    catch(pluginlib::PluginlibException& ex)
    {
        const std::vector<std::string>& classes = planner_plugin_loader->getDeclaredClasses();
        std::stringstream ss;
        for (const auto& cls : classes)
        ss << cls << " ";
        RCLCPP_ERROR(LOGGER, "Exception while loading planner '%s': %s\nAvailable plugins: %s", planner_plugin_name.c_str(),
                    ex.what(), ss.str().c_str());
    }
    moveit::planning_interface::MoveGroupInterface move_group(motion_planning_api_tutorial_node, PLANNING_GROUP);

    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools(motion_planning_api_tutorial_node, "base_link", rvt::RVIZ_MARKER_TOPIC, move_group.getRobotModel());
    visual_tools.enableBatchPublishing();
    visual_tools.deleteAllMarkers();
    visual_tools.trigger();

    /* Remote control is an introspection tool that allows users to step through a high level script
        via buttons and keyboard shortcuts in RViz */
    visual_tools.loadRemoteControl();

    // Rviz
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.75;
    visual_tools.publishText(text_pose, "Motion Planning API demo", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo. . .");

    // Pose Goal
    visual_tools.trigger();
    planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res;
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "base_link";
    pose.pose.position.x = 0.3;
    pose.pose.position.y = 0.4;
    pose.pose.position.z = 0.75;
    pose.pose.orientation.w = 1.0;

    // Pose Goal
    std::vector<double> tolerance_pose(3,0.01);
    std::vector<double> tolerance_angle(3,0.01);

    // We will create the request as a constraint using a helper function available
    // from the
    // :moveit_codedir:`kinematic_constraints<moveit_core/kinematic_constraints/include/moveit/kinematic_constraints/kinematic_constraint.h>`
    // package.
    moveit_msgs::msg::Constraints pose_goal = 
        kinematic_constraints::constructGoalConstraints("tcp_link", pose, tolerance_pose, tolerance_angle);
    req.group_name = PLANNING_GROUP;
    req.goal_constraints.push_back(pose_goal);

    // We now construct a planning context that encapsulate the scene,
    // the request and the response. We call the planner using this
    // planning context
    planning_interface::PlanningContextPtr context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
    context->solve(res);
    if(res.error_code_.val != res.error_code_.SUCCESS)
    {
        RCLCPP_ERROR(LOGGER, "Could not compute plan successfully");
        return 0;
    }

    // Visualize the result
    std::shared_ptr<rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>> display_publisher =
        motion_planning_api_tutorial_node->create_publisher<moveit_msgs::msg::DisplayTrajectory>("/display_planned_path",
                                                                                                1);
    moveit_msgs::msg::DisplayTrajectory display_trajectory;

    /* Visualize the trajectory */
    moveit_msgs::msg::MotionPlanResponse response;
    res.getMessage(response);

    display_trajectory.trajectory_start = response.trajectory_start;
    display_trajectory.trajectory.push_back(response.trajectory);
    visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
    visual_tools.trigger();
    display_publisher->publish(display_trajectory);

    /* Set the state in the planning scene to the final state of the last plan */
    robot_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
    planning_scene->setCurrentState(*robot_state.get());

    // Display the goal state
    visual_tools.publishAxisLabeled(pose.pose, "goal_1");
    visual_tools.publishText(text_pose, "Pose Goal (1)", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();

    /* We can also use visual_tools to wait for user input */
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    // Joint Space Goals
    // ^^^^^^^^^^^^^^^^^
    // Now, setup a joint space goal
    moveit::core::RobotState goal_state(robot_model);
    std::vector<double> joint_values = { -1.0, 0.7, 0.7, -1.5, -0.7, 2.0};
    goal_state.setJointGroupPositions(joint_model_group, joint_values);
    moveit_msgs::msg::Constraints joint_goal =
        kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);
    req.goal_constraints.clear();
    req.goal_constraints.push_back(joint_goal);

    // Call the planner and visualize the trajectory
    /* Re-construct the planning context */
    context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
    /* Call the Planner */
    context->solve(res);
    /* Check that the planning was successful */
    if (res.error_code_.val != res.error_code_.SUCCESS)
    {
        RCLCPP_ERROR(LOGGER, "Could not compute plan successfully");
        return 0;
    }

    /* Visualize the trajectory */
    res.getMessage(response);
    display_trajectory.trajectory.push_back(response.trajectory);

    /* Now you should see two planned trajectories in series*/
    visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
    visual_tools.trigger();
    display_publisher->publish(display_trajectory);

    /* We will add more goals. But first, set the state in the planning
        scene to the final state of the last plan */
    robot_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
    planning_scene->setCurrentState(*robot_state.get());

    // Display the goal state
    visual_tools.publishAxisLabeled(pose.pose, "goal_2");
    visual_tools.publishText(text_pose, "Joint Space Goal (2)", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();

    /* Wait for user input */
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    /* Now, we go back to the first goal to prepare for orientation constrained planning */
    req.goal_constraints.clear();
    req.goal_constraints.push_back(pose_goal);
    context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
    context->solve(res);
    res.getMessage(response);

    display_trajectory.trajectory.push_back(response.trajectory);
    visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
    visual_tools.trigger();
    display_publisher->publish(display_trajectory);

    /* Set the state in the planning scene to the final state of the last plan */
    robot_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
    planning_scene->setCurrentState(*robot_state.get());

    // Display the goal state
    visual_tools.trigger();

    /* Wait for user input */
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    // Adding Path Constraints
    // ^^^^^^^^^^^^^^^^^^^^^^^
    // Let's add a new pose goal again. This time we will also add a path constraint to the motion.
    /* Let's create a new pose goal */
    pose.pose.position.x = 0.32;
    pose.pose.position.y = -0.25;
    pose.pose.position.z = 0.65;
    pose.pose.orientation.w = 1.0;
    moveit_msgs::msg::Constraints pose_goal_2 =
        kinematic_constraints::constructGoalConstraints("tcp_link", pose, tolerance_pose, tolerance_angle);
        
    /* Now, let's try to move to this new pose goal*/
    req.goal_constraints.clear();
    req.goal_constraints.push_back(pose_goal_2);

    /* But, let's impose a path constraint on the motion.
        Here, we are asking for the end-effector to stay level*/
    geometry_msgs::msg::QuaternionStamped quaternion;
    quaternion.header.frame_id = "base_link";
    req.path_constraints = kinematic_constraints::constructGoalConstraints("tcp_link", quaternion);

    // Imposing path constraints requires the planner to reason in the space of possible positions of the end-effector
    // (the workspace of the robot)
    // because of this, we need to specify a bound for the allowed planning volume as well;
    // Note: a default bound is automatically filled by the WorkspaceBounds request adapter (part of the OMPL pipeline,
    // but that is not being used in this example).
    // We use a bound that definitely includes the reachable space for the arm. This is fine because sampling is not done
    // in this volume
    // when planning for the arm; the bounds are only used to determine if the sampled configurations are valid.
    req.workspace_parameters.min_corner.x = req.workspace_parameters.min_corner.y =
        req.workspace_parameters.min_corner.z = -2.0;
    req.workspace_parameters.max_corner.x = req.workspace_parameters.max_corner.y =
        req.workspace_parameters.max_corner.z = 2.0;

    // Call the planner and visualize all the plans created so far.
    context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
    context->solve(res);
    res.getMessage(response);
    display_trajectory.trajectory.push_back(response.trajectory);
    visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
    visual_tools.trigger();
    display_publisher->publish(display_trajectory);

    /* Set the state in the planning scene to the final state of the last plan */
    robot_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
    planning_scene->setCurrentState(*robot_state.get());

    // Display the goal state
    visual_tools.publishAxisLabeled(pose.pose, "goal_3");
    visual_tools.publishText(text_pose, "Orientation Constrained Motion Plan (3)", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();

    // END_TUTORIAL
    /* Wait for user input */
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to exit the demo");
    planner_instance.reset();

    rclcpp::shutdown();
    return 0;
}   