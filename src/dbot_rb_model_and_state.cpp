#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    // This enables loading undeclared parameters
    // best practice would be to declare parameters in the corresponding classes
    // and provide descriptions about expected use
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("dbot_rb_model_and_state_node", node_options);
    const auto& LOGGER = node->get_logger();

    // Robot Model
    robot_model_loader::RobotModelLoader robot_model_loader(node);
    const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
    RCLCPP_INFO(LOGGER, "Model frame: %s", kinematic_model->getModelFrame().c_str());

    // Using the :moveit_codedir:`RobotModel<moveit_core/robot_model/include/moveit/robot_model/robot_model.h>`, we can
    // construct a :moveit_codedir:`RobotState<moveit_core/robot_state/include/moveit/robot_state/robot_state.h>` that
    // maintains the configuration of the robot. We will set all joints in the state to their default values. We can then
    // get a :moveit_codedir:`JointModelGroup<moveit_core/robot_model/include/moveit/robot_model/joint_model_group.h>`,
    // which represents the robot model for a particular group, e.g. the "panda_arm" of the Panda robot.
    // Robot State
    moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(kinematic_model));
    robot_state->setToDefaultValues();
    const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("dbot_arm");
    const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

    // Get Joint Values
    std::vector<double> joint_values;
    robot_state->copyJointGroupPositions(joint_model_group, joint_values);
    for (size_t i = 0; i < joint_names.size(); i++)
    {
        RCLCPP_INFO(LOGGER, "Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }
    
    // Joint Limits
    joint_values[0] = 5.57;
    robot_state->setJointGroupPositions(joint_model_group, joint_values);
    RCLCPP_INFO_STREAM(LOGGER, "Current state is " << (robot_state->satisfiesBounds() ? "valid" : "not valid"));
    robot_state->enforceBounds();
    RCLCPP_INFO_STREAM(LOGGER, "Current state is " << (robot_state->satisfiesBounds() ? "valid" : "not valid"));

    // Forward Kinematics
    robot_state->setToRandomPositions(joint_model_group);
    const Eigen::Isometry3d& end_effector_state = robot_state->getGlobalLinkTransform("tcp_link");
    RCLCPP_INFO_STREAM(LOGGER, "Translation: \n" << end_effector_state.translation() << "\n");
    RCLCPP_INFO_STREAM(LOGGER, "Rotation: \n" << end_effector_state.rotation() << "\n");

    // Inverse Kinematics
    // ^^^^^^^^^^^^^^^^^^
    // We can now solve inverse kinematics (IK) for the Panda robot.
    // To solve IK, we will need the following:
    //
    //  * The desired pose of the end-effector (by default, this is the last link in the "panda_arm" chain):
    //    end_effector_state that we computed in the step above.
    //  * The timeout: 0.1 s
    double timeout = 0.1;
    bool found_ik = robot_state->setFromIK(joint_model_group, end_effector_state, timeout);
    if(found_ik)
    {
        robot_state->copyJointGroupPositions(joint_model_group, joint_values);
        for (size_t i = 0; i < joint_names.size(); i++)
        {
            RCLCPP_INFO(LOGGER, "Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
        }

    }
    else
    {
        RCLCPP_INFO(LOGGER, "Did not find IK solution.");
    }

    // Get the Jacobian
    Eigen::Vector3d reference_joint_position(0.0,0.0,0.0);
    Eigen::MatrixXd jacobian;
    robot_state->getJacobian(joint_model_group, robot_state->getLinkModel("tcp_link"), reference_joint_position, jacobian);
    RCLCPP_INFO_STREAM(LOGGER, "Jacobian: \n" << jacobian << "\n");

    rclcpp::shutdown();
    return 0;
}