#include <ros/ros.h>

#include <moveit/robot_model/robot_model.h>

#include <moveit/task_constructor/task.h>

#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/stages/generate_pose.h>

#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>

#include <moveit/task_constructor/stages/pour_into.h>

#include "mtc_pour/demo_utils.hpp"

using namespace moveit::task_constructor;

int main(int argc, char** argv){
	ros::init(argc, argv, "tiago_bartender");

	ros::AsyncSpinner spinner(1);
	spinner.start();

	Task t("test_task");
	t.loadRobotModel();

	// don't spill liquid
	moveit_msgs::Constraints upright_constraint;
	upright_constraint.name = "gripper_grasping_frame:upright";
	upright_constraint.orientation_constraints.resize(1);
	{
		moveit_msgs::OrientationConstraint& c= upright_constraint.orientation_constraints[0];
		c.link_name= "gripper_grasping_frame";
		c.header.frame_id= "base_footprint";
		c.orientation.w= 1.0;
		//c.absolute_x_axis_tolerance= M_PI;
		//c.absolute_y_axis_tolerance= M_PI;
		c.absolute_x_axis_tolerance= 0.65;
		c.absolute_y_axis_tolerance= 0.65;
		c.absolute_z_axis_tolerance= M_PI;
		c.weight= 1.0;
	}

	auto sampling_planner = std::make_shared<solvers::PipelinePlanner>();
	sampling_planner->setProperty("goal_joint_tolerance", 1e-5);

	auto cartesian_planner = std::make_shared<solvers::CartesianPath>();
	cartesian_planner->setMaxVelocityScaling(.3);
	cartesian_planner->setMaxAccelerationScaling(.3);
	cartesian_planner->setStepSize(.002);

	t.setProperty("group", "arm_torso");

	Stage* current_state= nullptr;
	{
		auto _current_state = std::make_unique<stages::CurrentState>("current state");
		current_state= _current_state.get();
		t.add(std::move(_current_state));
	}

	{
		auto stage = std::make_unique<stages::MoveRelative>("move", sampling_planner);
		stage->properties().set("marker_ns", "move");
		stage->properties().set("link", "gripper_grasping_frame");
		stage->properties().configureInitFrom(Stage::PARENT, {"group"});
		stage->setMinMaxDistance(.05, .3);
		stage->setPathConstraints(upright_constraint);
		stage->properties().set("timeout", 10.0);

		geometry_msgs::Vector3Stamped vec;
		vec.header.frame_id = "base_footprint";
		vec.vector.y = 1.0;
		stage->along(vec);
		t.add(std::move(stage));
	}

	t.enableIntrospection();

	ros::NodeHandle nh("~");

	bool execute= nh.param<bool>("execute", true);

	if(execute){
		ROS_INFO("Going to execute first computed solution");
	}

	ROS_INFO_STREAM( t );

	std::cout << "waiting for <enter> to begin" << std::endl;
	std::cin.get();

	try {
		t.plan(1);
	}
	catch(InitStageException& e){
		ROS_ERROR_STREAM(e);
	}

	if(execute && t.numSolutions() > 0){
		moveit_task_constructor_msgs::Solution solution;
		t.solutions().front()->fillMessage(solution);
		mtc_pour::executeSolution(solution);
	}

	std::cout << "waiting for <enter>" << std::endl;
	std::cin.get();

	return 0;
}
