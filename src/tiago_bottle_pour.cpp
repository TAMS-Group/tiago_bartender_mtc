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

	{
		moveit::planning_interface::PlanningSceneInterface psi;

		geometry_msgs::PoseStamped glass;
		glass.header.frame_id= "base_footprint";
		glass.pose.position.x= 0.5;
		glass.pose.position.y= -0.12;
		glass.pose.position.z= 0.7;
		glass.pose.orientation.w= 1.0;

		geometry_msgs::PoseStamped table;
		table.header.frame_id= "base_footprint";
		table.pose.position.x= 0.5;
		table.pose.position.y= 0.0;
		table.pose.position.z= 0.73;
		table.pose.orientation.w= 1.0;

		std::vector<moveit_msgs::CollisionObject> objs;

		mtc_pour::setupObjects(objs, glass /* stub bottle pose */, glass);
		objs.erase(objs.begin()); // remove the bottle - assume it's attached

		mtc_pour::setupTable(objs, table);

		std::vector<moveit_msgs::ObjectColor> colors;
		colors.emplace_back();
		colors.back().color.a= 1;
		colors.back().color.r= 0;
		colors.back().color.g= .5;
		colors.back().color.b= .4;

		colors.emplace_back();
		colors.back().color.a= 1;
		colors.back().color.r= .4;
		colors.back().color.g= .2;
		colors.back().color.b= .0;

		psi.applyCollisionObjects(objs, colors);
	}

	Task t;
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
	t.setProperty("eef", "gripper");
	t.setProperty("gripper", "gripper");

	Stage* current_state= nullptr;
	{
		auto _current_state = std::make_unique<stages::CurrentState>("current state");
		current_state= _current_state.get();
		t.add(std::move(_current_state));
	}

	{
		auto stage = std::make_unique<stages::Connect>("move to pre-pour pose", stages::Connect::GroupPlannerVector{{"arm_torso", sampling_planner}});
		stage->setTimeout(15.0);
		//stage->setPathConstraints(upright_constraint);
		stage->properties().configureInitFrom(Stage::PARENT);
		t.add(std::move(stage));
	}

	{
		auto stage = std::make_unique<stages::GeneratePose>("pose above glass");
		geometry_msgs::PoseStamped p;
		p.header.frame_id= "glass";
		p.pose.orientation.w= 1;
		p.pose.position.z= .3;
		stage->setPose(p);
		stage->properties().configureInitFrom(Stage::PARENT);

		stage->setMonitoredStage(current_state);

		auto wrapper = std::make_unique<stages::ComputeIK>("pre-pour pose", std::move(stage) );
		wrapper->setMaxIKSolutions(32);
		wrapper->setIKFrame("gripper_grasping_frame");
		wrapper->properties().configureInitFrom(Stage::PARENT, {"eef"});
		t.add(std::move(wrapper));
	}

	{
		auto stage = std::make_unique<mtc_pour::PourInto>("pouring");
		stage->setBottle("bottle");
		stage->setContainer("glass");
		stage->setPourOffset(Eigen::Vector3d(0,0.015,0.035));
		stage->setTiltAngle(2.0);
		stage->setPourDuration(ros::Duration(4.0));
		stage->properties().configureInitFrom(Stage::PARENT);
		t.add(std::move(stage));
	}

//	{
//		auto stage = std::make_unique<stages::MoveTo>("move home", sampling_planner);
//		stage->properties().configureInitFrom(Stage::PARENT, {"group"});
//		stage->setGoal("transport");
//		t.add(std::move(stage));
//	}

	t.enableIntrospection();

	ros::NodeHandle nh("~");

	bool execute= nh.param<bool>("execute", true);

	if(execute){
		ROS_INFO("Going to execute first computed solution");
	}

	ROS_INFO_STREAM( t );

	try {
		t.plan();
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
