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

const std::string BOTTLE="bottle_1";

int main(int argc, char** argv){
	ros::init(argc, argv, "tiago_bartender");

	ros::AsyncSpinner spinner(1);
	spinner.start();

	std::cout << "waiting for <enter>" << std::endl;
	std::cin.get();

	if(false){
		geometry_msgs::PoseStamped bottle, glass;
		bottle.header.frame_id= "base_footprint";
		bottle.pose.position.x= 0.5;
		bottle.pose.position.y= 0.15;
		bottle.pose.position.z= 0.7;
		bottle.pose.orientation.w= 1.0;

		glass.header.frame_id= "base_footprint";
		glass.pose.position.x= 0.5;
		glass.pose.position.y= -0.12;
		glass.pose.position.z= 0.7;
		glass.pose.orientation.w= 1.0;

		mtc_pour::cleanup();

		{
			moveit::planning_interface::MoveGroupInterface mgi("arm_torso");
			mgi.setNamedTarget("home");
			mgi.move();
		}

		moveit::planning_interface::PlanningSceneInterface psi;

		std::vector<moveit_msgs::CollisionObject> objs;
		mtc_pour::setupObjects(objs, bottle, glass);
		objs.erase(objs.begin()+1);
		mtc_pour::setupTable(objs, glass);

		std::vector<moveit_msgs::ObjectColor> colors;
//		colors.emplace_back();
//		colors.back().id= "glass";
//		colors.back().color.a= 1;
//		colors.back().color.r= 0;
//		colors.back().color.g= .5;
//		colors.back().color.b= .4;

		colors.emplace_back();
		colors.back().id= "table";
		colors.back().color.a= 1;
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
	sampling_planner->setProperty("timeout", 10.0);

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
		auto stage = std::make_unique<stages::MoveTo>("open gripper", sampling_planner);
		stage->setGroup("gripper");
		stage->setGoal("open");
		t.add(std::move(stage));
	}

	{
		auto stage = std::make_unique<stages::Connect>("move to pre-grasp pose", stages::Connect::GroupPlannerVector {{"arm_torso", sampling_planner}});
		stage->properties().configureInitFrom(Stage::PARENT);
		t.add(std::move(stage));
	}

	{
		auto stage = std::make_unique<stages::MoveRelative>("approach bottle", cartesian_planner);
		stage->properties().set("marker_ns", "approach");
		stage->properties().set("link", "gripper_grasping_frame");
		stage->properties().configureInitFrom(Stage::PARENT, {"group"});
		stage->setMinMaxDistance(.10, .20);

		geometry_msgs::Vector3Stamped vec;
		vec.header.frame_id = "gripper_grasping_frame";
		vec.vector.x = 1.0;
		stage->along(vec);
		t.add(std::move(stage));
	}

	{
		auto stage = std::make_unique<stages::GenerateGraspPose>("grasp work space pose");
		stage->properties().configureInitFrom(Stage::PARENT);
		stage->setNamedPose("open");
		stage->setObject(BOTTLE);
		stage->setAngleDelta(M_PI/6);

		stage->setMonitoredStage(current_state);

		auto wrapper = std::make_unique<stages::ComputeIK>("grasp pose", std::move(stage) );
		wrapper->setMaxIKSolutions(8);
		wrapper->setIKFrame(Eigen::Translation3d(0.05,0,-.09), "gripper_grasping_frame");
		wrapper->properties().configureInitFrom(Stage::PARENT, {"eef"});
		t.add(std::move(wrapper));
	}

	{
		auto stage = std::make_unique<stages::ModifyPlanningScene>("allow gripper->object collision");
		stage->allowCollisions(BOTTLE, t.getRobotModel()->getJointModelGroup("gripper")->getLinkModelNamesWithCollisionGeometry(), true);
		t.add(std::move(stage));
	}

	{
		auto stage = std::make_unique<stages::MoveTo>("close gripper", sampling_planner);
		stage->properties().property("group").configureInitFrom(Stage::PARENT, "gripper");
		stage->setGoal("closed");
		t.add(std::move(stage));
	}

	Stage* object_grasped= nullptr;
	{
		auto stage = std::make_unique<stages::ModifyPlanningScene>("attach object");
		stage->attachObject(BOTTLE, "gripper_grasping_frame");
		object_grasped= stage.get();
		t.add(std::move(stage));
	}

	{
		auto stage = std::make_unique<stages::MoveRelative>("lift object", cartesian_planner);
		stage->properties().configureInitFrom(Stage::PARENT, {"group"});
		stage->setMinMaxDistance(.01,.10);
		stage->setIKFrame("gripper_grasping_frame");

		stage->properties().set("marker_ns", "lift");

		geometry_msgs::Vector3Stamped vec;
		vec.header.frame_id= "base_footprint";
		vec.vector.z= 1.0;
		stage->along(vec);
		t.add(std::move(stage));
	}

	{
     //<group_state name="transport2" group="arm_torso">
     //    <joint name="torso_lift_joint" value="0.23"/>
     //    <joint name="arm_1_joint" value="0.19"/>
     //    <joint name="arm_2_joint" value="0.15"/>
     //    <joint name="arm_3_joint" value="-2.27"/>
     //    <joint name="arm_4_joint" value="2.28"/>
     //    <joint name="arm_5_joint" value="1.62"/>
     //    <joint name="arm_6_joint" value="-0.65"/>
     //    <joint name="arm_7_joint" value="2.03"/>
     //</group_state>

		auto stage = std::make_unique<stages::MoveTo>("move back", sampling_planner);
		stage->properties().configureInitFrom(Stage::PARENT, {"group"});
		stage->properties().set("timeout", 10.0);
		stage->setPathConstraints(upright_constraint);
		//stage->setIKFrame("gripper_grasp_frame");
		//geometry_msgs::PointStamped point;
		//point.header.frame_id= "base_footprint";
		//point.point.x= .5;
		//point.point.y= 0;
		//point.point.z= 1.05;
		//stage->setGoal(point);
		stage->setGoal("transport2");
		t.add(std::move(stage));
	}

	t.enableIntrospection();

	ros::NodeHandle nh("~");

	bool execute= nh.param<bool>("execute", true);

	if(execute){
		ROS_INFO("Going to execute first computed solution");
	}

	ROS_INFO_STREAM( t );

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
