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
	ros::init(argc, argv, "tiago_barkeeper");

	ros::AsyncSpinner spinner(1);
	spinner.start();

	{
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

		cleanup();

		{
			moveit::planning_interface::MoveGroupInterface mgi("arm_torso");
			mgi.setNamedTarget("home");
			mgi.move();
		}

		moveit::planning_interface::PlanningSceneInterface psi;

		std::vector<moveit_msgs::CollisionObject> objs;
		setupObjects(objs, bottle, glass);
		objs.erase(objs.begin()+1);
		setupTable(objs, glass);

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
		stage->setObject("bottle");
		stage->setAngleDelta(M_PI/6);

		stage->setMonitoredStage(current_state);

		auto wrapper = std::make_unique<stages::ComputeIK>("grasp pose", std::move(stage) );
		wrapper->setMaxIKSolutions(8);
		wrapper->setIKFrame(Eigen::Translation3d(0.05,0,0), "gripper_grasping_frame");
		wrapper->properties().configureInitFrom(Stage::PARENT, {"eef"});
		t.add(std::move(wrapper));
	}

	{
		auto stage = std::make_unique<stages::ModifyPlanningScene>("allow gripper->object collision");
		stage->allowCollisions("bottle", t.getRobotModel()->getJointModelGroup("gripper")->getLinkModelNamesWithCollisionGeometry(), true);
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
		stage->attachObject("bottle", "gripper_grasping_frame");
		object_grasped= stage.get();
		t.add(std::move(stage));
	}

	{
		auto stage = std::make_unique<stages::MoveRelative>("lift object", cartesian_planner);
		stage->properties().configureInitFrom(Stage::PARENT, {"group"});
		stage->setMinMaxDistance(.05,.10);
		stage->setIKFrame("gripper_grasping_frame");

		stage->properties().set("marker_ns", "lift");

		geometry_msgs::Vector3Stamped vec;
		vec.header.frame_id= "base_footprint";
		vec.vector.z= 1.0;
		stage->along(vec);
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

	if(!execute){
		std::cout << "waiting for <enter>" << std::endl;
		std::cin.get();
	}
	else {
		moveit_task_constructor_msgs::Solution solution;
		t.solutions().front()->fillMessage(solution);
		executeSolution(solution);
	}

	return 0;
}
