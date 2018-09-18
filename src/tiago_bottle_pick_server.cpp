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

#include <actionlib/server/simple_action_server.h>
#include <tiago_bartender_mtc/PickBottleAction.h>
#include <std_srvs/Empty.h>

using namespace moveit::task_constructor;

class TiagoBottlePick
{
public:
  TiagoBottlePick() : 
    as_(nh_, "tiago_bottle_pick", boost::bind(&TiagoBottlePick::pick_cb, this, _1), false)
  {
    scene_update_client_ = nh_.serviceClient<std_srvs::Empty>("dummy_planning_scene/update_planning_scene");
    as_.start();
  }

  void pick_cb(const tiago_bartender_mtc::PickBottleGoalConstPtr& goal)
  {
	std_srvs::Empty srv;
        if (!scene_update_client_.call(srv))
		ROS_ERROR("Failed to call service update_planning_scene");

        ros::Duration(1.0).sleep();

	std::string bottle = goal->bottle_id;

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
		stage->setDirection(vec);
		t.add(std::move(stage));
	}

	{
		auto stage = std::make_unique<stages::GenerateGraspPose>("grasp work space pose");
		stage->properties().configureInitFrom(Stage::PARENT);
		stage->setPreGraspPose("open");
		stage->setObject(bottle);
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
		stage->allowCollisions(bottle, t.getRobotModel()->getJointModelGroup("gripper")->getLinkModelNamesWithCollisionGeometry(), true);
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
		stage->attachObject(bottle, "gripper_grasping_frame");
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
		stage->setDirection(vec);
		t.add(std::move(stage));
	}

	{
		auto stage = std::make_unique<stages::MoveRelative>("retreat object", cartesian_planner);
		stage->properties().configureInitFrom(Stage::PARENT, {"group"});
		stage->setMinMaxDistance(0.01,0.10);
		stage->setIKFrame("gripper_grasping_frame");

		stage->properties().set("marker_ns", "retreat");

		geometry_msgs::Vector3Stamped vec;
		vec.header.frame_id= "base_footprint";
		vec.vector.x= -1.0;
		stage->setDirection(vec);
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

	tiago_bartender_mtc::PickBottleResult result;
	as_.setSucceeded(result);

  }
private:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<tiago_bartender_mtc::PickBottleAction> as_;
  ros::ServiceClient scene_update_client_;
};

int main(int argc, char** argv){
	ros::init(argc, argv, "tiago_bartender");

	ros::AsyncSpinner spinner(1);
	spinner.start();
        TiagoBottlePick tbp;

	while(ros::ok())
        {

        }

	return 0;
}
