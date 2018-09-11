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

#include <moveit_task_constructor_msgs/ExecuteTaskSolutionAction.h>

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

#include <tiago_bartender_msgs/PickAction.h>
#include <std_srvs/Empty.h>

using namespace moveit::task_constructor;

class TiagoBartenderPick
{
public:
	TiagoBartenderPick() :
		as_(nh_, "tiago_pick", boost::bind(&TiagoBartenderPick::pick_cb, this, _1), false),
		execute_("execute_task_solution", true)
	{
		execute_.waitForServer();

		//scene_client_ = nh_.serviceClient<moveit_msgs::GetPlanningScene>("get_planning_scene");
		as_.start();
	}

	void pick_cb(const tiago_bartender_msgs::PickGoalConstPtr& goal)
	{
		//	moveit_msgs::GetPlanningScene srv;
		//	srv.components.components = 1023; // get *full* scene
		//	if (!scene_client_.call(srv)){
		//		ROS_ERROR("Failed to call service update_planning_scene");
		//		as_.setAborted();
		//		return;
		//	}

		std::string object = goal->object_id;

		task_.reset( new moveit::task_constructor::Task );
		Task& t= *task_;
		t.loadRobotModel();

		auto sampling_planner = std::make_shared<solvers::PipelinePlanner>();
		sampling_planner->setProperty("goal_joint_tolerance", 1e-5);

		auto cartesian_planner = std::make_shared<solvers::CartesianPath>();
		cartesian_planner->setMaxVelocityScaling(1.0);
		cartesian_planner->setMaxAccelerationScaling(1.0);
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
			auto stage = std::make_unique<stages::MoveRelative>("approach object", cartesian_planner);
			stage->properties().set("marker_ns", "approach");
			stage->properties().set("link", "gripper_grasping_frame");
			stage->properties().configureInitFrom(Stage::PARENT, {"group"});
			stage->setMinMaxDistance(.10, .15);

			geometry_msgs::Vector3Stamped vec;
			vec.header.frame_id = "gripper_grasping_frame";
			vec.vector.x = 1.0;
			stage->along(vec);
			t.add(std::move(stage));
		}

		Stage* ik_state= nullptr;
		{
			auto stage = std::make_unique<stages::GenerateGraspPose>("grasp work space pose");
			stage->properties().configureInitFrom(Stage::PARENT);
			stage->setNamedPose("open");
			stage->setObject(object);
			stage->setAngleDelta(M_PI/6);

			stage->setMonitoredStage(current_state);

			auto wrapper = std::make_unique<stages::ComputeIK>("grasp pose", std::move(stage) );
			ik_state= wrapper.get();
			wrapper->setMaxIKSolutions(8);
			wrapper->setIKFrame(Eigen::Translation3d(0.05,0,-.09), "gripper_grasping_frame");
			wrapper->properties().configureInitFrom(Stage::PARENT, {"eef"});
			t.add(std::move(wrapper));
		}

		{
			auto stage = std::make_unique<stages::ModifyPlanningScene>("allow gripper->object collision");
			stage->allowCollisions(object, t.getRobotModel()->getJointModelGroup("gripper")->getLinkModelNamesWithCollisionGeometry(), true);
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
			stage->attachObject(object, "gripper_grasping_frame");
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
			auto stage = std::make_unique<stages::MoveRelative>("retreat object", cartesian_planner);
			stage->properties().configureInitFrom(Stage::PARENT, {"group"});
			stage->setMinMaxDistance(0.01,0.10);
			stage->setIKFrame("gripper_grasping_frame");

			stage->properties().set("marker_ns", "retreat");

			geometry_msgs::Vector3Stamped vec;
			vec.header.frame_id= "base_footprint";
			vec.vector.x= -1.0;
			stage->along(vec);
			t.add(std::move(stage));
		}

		{
			auto stage = std::make_unique<stages::MoveTo>("move home", sampling_planner);
			stage->properties().configureInitFrom(Stage::PARENT, {"group"});
			stage->setGoal("transport2");
			t.add(std::move(stage));
		}

		t.enableIntrospection();

		try {
			t.plan(1);
		}
		catch(InitStageException& e){
			ROS_ERROR_STREAM(e);
			tiago_bartender_msgs::PickResult result;
			result.result.result = tiago_bartender_msgs::ManipulationResult::INTERNAL_ERROR;
			as_.setAborted(result, "Initialization failed");
			return;
		}

		if(t.numSolutions() == 0){
			tiago_bartender_msgs::PickResult result;

			if(ik_state->solutions().size() == 0){
				result.result.result = tiago_bartender_msgs::ManipulationResult::UNREACHABLE;
				as_.setAborted(result, "IK failed, is the object reachable?");
			}
			else {
				result.result.result = tiago_bartender_msgs::ManipulationResult::NO_PLAN_FOUND;
				as_.setAborted(result, "Planning failed");
			}

			return;
		}

		moveit_task_constructor_msgs::Solution solution;
		t.solutions().front()->fillMessage(solution);

		moveit_task_constructor_msgs::ExecuteTaskSolutionGoal execute_goal;
		execute_goal.task_solution = solution;
		execute_.sendGoal(execute_goal);
		execute_.waitForResult();
		moveit_msgs::MoveItErrorCodes execute_result= execute_.getResult()->error_code;

		if(execute_result.val != moveit_msgs::MoveItErrorCodes::SUCCESS){
			ROS_ERROR_STREAM("task execution failed and returned: " << execute_.getState().toString());
			tiago_bartender_msgs::PickResult result;
			result.result.result = tiago_bartender_msgs::ManipulationResult::EXECUTION_FAILED;
			as_.setAborted(result, "Execution failed");
			return;
		}

		tiago_bartender_msgs::PickResult result;
		result.result.result= tiago_bartender_msgs::ManipulationResult::SUCCESS;
		as_.setSucceeded(result);
	}

private:
	ros::NodeHandle nh_;
	actionlib::SimpleActionServer<tiago_bartender_msgs::PickAction> as_;

	//ros::ServiceClient scene_update_client_;
   actionlib::SimpleActionClient<moveit_task_constructor_msgs::ExecuteTaskSolutionAction> execute_;

	// latest task is retained until new one arrives
	// to provide ROS interfaces for introspection
	moveit::task_constructor::TaskPtr task_;
};


int main(int argc, char** argv){
	ros::init(argc, argv, "tiago_bartender_mtc_pick");

	TiagoBartenderPick tbp;

	ros::spin();

	return 0;
}