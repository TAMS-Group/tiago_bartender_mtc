#include <ros/ros.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit/task_constructor/task.h>

#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/stages/generate_pose.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/predicate_filter.h>

#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>

#include <moveit/task_constructor/stages/pour_into.h>

#include <moveit_task_constructor_msgs/ExecuteTaskSolutionAction.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>

#include <tiago_bartender_msgs/PickAction.h>
#include <tiago_bartender_msgs/PlaceAction.h>
#include <tiago_bartender_msgs/PlacePickAction.h>
#include <tiago_bartender_msgs/PourAction.h>

#include <std_srvs/Empty.h>

using namespace moveit::task_constructor;

moveit_msgs::RobotState jointsToRS(std::vector<double> joint_positions) {
  moveit_msgs::RobotState rs;
  rs.is_diff = true;
  rs.joint_state.name = {"torso_lift_joint", "arm_1_joint", "arm_2_joint",
                         "arm_3_joint",      "arm_4_joint", "arm_5_joint",
                         "arm_6_joint",      "arm_7_joint"};
  rs.joint_state.position = joint_positions;

  assert(joint_positions.size() == rs.joint_state.name.size());

  return rs;
}

class TiagoBartender {
public:
  TiagoBartender()
      : as_pick_(nh_, "tiago_pick",
                 boost::bind(&TiagoBartender::pick_cb, this, _1), false),
        as_pour_(nh_, "tiago_pour",
                 boost::bind(&TiagoBartender::pour_cb, this, _1), false),
        as_place_pick_(nh_, "tiago_place_pick",
                       boost::bind(&TiagoBartender::place_pick_cb, this, _1),
                       false),
        as_place_(nh_, "tiago_place",
                  boost::bind(&TiagoBartender::place_cb, this, _1), false),

        execute_("execute_task_solution", true) {
    ROS_INFO("waiting for task execution");
    execute_.waitForServer();

    ros::NodeHandle pnh("~");

    execute_solutions_ = pnh.param<bool>("execute", true);

    // general parameters
    pourer_length_ = pnh.param<double>("pourer_length", 0.015);
    bottle_height_ = pnh.param<double>("bottle_height", 0.25);
    table_height_ = pnh.param<double>("table_height",
                                      0.75); // TODO: load from bringup config

    pre_pour_height_ = pnh.param<double>("pre_pour_height", 0.3);
    pour_angle_ = pnh.param<double>("pour_angle", 2.2);
    lift_object_min_dist_ = pnh.param<double>("lift_object_min_dist", 0.01);
    lift_object_max_dist_ = pnh.param<double>("lift_object_max_dist", 0.1);
    approach_object_min_dist_ =
        pnh.param<double>("approach_object_min_dist", 0.10);
    approach_object_max_dist_ =
        pnh.param<double>("approach_object_max_dist", 0.15);
    place_surface_offset_ = pnh.param<double>("place_surface_offset", 0.01);
    place_distance_ = pnh.param<double>("place_distance", 0.9);

    // compute gripper grasp frame depending on pourer length
    double grasp_offset_x = pnh.param<double>("grasp_offset_x", 0.04);
    double grasp_offset_z =
        pnh.param<double>("prasp_offset_z", 0.0) + 0.5 * pourer_length_;
    grasp_frame_transform_ =
        Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()) *
        Eigen::Translation3d(grasp_offset_x, 0, grasp_offset_z);

    // compute pour offset from glass depending on pourer length and pour angle
    double pour_offset_x = pnh.param<double>("pour_offset_x", 0.02);
    double pour_offset_z = pnh.param<double>("pour_offset_z", 0.025);
    double p_x = std::abs(std::sin(pour_angle_) * pourer_length_);
    double p_z = std::abs(std::cos(pour_angle_) * pourer_length_);
    pour_offset_ = Eigen::Vector3d(0, pour_offset_x + p_x, pour_offset_z + p_z);

    // don't spill liquid
    upright_constraint_.name = "gripper_grasping_frame:upright";
    upright_constraint_.orientation_constraints.resize(1);
    {
      moveit_msgs::OrientationConstraint &c =
          upright_constraint_.orientation_constraints[0];
      c.link_name = "gripper_grasping_frame";
      c.header.frame_id = "base_footprint";
      c.orientation.w = 1.0;
      c.absolute_x_axis_tolerance = 0.65;
      c.absolute_y_axis_tolerance = 0.65;
      c.absolute_z_axis_tolerance = M_PI;
      c.weight = 1.0;
    }

    supports_ =
        std::vector<std::string>{"table1", "table2", "table3", "invisible_box"};

    transport_pose_ = jointsToRS({
        0.30, 0.13, -0.10, -1.47, 2.29, -1.66, 0.90, 1.43
        // 0.3, 0.2182262942567457, -0.07057563931682798,
        // -1.2894996186397367, 2.3097855008155945, -1.568529541083217,
        // 0.578567797265917, -1.8625135096142151
    });
    // scene_client_ =
    // nh_.serviceClient<moveit_msgs::GetPlanningScene>("get_planning_scene");

    as_pick_.start();
    as_pour_.start();
    as_place_pick_.start();
    as_place_.start();

    ROS_INFO("Bartender MTC actions are available");

    if (!execute_solutions_) {
      ROS_INFO("Planned trajectories will not be executed.");
    } else {
      ROS_INFO("Solutions will be executed");
    }
  }

  std::unique_ptr<stages::PredicateFilter>
  filter_upright(moveit::task_constructor::Task &t,
                 std::unique_ptr<Stage> &&stage) {
    auto valid_trajectory = std::make_unique<stages::PredicateFilter>(
        "validate upright", std::move(stage));
    t.properties().exposeTo(valid_trajectory->properties(), {"group"});
    valid_trajectory->properties().configureInitFrom(Stage::PARENT, {"group"});
    valid_trajectory->setPredicate(
        [this](const SolutionBase &s, std::string &comment) {
          robot_trajectory::RobotTrajectoryConstPtr trajectory =
              dynamic_cast<const SubTrajectory *>(&s)->trajectory();
          if (!s.start()->scene()->isPathValid(*trajectory,
                                               this->upright_constraint_)) {
            comment = "trajectory does not keep eef upright";
            return false;
          }
          return true;
        });
    return std::move(valid_trajectory);
  }

  void pick_cb(const tiago_bartender_msgs::PickGoalConstPtr &goal) {
    const std::string object = goal->object_id;

    pick_task_.reset(new moveit::task_constructor::Task("pick_object"));
    Task &t = *pick_task_;
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

    Stage *current_state = nullptr;
    {
      auto _current_state =
          std::make_unique<stages::CurrentState>("current state");

      auto applicability_filter = std::make_unique<stages::PredicateFilter>(
          "applicability test", std::move(_current_state));
      applicability_filter->setPredicate(
          [object](const SolutionBase &s, std::string &comment) {
            if (s.start()->scene()->getCurrentState().hasAttachedBody(object)) {
              comment = "object with id '" + object +
                        "' is already attached and cannot be picked";
              return false;
            }
            return true;
          });

      current_state = applicability_filter.get();
      t.add(std::move(applicability_filter));
    }

    {
      auto stage =
          std::make_unique<stages::MoveTo>("grow tall", sampling_planner);
      stage->properties().configureInitFrom(Stage::PARENT, {"torso"});
      stage->setGroup("torso");
      stage->setGoal("tall");
      stage->restrictDirection(stages::MoveTo::FORWARD);

      t.add(std::move(stage));
    }

    {
      auto stage =
          std::make_unique<stages::MoveTo>("open gripper", sampling_planner);
      stage->setGroup("gripper");
      stage->setGoal("open");
      t.add(std::move(stage));
    }

    {
      auto stage = std::make_unique<stages::Connect>(
          "move to pre-grasp pose",
          stages::Connect::GroupPlannerVector{{"arm_torso", sampling_planner}});
      stage->properties().configureInitFrom(Stage::PARENT);
      t.add(std::move(stage));
    }

    {
      auto stage = std::make_unique<stages::MoveRelative>("approach object",
                                                          cartesian_planner);
      stage->properties().set("marker_ns", "approach");
      stage->properties().set("link", "gripper_grasping_frame");
      stage->properties().configureInitFrom(Stage::PARENT, {"group"});
      stage->setMinMaxDistance(approach_object_min_dist_,
                               approach_object_max_dist_);

      geometry_msgs::Vector3Stamped vec;
      vec.header.frame_id = "gripper_grasping_frame";
      vec.vector.x = 1.0;
      stage->setDirection(vec);
      t.add(std::move(stage));
    }

    auto grasp = std::make_unique<SerialContainer>("grasp object");
    t.properties().exposeTo(grasp->properties(), {"eef", "gripper"});
    grasp->properties().configureInitFrom(Stage::PARENT, {"eef", "gripper"});

    Stage *ik_state = nullptr;
    {
      auto stage =
          std::make_unique<stages::GenerateGraspPose>("grasp work space pose");
      stage->properties().configureInitFrom(Stage::PARENT);
      stage->setPreGraspPose("open");
      stage->setObject(object);
      stage->setAngleDelta(M_PI / 12);

      stage->setMonitoredStage(current_state);

      auto wrapper =
          std::make_unique<stages::ComputeIK>("grasp pose", std::move(stage));
      ik_state = wrapper.get();
      wrapper->setMaxIKSolutions(8);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(grasp_frame_transform_, "gripper_grasping_frame");
      wrapper->properties().configureInitFrom(Stage::PARENT, {"eef"});
      wrapper->properties().configureInitFrom(Stage::INTERFACE,
                                              {"target_pose"});
      grasp->insert(std::move(wrapper));
    }

    {
      auto stage = std::make_unique<stages::ModifyPlanningScene>(
          "allow gripper->object collision");
      stage->allowCollisions(object,
                             t.getRobotModel()
                                 ->getJointModelGroup("gripper")
                                 ->getLinkModelNamesWithCollisionGeometry(),
                             true);
      grasp->insert(std::move(stage));
    }

    {
      auto stage =
          std::make_unique<stages::MoveTo>("close gripper", sampling_planner);
      stage->properties().property("group").configureInitFrom(Stage::PARENT,
                                                              "gripper");
      stage->setGoal("grasped");
      grasp->insert(std::move(stage));
    }

    {
      auto stage =
          std::make_unique<stages::ModifyPlanningScene>("attach object");
      stage->attachObject(object, "gripper_grasping_frame");
      grasp->insert(std::move(stage));
    }

    t.add(std::move(grasp));

    // TODO: serial container fails to infer correct interfaces here
    // Error initializing stages:
    // lift off table: required interface is not satisfied
    // lift off table: input interface of 'allow (object,support) collision'
    // doesn't match mine

    // auto lift = std::make_unique<SerialContainer>("lift off table");
    // t.properties().exposeTo(lift->properties(), {"group"});
    // lift->properties().configureInitFrom(Stage::PARENT, {"group"});

    {
      auto stage = std::make_unique<stages::ModifyPlanningScene>(
          "allow (object,support) collision");
      stage->allowCollisions({object}, supports_, true);
      // lift->insert(std::move(stage));
      t.add(std::move(stage));
    }

    {
      auto stage = std::make_unique<stages::MoveRelative>("lift object",
                                                          cartesian_planner);
      stage->properties().configureInitFrom(Stage::PARENT, {"group"});
      stage->setMinMaxDistance(lift_object_min_dist_, lift_object_max_dist_);
      stage->setIKFrame("gripper_grasping_frame");

      stage->properties().set("marker_ns", "lift");

      geometry_msgs::Vector3Stamped vec;
      vec.header.frame_id = "base_footprint";
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      // lift->insert(std::move(stage));
      t.add(std::move(stage));
    }

    {
      auto stage = std::make_unique<stages::ModifyPlanningScene>(
          "forbid (object,support) collision");
      stage->allowCollisions({object}, supports_, false);
      // lift->insert(std::move(stage));
      t.add(std::move(stage));
    }

    // t.add(std::move(lift));

    //{
    //	auto stage = std::make_unique<stages::MoveRelative>("retreat object",
    // cartesian_planner);
    // stage->properties().configureInitFrom(Stage::PARENT,
    //{"group"}); 	stage->setMinMaxDistance(0.01,0.10);
    //	stage->setIKFrame("gripper_grasping_frame");

    //	stage->properties().set("marker_ns", "retreat");

    //	geometry_msgs::Vector3Stamped vec;
    //	vec.header.frame_id= "base_footprint";
    //	vec.vector.x= -1.0;
    //	stage->setDirection(vec);
    //	t.add(std::move(stage));
    //}

    {
      auto stage =
          std::make_unique<stages::MoveTo>("move home", sampling_planner);
      stage->properties().configureInitFrom(Stage::PARENT, {"group"});
      // stage->setPathConstraints(upright_constraint_);
      // stage->setGoal("transport");
      stage->setGoal(transport_pose_);
      stage->restrictDirection(stages::MoveTo::FORWARD);

      t.add(filter_upright(t, std::move(stage)));
    }

    t.enableIntrospection();

    try {
      t.plan(1);
    } catch (InitStageException &e) {
      ROS_ERROR_STREAM(e);
      tiago_bartender_msgs::PickResult result;
      result.result.result =
          tiago_bartender_msgs::ManipulationResult::INTERNAL_ERROR;
      as_pick_.setAborted(result, "Initialization failed");
      return;
    }

    if (t.numSolutions() == 0) {
      tiago_bartender_msgs::PickResult result;

      if (ik_state->solutions().size() == 0) {
        result.result.result =
            tiago_bartender_msgs::ManipulationResult::UNREACHABLE;
        as_pick_.setAborted(result, "IK failed, is the object reachable?");
      } else {
        result.result.result =
            tiago_bartender_msgs::ManipulationResult::NO_PLAN_FOUND;
        as_pick_.setAborted(result, "Planning failed");
      }

      return;
    }

    moveit_task_constructor_msgs::Solution solution;
    t.solutions().front()->fillMessage(solution);

    // ROS_INFO_STREAM( "last trajectory in solution:\n" <<
    // solution.sub_trajectory.back().trajectory );

    if (execute_solutions_) {
      moveit_task_constructor_msgs::ExecuteTaskSolutionGoal execute_goal;
      execute_goal.task_solution = solution;
      execute_.sendGoal(execute_goal);
      execute_.waitForResult();
      moveit_msgs::MoveItErrorCodes execute_result =
          execute_.getResult()->error_code;

      if (execute_result.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
        ROS_ERROR_STREAM("task execution failed and returned: "
                         << execute_.getState().toString());
        tiago_bartender_msgs::PickResult result;
        result.result.result =
            tiago_bartender_msgs::ManipulationResult::EXECUTION_FAILED;
        as_pick_.setAborted(result, "Execution failed");
        return;
      }
    }

    tiago_bartender_msgs::PickResult result;
    result.result.result = tiago_bartender_msgs::ManipulationResult::SUCCESS;
    as_pick_.setSucceeded(result);
  }

  void pour_cb(const tiago_bartender_msgs::PourGoalConstPtr &goal) {
    const std::string container = goal->container_id;

    std::map<std::string, moveit_msgs::AttachedCollisionObject>
        attached_objects = psi_.getAttachedObjects();
    if (attached_objects.size() != 1) {
      ROS_ERROR_STREAM("Invalid request for PourAction - No bottle in gripper");
      tiago_bartender_msgs::PourResult result;
      result.result.result =
          tiago_bartender_msgs::ManipulationResult::INTERNAL_ERROR;
      as_pour_.setAborted(result, "Invalid request - No bottle in gripper");
      return;
    }
    const std::string bottle = attached_objects.begin()->first;

    pour_task_.reset(new moveit::task_constructor::Task("pour_to_glass"));
    Task &t = *pour_task_;
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

    Stage *current_state = nullptr;
    {
      auto stage = std::make_unique<stages::CurrentState>("current state");
      current_state = stage.get();
      t.add(std::move(stage));
    }

    /*{
      auto stage =
          std::make_unique<stages::MoveTo>("grow tall", sampling_planner);
      stage->properties().configureInitFrom(Stage::PARENT, {"torso"});
      stage->setGroup("torso");
      stage->setGoal("tall");
      stage->restrictDirection(stages::MoveTo::FORWARD);

      t.add(std::move(stage));
    }*/

    {
      auto stage = std::make_unique<stages::Connect>(
          "move to pre-pour pose",
          stages::Connect::GroupPlannerVector{{"arm_torso", sampling_planner}});
      stage->properties().configureInitFrom(Stage::PARENT);
      // t.add(std::move(stage));

      t.add(filter_upright(t, std::move(stage)));
    }

    Stage *ik_state = nullptr;
    {
      auto stage = std::make_unique<stages::GeneratePose>("pose above glass");
      geometry_msgs::PoseStamped p;
      p.header.frame_id = container;
      p.pose.orientation.w = 1;
      p.pose.position.z = pre_pour_height_;
      stage->setPose(p);
      stage->properties().configureInitFrom(Stage::PARENT);

      stage->setMonitoredStage(current_state);

      auto wrapper = std::make_unique<stages::ComputeIK>("pre-pour pose",
                                                         std::move(stage));
      wrapper->setMaxIKSolutions(32);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(grasp_frame_transform_, "gripper_grasping_frame");
      wrapper->properties().configureInitFrom(
          Stage::PARENT, {"eef"}); // TODO: convenience wrapper
      wrapper->properties().configureInitFrom(Stage::INTERFACE,
                                              {"target_pose"});
      ik_state = wrapper.get();
      t.add(std::move(wrapper));
    }

    {
      auto stage = std::make_unique<mtc_pour::PourInto>("pouring");
      stage->setBottle(bottle);
      stage->setContainer(container);
      stage->setPourOffset(pour_offset_);
      stage->setTiltAngle(pour_angle_);
      stage->setPourDuration(ros::Duration(goal->pouring_duration));
      stage->properties().configureInitFrom(Stage::PARENT);
      t.add(std::move(stage));
    }

    {
      auto stage =
          std::make_unique<stages::MoveTo>("move home", sampling_planner);
      stage->properties().configureInitFrom(Stage::PARENT, {"group"});
      // stage->setPathConstraints(upright_constraint_);
      // stage->setGoal("transport");
      stage->setGoal(transport_pose_);
      stage->restrictDirection(stages::MoveTo::FORWARD);

      t.add(filter_upright(t, std::move(stage)));
    }

    t.enableIntrospection();

    try {
      t.plan(1);
    } catch (InitStageException &e) {
      ROS_ERROR_STREAM(e);
      tiago_bartender_msgs::PourResult result;
      result.result.result =
          tiago_bartender_msgs::ManipulationResult::INTERNAL_ERROR;
      as_pour_.setAborted(result, "Initialization failed");
      return;
    }

    if (t.numSolutions() == 0) {
      tiago_bartender_msgs::PourResult result;

      if (ik_state->solutions().size() == 0) {
        result.result.result =
            tiago_bartender_msgs::ManipulationResult::UNREACHABLE;
        as_pour_.setAborted(result, "IK failed, is the countainer reachable?");
      } else {
        result.result.result =
            tiago_bartender_msgs::ManipulationResult::NO_PLAN_FOUND;
        as_pour_.setAborted(result, "Planning failed");
      }

      return;
    }

    moveit_task_constructor_msgs::Solution solution;
    t.solutions().front()->fillMessage(solution);

    ROS_INFO("ready");

    if (execute_solutions_) {
      /*moveit_task_constructor_msgs::ExecuteTaskSolutionGoal execute_goal;
      execute_goal.task_solution = solution;
      execute_.sendGoal(execute_goal);
      execute_.waitForResult();
      moveit_msgs::MoveItErrorCodes execute_result=
      execute_.getResult()->error_code;

      if(execute_result.val != moveit_msgs::MoveItErrorCodes::SUCCESS){
              ROS_ERROR_STREAM("task execution failed and returned: " <<
      execute_.getState().toString()); tiago_bartender_msgs::PourResult result;
              result.result.result =
      tiago_bartender_msgs::ManipulationResult::EXECUTION_FAILED;
              as_pour_.setAborted(result, "Execution failed");
              return;
      }*/

      // arm_controller

      ROS_INFO("execute");

      static ros::NodeHandle node;
      static ros::Publisher trajectory_publisher =
          node.advertise<trajectory_msgs::JointTrajectory>(
              "/arm_controller/command", 1, true);

      for (auto &sub_trajectory : solution.sub_trajectory) {

        auto &joint_trajectory = sub_trajectory.trajectory.joint_trajectory;
        for (auto &n : joint_trajectory.joint_names) {
          ROS_INFO("joint %s", n.c_str());
        }

        trajectory_msgs::JointTrajectory joint_trajectory_2;

        std::vector<size_t> joint_indices;
        for (auto &n : {
                 "arm_1_joint",
                 "arm_2_joint",
                 "arm_3_joint",
                 "arm_4_joint",
                 "arm_5_joint",
                 "arm_6_joint",
                 "arm_7_joint",
             }) {
          for (size_t i = 0; i < joint_trajectory.joint_names.size(); i++) {
            if (joint_trajectory.joint_names[i] == n) {
              joint_indices.push_back(i);
            }
          }
        }

        ROS_INFO("bla 1");

        if (joint_indices.empty()) {
          continue;
        }

        ROS_INFO("bla 2");

        for (auto i : joint_indices) {
          joint_trajectory_2.joint_names.emplace_back(
              joint_trajectory.joint_names[i]);
        }

        ROS_INFO("bla 3");

        joint_trajectory_2.points.resize(joint_trajectory.points.size());

        ROS_INFO("bla 4");

        for (size_t pi = 0; pi < joint_trajectory.points.size(); pi++) {

          for (auto ji : joint_indices) {

            if (!joint_trajectory.points[pi].positions.empty())
              joint_trajectory_2.points[pi].positions.push_back(
                  joint_trajectory.points[pi].positions[ji]);

            if (!joint_trajectory.points[pi].velocities.empty())
              joint_trajectory_2.points[pi].velocities.push_back(
                  joint_trajectory.points[pi].velocities[ji]);

            if (!joint_trajectory.points[pi].accelerations.empty())
              joint_trajectory_2.points[pi].accelerations.push_back(
                  joint_trajectory.points[pi].accelerations[ji]);

            if (!joint_trajectory.points[pi].effort.empty())
              joint_trajectory_2.points[pi].effort.push_back(
                  joint_trajectory.points[pi].effort[ji]);
          }

          joint_trajectory_2.points[pi].time_from_start =
              joint_trajectory.points[pi].time_from_start + ros::Duration(5.0);
        }

        ROS_INFO("bla 5");

        trajectory_publisher.publish(joint_trajectory_2);

        ROS_INFO("bla 6");

        (ros::Duration(3.0) +
         ros::Duration(joint_trajectory_2.points.back().time_from_start))
            .sleep();
      }
    }

    tiago_bartender_msgs::PourResult result;
    result.result.result = tiago_bartender_msgs::ManipulationResult::SUCCESS;
    as_pour_.setSucceeded(result);
  }

  void place_pick_cb(const tiago_bartender_msgs::PlacePickGoalConstPtr &goal) {
    std::map<std::string, moveit_msgs::AttachedCollisionObject>
        attached_objects = psi_.getAttachedObjects();
    if (attached_objects.size() != 1) {
      ROS_ERROR_STREAM("Invalid request for PlacePickAction - No bottle in "
                       "gripper to place");
      tiago_bartender_msgs::PlacePickResult result;
      result.result.result =
          tiago_bartender_msgs::ManipulationResult::INTERNAL_ERROR;
      as_place_pick_.setAborted(result,
                                "Invalid request - No bottle in gripper");
      return;
    }
    const std::string place_bottle = attached_objects.begin()->first;
    const std::string pick_bottle = goal->pick_object_id;

    place_pick_task_.reset(
        new moveit::task_constructor::Task("place_pick_object"));
    Task &t = *place_pick_task_;
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

    Stage *current_state = nullptr;
    {
      auto _current_state =
          std::make_unique<stages::CurrentState>("current state");

      current_state = _current_state.get();
      t.add(std::move(_current_state));
    }

    // PLACE

    {
      auto stage = std::make_unique<stages::Connect>(
          "move to pre-place pose",
          stages::Connect::GroupPlannerVector{{"arm_torso", sampling_planner}});
      stage->setTimeout(15.0);
      // stage->setPathConstraints(upright_constraint);
      stage->properties().configureInitFrom(Stage::PARENT);

      t.add(filter_upright(t, std::move(stage)));
    }

    {
      auto stage = std::make_unique<stages::MoveRelative>("put down object",
                                                          cartesian_planner);
      stage->properties().set("marker_ns", "approach_place");
      stage->properties().set("link", "gripper_grasping_frame");
      stage->properties().configureInitFrom(Stage::PARENT, {"group"});
      stage->setMinMaxDistance(.08, .13);

      geometry_msgs::Vector3Stamped vec;
      vec.header.frame_id = "base_footprint";
      vec.vector.z = -1.0;
      stage->setDirection(vec);
      t.add(std::move(stage));
    }

    {
      auto stage = std::make_unique<stages::GeneratePose>("place pose");
      // TODO: two relative poses in parallel
      geometry_msgs::PoseStamped p;
      p.header.frame_id = pick_bottle;
      p.pose.orientation.w = 1;
      p.pose.position.x = -0.1;
      p.pose.position.y = -0.15;
      p.pose.position.z = 0.1;
      stage->setPose(p);
      stage->properties().configureInitFrom(Stage::PARENT);

      stage->setMonitoredStage(current_state);

      auto wrapper = std::make_unique<stages::ComputeIK>(
          "place pose kinematics", std::move(stage));
      wrapper->setMaxIKSolutions(32);
      wrapper->setIKFrame(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()) *
                              Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()),
                          "gripper_grasping_frame");
      wrapper->properties().configureInitFrom(Stage::PARENT, {"eef"});
      wrapper->properties().configureInitFrom(Stage::INTERFACE,
                                              {"target_pose"});
      t.add(std::move(wrapper));
    }

    {
      auto stage =
          std::make_unique<stages::MoveTo>("release object", sampling_planner);
      stage->properties().property("group").configureInitFrom(Stage::PARENT,
                                                              "gripper");
      stage->setGoal("open");
      t.add(std::move(stage));
    }

    {
      auto stage = std::make_unique<stages::ModifyPlanningScene>(
          "allow gripper->object collision");
      stage->allowCollisions(place_bottle,
                             t.getRobotModel()
                                 ->getJointModelGroup("gripper")
                                 ->getLinkModelNamesWithCollisionGeometry(),
                             false);
      t.add(std::move(stage));
    }

    {
      auto stage =
          std::make_unique<stages::ModifyPlanningScene>("detach object");
      stage->detachObject(place_bottle, "gripper_grasping_frame");
      t.add(std::move(stage));
    }

    Stage *after_place = nullptr;
    {
      auto stage = std::make_unique<stages::MoveRelative>("retreat after place",
                                                          cartesian_planner);
      stage->properties().configureInitFrom(Stage::PARENT, {"group"});
      stage->setMinMaxDistance(.12, .25);
      stage->setIKFrame("gripper_grasping_frame");

      stage->properties().set("marker_ns", "post-place");

      geometry_msgs::Vector3Stamped vec;
      vec.header.frame_id = "gripper_grasping_frame";
      vec.vector.x = -1.0;
      stage->setDirection(vec);

      after_place = stage.get();

      t.add(std::move(stage));
    }

    // PICK

    {
      auto stage = std::make_unique<stages::Connect>(
          "move to pre-grasp pose",
          stages::Connect::GroupPlannerVector{{"arm_torso", sampling_planner}});
      stage->properties().configureInitFrom(Stage::PARENT);
      t.add(std::move(stage));
    }

    {
      auto stage = std::make_unique<stages::MoveRelative>("approach object",
                                                          cartesian_planner);
      stage->properties().set("marker_ns", "approach");
      stage->properties().set("link", "gripper_grasping_frame");
      stage->properties().configureInitFrom(Stage::PARENT, {"group"});
      stage->setMinMaxDistance(.10, .15);

      geometry_msgs::Vector3Stamped vec;
      vec.header.frame_id = "gripper_grasping_frame";
      vec.vector.x = 1.0;
      stage->setDirection(vec);
      t.add(std::move(stage));
    }

    auto grasp = std::make_unique<SerialContainer>("grasp object");
    t.properties().exposeTo(grasp->properties(), {"eef", "gripper"});
    grasp->properties().configureInitFrom(Stage::PARENT, {"eef", "gripper"});

    Stage *ik_state = nullptr;
    {
      auto stage =
          std::make_unique<stages::GenerateGraspPose>("grasp work space pose");
      stage->properties().configureInitFrom(Stage::PARENT);
      stage->setPreGraspPose("open");
      stage->setObject(pick_bottle);
      stage->setAngleDelta(M_PI / 6);

      stage->setMonitoredStage(after_place);

      auto wrapper =
          std::make_unique<stages::ComputeIK>("grasp pose", std::move(stage));
      ik_state = wrapper.get();
      wrapper->setMaxIKSolutions(8);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()) *
                              Eigen::Translation3d(0.05, 0, -.04),
                          "gripper_grasping_frame");
      wrapper->properties().configureInitFrom(Stage::PARENT, {"eef"});
      wrapper->properties().configureInitFrom(Stage::INTERFACE,
                                              {"target_pose"});
      grasp->insert(std::move(wrapper));
    }

    {
      auto stage = std::make_unique<stages::ModifyPlanningScene>(
          "allow gripper->object collision");
      stage->allowCollisions(pick_bottle,
                             t.getRobotModel()
                                 ->getJointModelGroup("gripper")
                                 ->getLinkModelNamesWithCollisionGeometry(),
                             true);
      grasp->insert(std::move(stage));
    }

    {
      auto stage =
          std::make_unique<stages::MoveTo>("close gripper", sampling_planner);
      stage->properties().property("group").configureInitFrom(Stage::PARENT,
                                                              "gripper");
      stage->setGoal("grasped");
      grasp->insert(std::move(stage));
    }

    {
      auto stage =
          std::make_unique<stages::ModifyPlanningScene>("attach object");
      stage->attachObject(pick_bottle, "gripper_grasping_frame");
      grasp->insert(std::move(stage));
    }

    t.add(std::move(grasp));

    {
      auto stage = std::make_unique<stages::ModifyPlanningScene>(
          "allow (object,support) collision");
      stage->allowCollisions({pick_bottle}, supports_, true);
      t.add(std::move(stage));
    }

    {
      auto stage = std::make_unique<stages::MoveRelative>("lift object",
                                                          cartesian_planner);
      stage->properties().configureInitFrom(Stage::PARENT, {"group"});
      stage->setMinMaxDistance(.01, .10);
      stage->setIKFrame("gripper_grasping_frame");

      stage->properties().set("marker_ns", "lift");

      geometry_msgs::Vector3Stamped vec;
      vec.header.frame_id = "base_footprint";
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      t.add(std::move(stage));
    }

    {
      auto stage = std::make_unique<stages::ModifyPlanningScene>(
          "forbid (object,support) collision");
      stage->allowCollisions({pick_bottle}, supports_, false);
      t.add(std::move(stage));
    }

    {
      auto stage =
          std::make_unique<stages::MoveTo>("move home", sampling_planner);
      stage->properties().configureInitFrom(Stage::PARENT, {"group"});
      stage->setGoal(transport_pose_);
      stage->restrictDirection(stages::MoveTo::FORWARD);

      t.add(filter_upright(t, std::move(stage)));
    }

    t.enableIntrospection();

    try {
      t.plan(1);
    } catch (InitStageException &e) {
      ROS_ERROR_STREAM(e);
      tiago_bartender_msgs::PlacePickResult result;
      result.result.result =
          tiago_bartender_msgs::ManipulationResult::INTERNAL_ERROR;
      as_place_pick_.setAborted(result, "Initialization failed");
      return;
    }

    if (t.numSolutions() == 0) {
      tiago_bartender_msgs::PlacePickResult result;

      if (ik_state->solutions().size() == 0) {
        result.result.result =
            tiago_bartender_msgs::ManipulationResult::UNREACHABLE;
        as_place_pick_.setAborted(result,
                                  "IK failed, is the object reachable?");
      } else {
        result.result.result =
            tiago_bartender_msgs::ManipulationResult::NO_PLAN_FOUND;
        as_place_pick_.setAborted(result, "Planning failed");
      }

      return;
    }

    moveit_task_constructor_msgs::Solution solution;
    t.solutions().front()->fillMessage(solution);

    // ROS_INFO_STREAM( "last trajectory in solution:\n" <<
    // solution.sub_trajectory.back().trajectory );

    if (execute_solutions_) {
      moveit_task_constructor_msgs::ExecuteTaskSolutionGoal execute_goal;
      execute_goal.task_solution = solution;
      execute_.sendGoal(execute_goal);
      execute_.waitForResult();
      moveit_msgs::MoveItErrorCodes execute_result =
          execute_.getResult()->error_code;

      if (execute_result.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
        ROS_ERROR_STREAM("task execution failed and returned: "
                         << execute_.getState().toString());
        tiago_bartender_msgs::PickResult result;
        result.result.result =
            tiago_bartender_msgs::ManipulationResult::EXECUTION_FAILED;
        as_pick_.setAborted(result, "Execution failed");
        return;
      }
    }

    tiago_bartender_msgs::PlacePickResult result;
    result.result.result = tiago_bartender_msgs::ManipulationResult::SUCCESS;
    as_place_pick_.setSucceeded(result);
  }

  void place_cb(const tiago_bartender_msgs::PlaceGoalConstPtr &goal) {
    std::map<std::string, moveit_msgs::AttachedCollisionObject>
        attached_objects = psi_.getAttachedObjects();
    if (attached_objects.size() != 1) {
      ROS_ERROR_STREAM(
          "Invalid request for PlaceAction - No bottle in gripper to place");
      tiago_bartender_msgs::PlaceResult result;
      result.result.result =
          tiago_bartender_msgs::ManipulationResult::INTERNAL_ERROR;
      as_place_.setAborted(result, "Invalid request - No bottle in gripper");
      return;
    }
    const std::string place_bottle = attached_objects.begin()->first;

    place_task_.reset(new moveit::task_constructor::Task("place_object"));
    Task &t = *place_task_;
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

    Stage *current_state = nullptr;
    {
      auto _current_state =
          std::make_unique<stages::CurrentState>("current state");

      current_state = _current_state.get();
      t.add(std::move(_current_state));
    }

    // PLACE

    {
      auto stage = std::make_unique<stages::Connect>(
          "move to pre-place pose",
          stages::Connect::GroupPlannerVector{{"arm_torso", sampling_planner}});
      stage->setTimeout(15.0);
      // stage->setPathConstraints(upright_constraint);
      stage->properties().configureInitFrom(Stage::PARENT);

      t.add(filter_upright(t, std::move(stage)));
    }

    {
      auto stage = std::make_unique<stages::MoveRelative>("put down object",
                                                          cartesian_planner);
      stage->properties().set("marker_ns", "approach_place");
      stage->properties().set("link", "gripper_grasping_frame");
      stage->properties().configureInitFrom(Stage::PARENT, {"group"});
      stage->setMinMaxDistance(.08, .13);

      geometry_msgs::Vector3Stamped vec;
      vec.header.frame_id = "base_footprint";
      vec.vector.z = -1.0;
      stage->setDirection(vec);
      t.add(std::move(stage));
    }

    {
      auto stage = std::make_unique<stages::GeneratePose>("place pose");
      geometry_msgs::PoseStamped p;
      p.header.frame_id = "base_footprint";
      p.pose.orientation.w = 1;
      p.pose.position.x =
          0.9 + grasp_frame_transform_.inverse().translation().x();
      p.pose.position.y = 0.0;
      p.pose.position.z = table_height_ + 0.5 * bottle_height_ +
                          grasp_frame_transform_.inverse().translation().z() +
                          place_surface_offset_;
      stage->setPose(p);
      stage->properties().configureInitFrom(Stage::PARENT);

      stage->setMonitoredStage(current_state);

      auto wrapper = std::make_unique<stages::ComputeIK>(
          "place pose kinematics", std::move(stage));
      wrapper->setMaxIKSolutions(32);
      wrapper->setIKFrame(grasp_frame_transform_, "gripper_grasping_frame");
      wrapper->properties().configureInitFrom(Stage::PARENT, {"eef"});
      wrapper->properties().configureInitFrom(Stage::INTERFACE,
                                              {"target_pose"});
      t.add(std::move(wrapper));
    }

    {
      auto stage =
          std::make_unique<stages::MoveTo>("release object", sampling_planner);
      stage->properties().property("group").configureInitFrom(Stage::PARENT,
                                                              "gripper");
      stage->setGoal("open");
      t.add(std::move(stage));
    }

    {
      auto stage = std::make_unique<stages::ModifyPlanningScene>(
          "allow gripper->object collision");
      stage->allowCollisions(place_bottle,
                             t.getRobotModel()
                                 ->getJointModelGroup("gripper")
                                 ->getLinkModelNamesWithCollisionGeometry(),
                             false);
      t.add(std::move(stage));
    }

    {
      auto stage =
          std::make_unique<stages::ModifyPlanningScene>("detach object");
      stage->detachObject(place_bottle, "gripper_grasping_frame");
      t.add(std::move(stage));
    }

    Stage *after_place = nullptr;
    {
      auto stage = std::make_unique<stages::MoveRelative>("retreat after place",
                                                          cartesian_planner);
      stage->properties().configureInitFrom(Stage::PARENT, {"group"});
      stage->setMinMaxDistance(.12, .25);
      stage->setIKFrame("gripper_grasping_frame");

      stage->properties().set("marker_ns", "post-place");

      geometry_msgs::Vector3Stamped vec;
      vec.header.frame_id = "gripper_grasping_frame";
      vec.vector.x = -1.0;
      stage->setDirection(vec);

      after_place = stage.get();

      t.add(std::move(stage));
    }

    {
      auto stage =
          std::make_unique<stages::MoveTo>("move home", sampling_planner);
      stage->properties().configureInitFrom(Stage::PARENT, {"group"});
      stage->setGoal(transport_pose_);
      stage->restrictDirection(stages::MoveTo::FORWARD);

      t.add(filter_upright(t, std::move(stage)));
    }

    t.enableIntrospection();

    try {
      t.plan(1);
    } catch (InitStageException &e) {
      ROS_ERROR_STREAM(e);
      tiago_bartender_msgs::PlaceResult result;
      result.result.result =
          tiago_bartender_msgs::ManipulationResult::INTERNAL_ERROR;
      as_place_.setAborted(result, "Initialization failed");
      return;
    }

    if (t.numSolutions() == 0) {
      tiago_bartender_msgs::PlaceResult result;

      result.result.result =
          tiago_bartender_msgs::ManipulationResult::NO_PLAN_FOUND;
      as_place_.setAborted(result, "Planning failed");

      return;
    }

    moveit_task_constructor_msgs::Solution solution;
    t.solutions().front()->fillMessage(solution);

    // ROS_INFO_STREAM( "last trajectory in solution:\n" <<
    // solution.sub_trajectory.back().trajectory );

    if (execute_solutions_) {
      moveit_task_constructor_msgs::ExecuteTaskSolutionGoal execute_goal;
      execute_goal.task_solution = solution;
      execute_.sendGoal(execute_goal);
      execute_.waitForResult();
      moveit_msgs::MoveItErrorCodes execute_result =
          execute_.getResult()->error_code;

      if (execute_result.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
        ROS_ERROR_STREAM("task execution failed and returned: "
                         << execute_.getState().toString());
        tiago_bartender_msgs::PickResult result;
        result.result.result =
            tiago_bartender_msgs::ManipulationResult::EXECUTION_FAILED;
        as_pick_.setAborted(result, "Execution failed");
        return;
      }
    }

    tiago_bartender_msgs::PlaceResult result;
    result.result.result = tiago_bartender_msgs::ManipulationResult::SUCCESS;
    as_place_.setSucceeded(result);
  }

private:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<tiago_bartender_msgs::PickAction> as_pick_;
  actionlib::SimpleActionServer<tiago_bartender_msgs::PourAction> as_pour_;
  actionlib::SimpleActionServer<tiago_bartender_msgs::PlacePickAction>
      as_place_pick_;
  actionlib::SimpleActionServer<tiago_bartender_msgs::PlaceAction> as_place_;

  actionlib::SimpleActionClient<
      moveit_task_constructor_msgs::ExecuteTaskSolutionAction>
      execute_;

  moveit::planning_interface::PlanningSceneInterface psi_;

  bool execute_solutions_;

  // latest tasks are retained until new ones arrive
  // to provide ROS interfaces for introspection
  moveit::task_constructor::TaskPtr pick_task_;
  moveit::task_constructor::TaskPtr pour_task_;
  moveit::task_constructor::TaskPtr place_pick_task_;
  moveit::task_constructor::TaskPtr place_task_;

  moveit_msgs::Constraints upright_constraint_;

  std::vector<std::string> supports_;

  moveit_msgs::RobotState transport_pose_;

  // ros params
  double pourer_length_;
  double bottle_height_;
  double table_height_;

  double pre_pour_height_;
  double pour_angle_;
  double approach_object_min_dist_;
  double approach_object_max_dist_;
  double lift_object_min_dist_;
  double lift_object_max_dist_;
  double place_surface_offset_;
  double place_distance_;

  Eigen::Affine3d grasp_frame_transform_;
  Eigen::Vector3d pour_offset_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "tiago_bartender_mtc_server");

  TiagoBartender tb;

  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();

  return 0;
}
