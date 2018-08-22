#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <sensor_msgs/JointState.h>

class TiagoGripperController
{
public:
  TiagoGripperController() 
  {
    joint_states_sub_ = nh_.subscribe("joint_states", 1000, &TiagoGripperController::joint_states_cb, this);

    as_ = boost::make_shared<actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>>(nh_, "gripper_grasp_controller/follow_joint_trajectory", boost::bind(&TiagoGripperController::goal_cb, this, _1), false);
    ac_ = boost::make_shared<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>>("gripper_controller/follow_joint_trajectory", true);
    ac_->waitForServer();
    as_->start();
  }
private:
  void joint_states_cb(const sensor_msgs::JointStateConstPtr& msg)
  {
    for(size_t i = 0; i < msg->name.size(); i++)
    {
      if(msg->name[i] == "gripper_left_finger_joint")
      {
        finger_joint_effort_ = msg->effort[i];
        break;
      }
    }
  }

  void goal_cb(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal)
  {
    double direction = 0.0;
    for(size_t i = 0; i < goal->trajectory.joint_names.size(); ++i)
    {
      if(goal->trajectory.joint_names[i] == "gripper_left_finger_joint")
      {
        double start = goal->trajectory.points.begin()->positions[i];
        double end = goal->trajectory.points.back().positions[i];
        direction = end - start;
        break;
      }
    }

    ac_->sendGoal(*goal);
    control_msgs::FollowJointTrajectoryResult result;
    if(direction >= 0.0)
    {
      ac_->waitForResult();
      result = *ac_->getResult();
    }
    else
    {
      while(true)
      {
        if(finger_joint_effort_ < -15.0)
        {
          result.error_string = "stalled gripper detected";
          result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
          break;
        }
        if(ac_->getState().isDone())
        {
          result = *ac_->getResult();
          break;
        }
        ros::Duration(0.01).sleep();
      }
    }
    as_->setSucceeded(result);
  }

  ros::NodeHandle nh_;
  boost::shared_ptr<actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>> as_;
  boost::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> ac_;

  ros::Subscriber joint_states_sub_;
  double finger_joint_effort_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tiago_gripper_controller");
  TiagoGripperController tgc;
  ros::spin();
}
