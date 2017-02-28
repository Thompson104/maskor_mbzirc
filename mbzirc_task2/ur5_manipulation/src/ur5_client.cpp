#include <ros/ros.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <actionlib/server/simple_action_server.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <ur5_manipulation/MoveAngleAction.h>
#include <ur5_manipulation/MoveLineAction.h>
#include <ur5_manipulation/MoveP2PAction.h>

#include <robot_perception/GetWrenchPose.h>

#include <std_msgs/Int8.h>

std::string move_line_action_name = "move_line";
std::string move_p2p_action_name = "move_p2p";
std::string move_angle_action_name = "move_angle";

typedef moveit::planning_interface::MoveGroup MoveGroup;
typedef moveit::planning_interface::MoveGroup::Plan Plan;
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajClient;

int main (int argc, char **argv) {
    ros::init(argc, argv, "test_fibonacci");
    ros::NodeHandle nh;

    //-----start the clients------------
    actionlib::SimpleActionClient<ur5_manipulation::MoveP2PAction> ac_p2p("move_p2p", true);

    ROS_INFO("Waiting for action server to start.");
    // wait for the action server to start
    ac_p2p.waitForServer(); //will wait for infinite time

    ROS_INFO("Action server started, sending goal.");

    ros::ServiceClient wrench_client = nh.serviceClient<robot_perception::GetWrenchPose>("get_wrench_pose");

    actionlib::SimpleActionClient<ur5_manipulation::MoveLineAction> ac_line("move_line", true);

    ROS_INFO("Waiting for action server to start.");
    // wait for the action server to start
    ac_line.waitForServer(); //will wait for infinite time

    ROS_INFO("Action server started, sending goal.");

    /*
    //--------HOME POS------

    ur5_manipulation::MoveP2PGoal goal_p2p;
    geometry_msgs::Pose homePose;
    homePose.position.x = -0.27;
    homePose.position.y = 0.035;
    homePose.position.z = 1.02;
    goal_p2p.endPose = homePose;
    ac_p2p.sendGoal(goal_p2p);

    //wait for the action to return
    bool finished_before_timeout = ac_p2p.waitForResult(ros::Duration(30.0));

    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = ac_p2p.getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
    }
    else
        ROS_INFO("Action did not finish before the time out.");



    //get the wrench pose

    robot_perception::GetWrenchPose getPose;
    getPose.request.imageTopic = "/camera/image_rect";
    getPose.request.scanTopic = "/scan";
    getPose.request.wrenchNum = 2;

    wrench_client.call(getPose);
    std::cout << "pose : " << getPose.response.wrenchPose << std::endl;

     geometry_msgs::Pose pose = getPose.response.wrenchPose;

    //-----------PICK APPROACH---------------------------
    // create the action client
    // true causes the client to spin its own thread

    goal_p2p.endPose = pose;
    ac_p2p.sendGoal(goal_p2p);

    //wait for the action to return
    finished_before_timeout = ac_p2p.waitForResult(ros::Duration(30.0));

    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = ac_p2p.getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
    }
    else
        ROS_INFO("Action did not finish before the time out.");



    //---------PICK-----------------------


    ur5_manipulation::MoveLineGoal goal_line;
    pose.position.x -= 0.08;
    goal_line.endPose = pose;
    ac_line.sendGoal(goal_line);

    //wait for the action to return
    finished_before_timeout = ac_line.waitForResult(ros::Duration(30.0));

    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = ac_line.getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
    }
    else
        ROS_INFO("Action did not finish before the time out.");

    pose.position.x += 0.08;
    goal_line.endPose = pose;
    ac_line.sendGoal(goal_line);

    //wait for the action to return
    finished_before_timeout = ac_line.waitForResult(ros::Duration(30.0));

    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = ac_line.getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
    }
    else
        ROS_INFO("Action did not finish before the time out.");

    //-------GO HOME-----------------
    goal_p2p.endPose = homePose;
    ac_p2p.sendGoal(goal_p2p);

    //wait for the action to return
    finished_before_timeout = ac_p2p.waitForResult(ros::Duration(30.0));

    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = ac_p2p.getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
    }
    else
        ROS_INFO("Action did not finish before the time out.");

    */
    //exit

    ur5_manipulation::MoveLineGoal goal_line;
    geometry_msgs::Pose pose;
    pose.position.x = -0.49;
    pose.position.y = -0.24;
    pose.position.z = 1.01;

    goal_line.endPose = pose;
    ac_line.sendGoal(goal_line);

    //wait for the action to return
    bool finished_before_timeout = ac_line.waitForResult(ros::Duration(30.0));

    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = ac_line.getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
    }
    else
        ROS_INFO("Action did not finish before the time out.");



    return 0;
}

