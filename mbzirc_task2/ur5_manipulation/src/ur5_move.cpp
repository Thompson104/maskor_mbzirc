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

#include <std_msgs/Int8.h>

std::string move_line_action_name = "move_line";
std::string move_p2p_action_name = "move_p2p";
std::string move_angle_action_name = "move_angle";

typedef moveit::planning_interface::MoveGroup MoveGroup;
typedef moveit::planning_interface::MoveGroup::Plan Plan;
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajClient;

class UR5_Interface {
public:
    UR5_Interface(ros::NodeHandle nh) : nodeHandle(nh),
        server_line(nh, move_line_action_name, boost::bind(&UR5_Interface::execute_line, this, _1), false),
        server_p2p(nh, move_p2p_action_name, boost::bind(&UR5_Interface::execute_p2p, this, _1), false),
        server_angle(nh, move_angle_action_name, boost::bind(&UR5_Interface::execute_angle, this, _1), false){

        server_line.start();
        server_p2p.start();
        server_angle.start();

        std::cout << "move_p2p is active? : " << server_p2p.isActive() << std::endl;
        group = new MoveGroup("ur5_arm");
        group->setPlanningTime(30.0);
        group->setPlannerId("RRTConnectkConfigDefault");

        group->allowReplanning(true);
        group->setNumPlanningAttempts(5);
        group->setMaxVelocityScalingFactor(0.1);

        traj_client = new TrajClient("/arm_controller/follow_joint_trajectory", true); //Needs the namespace 'arm_controller'?

        arm_goal.trajectory.joint_names.push_back("ur5_arm_shoulder_pan_joint");
        arm_goal.trajectory.joint_names.push_back("ur5_arm_shoulder_lift_joint");
        arm_goal.trajectory.joint_names.push_back("ur5_arm_elbow_joint");
        arm_goal.trajectory.joint_names.push_back("ur5_arm_wrist_1_joint");
        arm_goal.trajectory.joint_names.push_back("ur5_arm_wrist_2_joint");
        arm_goal.trajectory.joint_names.push_back("ur5_arm_wrist_3_joint");
    }

    void execute_line(const ur5_manipulation::MoveLineGoalConstPtr &goal) {
        std_msgs::Int8 status;
        //geometry_msgs::Pose endPose = goal->endPose;

        std::vector<geometry_msgs::Pose> waypoints;
        geometry_msgs::Pose startPose = group->getCurrentPose().pose;
        waypoints.push_back(startPose);
        geometry_msgs::Pose endPose = startPose;
        endPose.position.z -= 0.05;
        waypoints.push_back(endPose);

        if(executeTrajectory(waypoints)) {
            //TODO: do the feedback loop here
            std::cout << "move complete!" << std::endl;

            //status.data = res_line.status.SUCCEEDED;
            //res_line.result.success = status;
            //server_line.setSucceeded(res_line);
        } else {
            //status.data = res_line.status.REJECTED;
            //res_line.result.success = status;
            //server_line.setAborted(res_line);
        }

    }

    void execute_p2p(const ur5_manipulation::MoveP2PGoalConstPtr &goal) {
        std_msgs::Int8 status;
        geometry_msgs::Pose endPose = goal->endPose;

        Plan traj_plan;
        group->setPoseTarget(endPose);
        group->setStartStateToCurrentState();
        bool success = group->plan(traj_plan);
        ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
        if(success) {
            //TODO: do the feedback loop here, but not absolutely necessary
            group->execute(traj_plan);
            //status.data = res_p2p.status.SUCCEEDED;
            //res_p2p.result.success = status;
            server_p2p.setSucceeded(res_p2p);
        } else {
            //status.data = res_p2p.status.REJECTED;
            //res_p2p.result.success = status;
            //server_p2p.setAborted(res_p2p);
        }
    }

    void execute_angle(const ur5_manipulation::MoveAngleGoalConstPtr &goal) {
        std::vector<double> joint_values;
        group->getCurrentState()->copyJointGroupPositions(
                    group->getCurrentState()->getRobotModel()->
                    getJointModelGroup(group->getName()), joint_values);

        float angle = 0.0;

        arm_goal.trajectory.points.clear();
        arm_goal.trajectory.points.resize(2);
        arm_goal.trajectory.points[0].positions.resize(6);
        arm_goal.trajectory.points[0].positions = joint_values;

        // Velocities
        arm_goal.trajectory.points[0].velocities.resize(6);
        for (size_t j = 0; j < 6; ++j){
            arm_goal.trajectory.points[0].velocities[j] = 0.0;
        }
        // To be reached 1 second after starting along the trajectory
        arm_goal.trajectory.points[0].time_from_start = ros::Duration(1.0);

        arm_goal.trajectory.points[1].positions.resize(6);
        std::cout << "angle : " << joint_values[5] << " " << angle  << std::endl;
        joint_values[5] += angle;

        arm_goal.trajectory.points[1].positions = joint_values;

        // Velocities
        arm_goal.trajectory.points[1].velocities.resize(6);
        for (size_t j = 0; j < 6; ++j){
            arm_goal.trajectory.points[1].velocities[j] = 0.0;
        }
        // To be reached 1 second after starting along the trajectory
        arm_goal.trajectory.points[1].time_from_start = ros::Duration(2.0);
        arm_goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(0.2);


        traj_client->sendGoal(arm_goal);
        traj_client->waitForResult(ros::Duration(10.0));
        if(traj_client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
            ROS_INFO("Goal reached!");
            ROS_INFO("Task completed!");
        }
    }

private:
    ros::NodeHandle nodeHandle;

    actionlib::SimpleActionServer<ur5_manipulation::MoveLineAction> server_line; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
    ur5_manipulation::MoveLineActionFeedback fb_line;
    ur5_manipulation::MoveLineResult res_line;

    actionlib::SimpleActionServer<ur5_manipulation::MoveP2PAction> server_p2p;
    ur5_manipulation::MoveP2PActionFeedback fb_p2p;
    ur5_manipulation::MoveP2PResult res_p2p;

    actionlib::SimpleActionServer<ur5_manipulation::MoveAngleAction> server_angle;
    ur5_manipulation::MoveAngleActionFeedback fb_angle;
    ur5_manipulation::MoveAngleResult res_angle;

    MoveGroup *group;
    TrajClient* traj_client;
    control_msgs::FollowJointTrajectoryGoal arm_goal;

    bool executeTrajectory(std::vector<geometry_msgs::Pose> waypoints) {
        group->setStartStateToCurrentState();
        moveit_msgs::RobotTrajectory trajectory;

        double fraction = group->computeCartesianPath(waypoints,
                                                      0.01,  // eef_step
                                                      0.0,   // jump_threshold
                                                      trajectory);
        ROS_INFO("Visualizing plan (cartesian path) (%.2f%% acheived)",
                 fraction * 100.0);

        // The trajectory needs to be modified so it will include velocities as well.
        // First to create a RobotTrajectory object
        robot_trajectory::RobotTrajectory rt(group->getCurrentState()->getRobotModel(), "ur5_arm");

        // Second get a RobotTrajectory from trajectory
        rt.setRobotTrajectoryMsg(*group->getCurrentState(), trajectory);

        // Thrid create a IterativeParabolicTimeParameterization object
        trajectory_processing::IterativeParabolicTimeParameterization iptp;
        // Fourth compute computeTimeStamps
        bool success = iptp.computeTimeStamps(rt, 0.01);
        ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");
        // Get RobotTrajectory_msg from RobotTrajectory
        rt.getRobotTrajectoryMsg(trajectory);

        Plan traj_plan;
        traj_plan.trajectory_ = trajectory;
        if(fraction > 0.95){
            group->asyncExecute(traj_plan);
            return true;
        }
        return false;
    }
};


int main (int argc, char **argv) {
    ros::init(argc, argv, "ur5_move");
    ros::NodeHandle nh;

    UR5_Interface ur5_interface(nh);

    ros::spin();
    return 0;
}
