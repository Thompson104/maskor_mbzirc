#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float64.h>

#include <moveit/trajectory_processing/iterative_time_parameterization.h>

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajClient;
typedef moveit::planning_interface::MoveGroup MoveGroup;
typedef moveit::planning_interface::MoveGroup::Plan Plan;

MoveGroup *group;

double a, b;

bool goToPose(const std::string &marker_frame) {
    geometry_msgs::Pose approach;
    Eigen::Quaterniond quat = Eigen::AngleAxis<double>(double(-M_PI), Eigen::Vector3d::UnitY())
                            * Eigen::AngleAxis<double>(double(a), Eigen::Vector3d::UnitZ())
                            * Eigen::AngleAxis<double>(double(b), Eigen::Vector3d::UnitX());
    approach.orientation.w = quat.w();
    approach.orientation.x = quat.x();
    approach.orientation.y = quat.y();
    approach.orientation.z = quat.z();

    approach.position.x = -0.27;
    approach.position.y = 0.035;
    approach.position.z = 1.02;

    std::cout << "approach " << approach.position << std::endl;

    Plan traj_plan;
    group->setPoseTarget(approach);
    group->setStartStateToCurrentState();
    bool success = group->plan(traj_plan);
    ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
    if(success) {
        group->execute(traj_plan);
        ROS_INFO("Reached approach");
        ros::Duration(2.0).sleep();
        return true;
    }
    return false;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pick_task");
    group = new MoveGroup("ur5_arm");
    ros::Duration(5.0).sleep();
    group->setPlanningTime(30.0);
    group->setPlannerId("KPIECEkConfigDefault");

    // start a background "spinner", so our node can process ROS messages
    //  - this lets us know when the move is completed
    ros::AsyncSpinner spinner(1);
    spinner.start();

    sscanf(argv[1],"%lf",&a);
    sscanf(argv[2],"%lf",&b);

    std::string marker2_frame = "/ar_marker_2";
    std::string marker0_frame = "/ar_marker_0";
    if(goToPose(marker2_frame)) {
        std::cout << "success!" << std::endl;
    }

    ros::shutdown();
    return 0;
}
