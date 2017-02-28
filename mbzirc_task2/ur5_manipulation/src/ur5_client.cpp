#include <ros/ros.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <actionlib/server/simple_action_server.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>

#include <ur5_manipulation/MoveAngleAction.h>
#include <ur5_manipulation/MoveLineAction.h>
#include <ur5_manipulation/MoveP2PAction.h>

#include <robot_perception/GetWrenchPose.h>
#include <robot_perception/GetValvePose.h>

#include <std_msgs/Int8.h>
#include <std_msgs/Float64.h>

std::string move_line_action_name = "move_line";
std::string move_p2p_action_name = "move_p2p";
std::string move_angle_action_name = "move_angle";

typedef moveit::planning_interface::MoveGroup MoveGroup;
typedef moveit::planning_interface::MoveGroup::Plan Plan;
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajClient;

actionlib::SimpleActionClient<ur5_manipulation::MoveP2PAction> *ac_p2p;//("move_p2p", true);
actionlib::SimpleActionClient<ur5_manipulation::MoveLineAction> *ac_line;//("move_line", true);
ros::ServiceClient valve_client;

ros::Subscriber odmini_sub;

tf::StampedTransform transform;
tf::TransformListener *tf_listener;

std::vector<pcl::PointXY> valve_scan;

bool sendLineGoal(geometry_msgs::Pose pose, bool wait) {
    ur5_manipulation::MoveLineGoal goal;
    goal.endPose = pose;
    ac_line->sendGoal(goal);

    if(!wait)
        return true;

    //wait for the action to return
    bool finished_before_timeout = ac_line->waitForResult(ros::Duration(30.0));

    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = ac_line->getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
    }
    else
        ROS_INFO("Action did not finish before the time out.");

    return finished_before_timeout;
}

bool sendP2PGoal(geometry_msgs::Pose pose) {
    ur5_manipulation::MoveP2PGoal goal;
    goal.endPose = pose;
    ac_p2p->sendGoal(goal);

    //wait for the action to return
    bool finished_before_timeout = ac_line->waitForResult(ros::Duration(30.0));

    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = ac_p2p->getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
    }
    else
        ROS_INFO("Action did not finish before the time out.");

    return finished_before_timeout;
}

void odmini_cb(std_msgs::Float64 dist) {
    std::cout << "dist : " << dist.data << std::endl;

    try{
        //pick
        tf_listener->waitForTransform("/base_link", "odmini_link", ros::Time(0), ros::Duration(10.0));
        tf_listener->lookupTransform("/base_link", "odmini_link", ros::Time(0), transform);


        pcl::PointXY p;
        p.x = transform.getOrigin().y(); //is it really
        p.y = dist.data;

        valve_scan.push_back(p);
        //ROS_INFO_STREAM(transform.getOrigin().y() << " " << dist.data);
        std::cout << p << std::endl;

    }catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

}

void trajResultCB(const control_msgs::FollowJointTrajectoryActionResultConstPtr &result) {
    //Stop the odmini sub once the scanning is complete!
    odmini_sub.shutdown();

    //--- not sure if this is the right place---
    //Form the point cloud

    pcl::PointCloud<pcl::PointXY>::Ptr cloud(new pcl::PointCloud<pcl::PointXY>);

    cloud->width = valve_scan.size();
    cloud->height = 1;
    cloud->resize(valve_scan.size());

    for (int i = 0; i < valve_scan.size(); ++i) {
        cloud->points[i].x = valve_scan[i].x;
        cloud->points[i].y = valve_scan[i].y;
    }

    //Call the valve detection service
    sensor_msgs::PointCloud2 cloudMsg;
    cloudMsg.header.stamp = ros::Time::now();
    cloudMsg.header.frame_id = "odmini_link";
    pcl::toROSMsg(*cloud, cloudMsg);

    robot_perception::GetValvePose valvePose;
    valvePose.request.valveScan = cloudMsg;
    valve_client.call(valvePose);

    //valvePose.response.valvePose
}

int main (int argc, char **argv) {
    ros::init(argc, argv, "test_fibonacci");
    ros::NodeHandle nh;

    ac_p2p = new actionlib::SimpleActionClient<ur5_manipulation::MoveP2PAction>("move_p2p", true);
    ac_line = new actionlib::SimpleActionClient<ur5_manipulation::MoveLineAction>("move_line", true);


    //-----start the clients------------
    ROS_INFO("Waiting for action server to start.");
    // wait for the action server to start
    ac_p2p->waitForServer(); //will wait for infinite time

    ROS_INFO("Action server started, sending goal.");

    ROS_INFO("Waiting for action server to start.");
    // wait for the action server to start
    ac_line->waitForServer(); //will wait for infinite time

    ROS_INFO("Action server started, sending goal.");

    tf_listener = new tf::TransformListener();
    ros::ServiceClient wrench_client = nh.serviceClient<robot_perception::GetWrenchPose>("get_wrench_pose");
    valve_client = nh.serviceClient<robot_perception::GetValvePose>("get_valve_pose");

    /*
    //-----GO HOME!-------------
    geometry_msgs::Pose homePose;
    homePose.position.x = -0.27;
    homePose.position.y = 0.035;
    homePose.position.z = 1.02;
    sendP2PGoal(homePose);

    //Get wrench pose
    robot_perception::GetWrenchPose getPose;
    getPose.request.imageTopic = "/camera/image_rect";
    getPose.request.scanTopic = "/scan";
    getPose.request.wrenchNum = 2;

    wrench_client.call(getPose);
    std::cout << "pose : " << getPose.response.wrenchPose << std::endl;

    geometry_msgs::Pose pose = getPose.response.wrenchPose;
    if(pose.position.x == 0 &&
            pose.position.y == 0 &&
            pose.position.z == 0) {

        std::cerr << "UR5 client : Invalid wrench pose" << std::endl;
        return -1;
    }

    //go to approach pose
    sendP2PGoal(pose);

    //pick
    pose.position.x -= 0.08;
    sendLineGoal(pose, true);
    pose.position.x += 0.08;
    sendLineGoal(pose, true);

    sendP2PGoal(homePose);

    //get valve pose -- as service
    */



    geometry_msgs::Pose pose;
    pose.position.x = -0.49;
    pose.position.y = -0.36;
    pose.position.z = 1.0;

    sendLineGoal(pose, false);
    odmini_sub = nh.subscribe<std_msgs::Float64>("/od_mini", 1, odmini_cb);
    ros::Subscriber traj_result_sub = nh.subscribe<control_msgs::FollowJointTrajectoryActionResult>("/follow_joint_trajectory/result", 1, trajResultCB);

    valve_scan.clear();

    ros::spin();
    return 0;
}

