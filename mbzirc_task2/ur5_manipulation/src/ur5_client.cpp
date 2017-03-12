#include <ros/ros.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <actionlib/server/simple_action_server.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <moveit_msgs/PickupAction.h>
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
std::string task_action_name = "perform_manipulation";

typedef moveit::planning_interface::MoveGroup MoveGroup;
typedef moveit::planning_interface::MoveGroup::Plan Plan;
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajClient;

#define Z_WRENCH_PICK 1.015
#define Z_VALVE_SCAN 1.01

class TaskManager {
public:
    TaskManager(ros::NodeHandle nh) : nodeHandle(nh),
        server_task(nh, task_action_name, boost::bind(&TaskManager::execute_task, this, _1), false),
        valve_scan_finished(true){

        ac_p2p = new actionlib::SimpleActionClient<ur5_manipulation::MoveP2PAction>("move_p2p", true);
        ac_line = new actionlib::SimpleActionClient<ur5_manipulation::MoveLineAction>("move_line", true);
        ac_rotate = new actionlib::SimpleActionClient<ur5_manipulation::MoveAngleAction>("move_angle", true);

        //-----start the clients------------
        ROS_INFO("Task Action server started");

        ROS_INFO("Waiting for P2P action server to start.");
        // wait for the action server to start
        ac_p2p->waitForServer(); //will wait for infinite time

        ROS_INFO("P2P Action server started, sending goal.");

        ROS_INFO("Waiting for Line action server to start.");
        // wait for the action server to start
        ac_line->waitForServer(); //will wait for infinite time

        ROS_INFO("Line Action server started, sending goal.");

        ROS_INFO("Waiting for Rotate action server to start.");
        // wait for the action server to start
        ac_rotate->waitForServer(); //will wait for infinite time

        ROS_INFO("Rotate Action server started, sending goal.");

        tf_listener = new tf::TransformListener();
        wrench_client = nodeHandle.serviceClient<robot_perception::GetWrenchPose>("get_wrench_pose");
        valve_client = nodeHandle.serviceClient<robot_perception::GetValvePose>("get_valve_pose");
    }

    void execute_task(const moveit_msgs::PickupGoalConstPtr &goal) {

        //-----GO HOME!-------------
        geometry_msgs::Pose homePose;
        homePose.position.x = -0.27;
        homePose.position.y = 0.035;
        homePose.position.z = 1.02;
        sendP2PGoal(homePose);

        std::string s = goal->target_name;
        if(s.compare("task") == 0) {

            //Get wrench pose
            robot_perception::GetWrenchPose getPose;
            getPose.request.imageTopic = "/camera/image_rect";
            getPose.request.scanTopic = "/scan_top";
            getPose.request.wrenchNum = 2;

            wrench_client.call(getPose);
            std::cout << "pose : " << getPose.response.wrenchPose << std::endl;

            geometry_msgs::Pose pWrench = getPose.response.wrenchPose;
            if(pWrench.position.x == 0 &&
                    pWrench.position.y == 0 &&
                    pWrench.position.z == 0) {

                std::cerr << "UR5 client : Invalid wrench pose" << std::endl;
                server_task.setAborted();
                return;
            }

            //transform wrench approach in base_link frame
            geometry_msgs::Pose pAWrench = pWrench;
            pAWrench.position.z -= 0.1;
            geometry_msgs::Pose uAWrench = getPoseInBaseLinkFrame(pAWrench);
            uAWrench.position.z = Z_WRENCH_PICK;

            //transform wrench reach in base_link frame
            geometry_msgs::Pose pRWrench = pWrench;
            pRWrench.position.z -= 0.05;
            geometry_msgs::Pose uRWrench = getPoseInBaseLinkFrame(pRWrench);
            uRWrench.position.z = Z_WRENCH_PICK;

            //go to approach pose
            sendP2PGoal(uAWrench);

            //pick
            sendLineGoal(uRWrench, 0.005, true);
            sendLineGoal(uAWrench, 0.005, true);

            sendP2PGoal(homePose);

            //valve scan
            geometry_msgs::Pose pValve;
            pValve = getPose.response.valvePose;
            if(pValve.position.x == 0 &&
                    pValve.position.y == 0 &&
                    pValve.position.z == 0) {

                std::cerr << "UR5 client : Invalid valve pose" << std::endl;
                //server_task.setAborted();
                server_task.setAborted(); // delete this!
                return;
            }

            //transform valve scan start in base_link frame
            geometry_msgs::Pose pSValve = pValve;
            pSValve.position.x += 0.1;
            pSValve.position.y += 0.04;
            geometry_msgs::Pose uSValve = getPoseInBaseLinkFrame(pSValve);
            uSValve.position.z = Z_VALVE_SCAN;

            //transform valve scan end in base_link frame
            geometry_msgs::Pose pEValve = pValve;
            pEValve.position.x += 0.1;
            pEValve.position.y -= 0.04;
            geometry_msgs::Pose uEValve = getPoseInBaseLinkFrame(pEValve);
            uEValve.position.z = Z_VALVE_SCAN;

            //go to valve scan start
            sendP2PGoal(uSValve);

            //scan valve
            sendLineGoal(uEValve, 0.0001, false);

            odmini_sub = nodeHandle.subscribe<std_msgs::Float64>("/od_mini", 1, boost::bind(&TaskManager::odmini_cb, this, _1));
            ros::Subscriber traj_result_sub = nodeHandle.subscribe<control_msgs::FollowJointTrajectoryActionResult>("/follow_joint_trajectory/result", 1, boost::bind(&TaskManager::trajResultCB, this, _1));

            valve_scan.clear();

            valve_scan_finished = false;
            while(1) {
                if(valve_scan_finished)
                    break;
                ros::spinOnce();
            }

            //call P2P, line and rotate
            geometry_msgs::Pose uAValve, uRValve;
            getValvePose(uAValve, uRValve);

            //go to valve insert approach --- angle should be included in the pose
            sendP2PGoal(uAValve);
            //insert wrench --- śhould be the same pose
            sendLineGoal(uRValve, 0.005, false);
            //rotate!! --- rotate in the opposite dir of 'angle'
            sendRotateGoal(2 * M_PI);
            server_task.setSucceeded();
        }
    }

private:
    ros::NodeHandle nodeHandle;
    bool valve_scan_finished;

    actionlib::SimpleActionClient<ur5_manipulation::MoveP2PAction> *ac_p2p;//("move_p2p", true);
    actionlib::SimpleActionClient<ur5_manipulation::MoveLineAction> *ac_line;//("move_line", true);
    actionlib::SimpleActionClient<ur5_manipulation::MoveAngleAction> *ac_rotate;

    actionlib::SimpleActionServer<moveit_msgs::PickupAction> server_task;
    moveit_msgs::PickupFeedback fb_task;
    moveit_msgs::PickupResult res_task;

    ros::ServiceClient wrench_client;
    ros::ServiceClient valve_client;

    ros::Subscriber odmini_sub;

    tf::StampedTransform transform;
    tf::TransformListener *tf_listener;

    std::vector<pcl::PointXY> valve_scan;

    bool sendLineGoal(geometry_msgs::Pose pose, double vel_scale, bool wait) {
        ur5_manipulation::MoveLineGoal goal;
        goal.endPose = pose;
        goal.velScaleFactor = vel_scale;

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
        bool finished_before_timeout = ac_p2p->waitForResult(ros::Duration(30.0));

        if (finished_before_timeout)
        {
            actionlib::SimpleClientGoalState state = ac_p2p->getState();
            ROS_INFO("Action finished: %s",state.toString().c_str());
        }
        else
            ROS_INFO("Action did not finish before the time out.");

        return finished_before_timeout;
    }

    bool sendRotateGoal(double angle) {
        ur5_manipulation::MoveAngleGoal goal;
        goal.angle = angle;
        ac_rotate->sendGoal(goal);

        //wait for the action to return
        bool finished_before_timeout = ac_rotate->waitForResult(ros::Duration(30.0));

        if (finished_before_timeout)
        {
            actionlib::SimpleClientGoalState state = ac_rotate->getState();
            ROS_INFO("Action finished: %s",state.toString().c_str());
        }
        else
            ROS_INFO("Action did not finish before the time out.");

        return finished_before_timeout;
    }

    void odmini_cb(std_msgs::Float64ConstPtr dist) {
        std::cout << "dist : " << dist->data << std::endl;

        try{
            //pick
            tf_listener->waitForTransform("/ur5_base_link", "odmini_link", ros::Time(0), ros::Duration(10.0));
            tf_listener->lookupTransform("/ur5_base_link", "odmini_link", ros::Time(0), transform);


            pcl::PointXY p;
            p.x = transform.getOrigin().y(); //is it really
            p.y = dist->data;

            valve_scan.push_back(p);
            //ROS_INFO_STREAM(transform.getOrigin().y() << " " << dist.data);
            std::cout << p << std::endl;

        }catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }

    }

    geometry_msgs::Pose getValvePose(geometry_msgs::Pose &approach, geometry_msgs::Pose &reach) {
        pcl::PointCloud<pcl::PointXY>::Ptr cloud(new pcl::PointCloud<pcl::PointXY>);

        cloud->width = valve_scan.size();
        cloud->height = 1;
        cloud->resize(valve_scan.size());

        for (int i = 0; i < valve_scan.size(); ++i) {
            cloud->points[i].x = valve_scan[i].x;
            cloud->points[i].y = valve_scan[i].y;
        }

        std::cout << "cloud size : " << cloud->size() << std::endl;

        //Call the valve detection service
        sensor_msgs::PointCloud2 cloudMsg;
        cloudMsg.header.stamp = ros::Time::now();
        cloudMsg.header.frame_id = "odmini_link";
        pcl::toROSMsg(*cloud, cloudMsg);

        robot_perception::GetValvePose valvePose;
        valvePose.request.valveScan = cloudMsg;
        valve_client.call(valvePose);

        //assuming that no adjustments / transformations are needed --- make these changes later
        approach = valvePose.response.valveApproach;
        reach = valvePose.response.valveReach;
        double angle = valvePose.response.angle;   //Instead, include this angle in the pose above after calculations
    }

    void trajResultCB(const control_msgs::FollowJointTrajectoryActionResultConstPtr &result) {
        //Stop the odmini sub once the scanning is complete!
        odmini_sub.shutdown();

        valve_scan_finished = true;
    }

    geometry_msgs::Pose getPoseInBaseLinkFrame(geometry_msgs::Pose p) {
        tf::Stamped<tf::Pose> p1, p2;

        p1.frame_id_ = "panel";
        p1.setOrigin(tf::Vector3(p.position.x,
                                 p.position.y,
                                 p.position.z));
        std::cout << "panel point : " << p1.getOrigin().x() << " " << p1.getOrigin().y() << " " << p1.getOrigin().z() << std::endl;

        tf::Quaternion q;
        q.setRPY(0, 0, 0);
        p1.setRotation(q);

        tf_listener->transformPose("ur5_base_link", p1, p2);
        std::cout << "wrench in base_link frame : " << p2.getOrigin().x() << " " <<
                     p2.getOrigin().y() << " " <<
                     p2.getOrigin().z() << std::endl;

        geometry_msgs::Pose pose;
        pose.position.x = p2.getOrigin().x();
        pose.position.y = p2.getOrigin().y();
        pose.position.z = p2.getOrigin().z();
    }

};

int main (int argc, char **argv) {
    ros::init(argc, argv, "test_fibonacci");
    ros::NodeHandle nh;

    TaskManager taskManager(nh);


    ros::spin();
    return 0;
}

