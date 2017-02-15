#include <ros/ros.h>

#include <moveit/move_group_interface/move_group.h>
//#include <moveit_msgs/PlanningScene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
//#include <moveit_msgs/planning_scene/planning_scene.h>

int main (int argc, char **argv)
{
	ros::init(argc, argv, "move_demo");

	ros::AsyncSpinner spinner(1);
   	spinner.start();

	moveit::planning_interface::MoveGroup group("manipulator");
    	//group.setPlannerId("KPIECEKConfigDefault");
        //group.setPlannerId("RRTConnectkConfigDefault");
    	group.allowReplanning(true);
	group.setNumPlanningAttempts(5);
    	group.setMaxVelocityScalingFactor(0.1);
	group.setPlanningTime(5.0);

    /*
	group.setStartStateToCurrentState();
	group.setNamedTarget("up");
	group.move();
    */

	while(ros::ok())
    {
		group.setStartStateToCurrentState();
		group.setNamedTarget("left");
		group.move();



		group.setStartStateToCurrentState();
		group.setNamedTarget("right");
		group.move();

	}
}

