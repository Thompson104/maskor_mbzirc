#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <iostream>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>

#include <robot_perception/GetPanelOrigin.h>

class PanelDetector {
public:
    PanelDetector(ros::NodeHandle nh) : nodeHandle(nh) {
        service = nodeHandle.advertiseService("get_panel_origin", &PanelDetector::getPanelOrigin, this);
    }

    double dist(geometry_msgs::Point p1, geometry_msgs::Point p2) {
        double d = pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2);
        return sqrt(d);
    }

    bool isLineValid(geometry_msgs::Point p1, geometry_msgs::Point p2) {
        double d = dist(p1, p2);
        if(d > 0.9 && d < 1.1) {
            return true;
        }
        return false;
    }

    bool getPanelLine(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, geometry_msgs::Point &p1, geometry_msgs::Point &p2) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_line(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final(new pcl::PointCloud<pcl::PointXYZ>);

        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        // Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        // Optional
        seg.setOptimizeCoefficients (true);
        // Mandatory
        seg.setModelType (pcl::SACMODEL_LINE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setDistanceThreshold (0.01);

        seg.setInputCloud (cloud);
        seg.segment (*inliers, *coefficients);

        if (inliers->indices.size () == 0) {
            PCL_ERROR ("Could not estimate a planar model for the given dataset.");
            return false;
        }

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud (cloud);
        extract.setIndices (inliers);
        extract.setNegative (false);

        // Get the points associated with the line
        extract.filter(*cloud_line);

        //remove sparse points
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(cloud_line);
        sor.setMeanK(50);
        sor.setStddevMulThresh(1.0);
        sor.filter(*cloud_final);

        p1.x = cloud_final->at(0).x;
        p1.y = cloud_final->at(0).y;
        p1.z = cloud_final->at(0).z;

        p2.x = cloud_final->at(cloud_final->size() - 1).x;
        p2.y = cloud_final->at(cloud_final->size() - 1).y;
        p2.z = cloud_final->at(cloud_final->size() - 1).z;

        return true;
    }

    bool getPanelOrigin(robot_perception::GetPanelOrigin::Request  &req,
                        robot_perception::GetPanelOrigin::Response &res) {

        geometry_msgs::Point p1, p2;
        tf::Transform transform;

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

        //get scan data from TIM
        std::string topic = req.scanTopic;
        sensor_msgs::LaserScanConstPtr msgScan = ros::topic::waitForMessage<sensor_msgs::LaserScan>(topic, nodeHandle);

        //convert laser scan message to pcl PointCloud
        sensor_msgs::PointCloud2 msgPCL;
        pcl::PCLPointCloud2 pclCloud;
        projector_.projectLaser(*msgScan, msgPCL);
        pcl_conversions::toPCL(msgPCL, pclCloud);
        pcl::fromPCLPointCloud2(pclCloud,*cloud);

        //get panel line end points from available point cloud
        getPanelLine(cloud, p1, p2);

        bool b = isLineValid(p1, p2);

        ROS_INFO_STREAM(p1 << " " << p2 << " " << b);

        transform.setOrigin( tf::Vector3(p2.x, p2.y, p2.z - 0.116) );
        tf::Quaternion q;
        double yaw = atan2(p1.y - p2.y, p1.x - p2.x) - M_PI_2;
        std::cout << "yaw : " << yaw << std::endl;
        q.setRPY(-M_PI_2, 0, -M_PI_2 - yaw);
        transform.setRotation(q);

        geometry_msgs::Transform t;
        tf::transformTFToMsg(transform, t);
        res.origin = t;

        ROS_INFO(topic.c_str());
    }

private:
    ros::NodeHandle nodeHandle;
    ros::ServiceServer service;

    laser_geometry::LaserProjection projector_;

};

int main (int argc, char** argv) {
    // Initialize ROS
    ros::init (argc, argv, "my_pcl_tutorial");
    ros::NodeHandle nh;

    PanelDetector panelDetector(nh);

    // Create a ROS subscriber for the input point cloud
    //ros::Subscriber sub = nh.subscribe ("scan", 1, cloud_cb);

    // Create a ROS publisher for the output point cloud
    //pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

    //marker_pub = nh.advertise<visualization_msgs::Marker>("panel_scan", 10);

    //ros::ServiceServer service = nh.advertiseService("get_panel_origin", getPanelOrigin);

    // Spin
    ros::spin ();
}
