#include <ros/ros.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/convex_hull.h>
#include <boost/thread/thread.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>

#include <robot_perception/GetValvePose.h>

class ValveDetector {
public:
    ValveDetector(ros::NodeHandle nh) : nodeHandle(nh) {
        service = nodeHandle.advertiseService("get_valve_pose", &ValveDetector::getValvePose, this);
    }

private:
    ros::NodeHandle nodeHandle;
    ros::ServiceServer service;

    bool getValvePose(robot_perception::GetValvePose::Request  &req,
                       robot_perception::GetValvePose::Response &res) {


        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(req.valveScan, *cloud);

        pcl::ConvexHull<pcl::PointXYZ> cHull;
        pcl::PointCloud<pcl::PointXYZ> cHull_points;
        //pcl::PointIndices::Ptr cHull_inliers (new pcl::PointIndices);
        cHull.setInputCloud(cloud);
        cHull.reconstruct (cHull_points);
        cHull_points[cHull_points.size() - 1] = cHull_points[0];  //only if last element is 0... else push_back the 1st element

        // Neighbors within radius search

        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud (cloud);

        float radius = 1e-20;
        int idxPrev = -1;
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        for (int i = 0; i < cHull_points.size(); ++i) {
            //find indices of the convex hull point
            pcl::PointXYZ searchPoint = cHull_points[i];

            pointIdxRadiusSearch.clear();
            pointRadiusSquaredDistance.clear();
            if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )        {
                //get indices between 2 adjacent convex hull points
                if(idxPrev >= 0) {
                    pcl::PointIndices::Ptr line_inliers (new pcl::PointIndices);
                    int nPts = abs(pointIdxRadiusSearch[0] - idxPrev);
                    line_inliers->indices.resize(nPts + 1);

                    int start = std::min(idxPrev, pointIdxRadiusSearch[0]);

                    for (int j = 0; j <= nPts; ++j) {
                        line_inliers->indices[j] = start + j;
                    }

                    //extract each 'edge' of convex hull
                    if (line_inliers->indices.size () < 100) {
                        std::cerr << "too less num of points for a line" << std::endl;
                        continue;
                    }

                    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZ>);
                    pcl::ExtractIndices<pcl::PointXYZ> extract;

                    // Extract the inliers
                    extract.setInputCloud (cloud);
                    extract.setIndices (line_inliers);
                    extract.setNegative (false);
                    extract.filter (*cloud_p);

                    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
                    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
                    // Create the segmentation object
                    pcl::SACSegmentation<pcl::PointXYZ> seg;
                    // Optional
                    seg.setOptimizeCoefficients (true);
                    // Mandatory
                    seg.setModelType (pcl::SACMODEL_LINE);
                    seg.setMethodType (pcl::SAC_RANSAC);
                    seg.setDistanceThreshold (0.0001);

                    seg.setInputCloud (cloud_p);
                    seg.segment (*inliers, *coefficients);

                    if (inliers->indices.size () == 0) {
                        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
                        return (-1);
                    }

                    pcl::PointXYZ p1 = cloud_p->points[inliers->indices[0]];
                    pcl::PointXYZ p2 = cloud_p->points[inliers->indices[inliers->indices.size() - 1]];
                    float d = dist(p1, p2);

                    if(d > 0.01 || d < 0.014) {     //TODO: add param
                        //find 'entry point'
                        float m2 = -coefficients->values[3] / coefficients->values[4];//slope of perpendicular
                        pcl::PointXYZ mid;
                        mid.x = (p1.x + p2.x) / 2.0;
                        mid.y = (p1.y + p2.y) / 2.0;
                        mid.z = (p1.z + p2.z) / 2.0;

                        pcl::PointXYZ p;
                        if(m2 < 0)
                            p.x = mid.x - 0.012 / sqrt(1 + m2 * m2);
                        else
                            p.x = mid.x + 0.012 / sqrt(1 + m2 * m2);
                        p.y = (p.x - mid.x)*m2 + mid.y;
                        p.z = 0;//mid.z + 0.012 / sqrt(1 + m2 * m2);
                    }

                    //TODO: if dist != valve width, exclude it

                }
                idxPrev = pointIdxRadiusSearch[0];

            }
        }

    }

    float dist(pcl::PointXYZ p1, pcl::PointXYZ p2) {
        float d = pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2);
        return sqrt(d);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "valve_detector");
    ros::NodeHandle nh;
    ValveDetector valveDetector(nh);

    ros::spin();

    return 0;
}
