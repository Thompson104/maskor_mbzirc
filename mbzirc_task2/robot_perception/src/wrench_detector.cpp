#include <ros/ros.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <std_msgs/UInt8.h>
#include <std_msgs/Float64.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>
#include <tf/transform_listener.h>

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

#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <boost/thread/thread.hpp>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <image_geometry/pinhole_camera_model.h>

#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>

#include <robot_perception/GetWrenchPose.h>
#include <robot_perception/GetWrenchROI.h>

cv::RNG rng(12345);

bool sortByHeigth(const cv::RotatedRect &r1, const cv::RotatedRect &r2) {
    return r1.boundingRect().height > r2.boundingRect().height;
}

bool sortByX(const cv::RotatedRect &r1, const cv::RotatedRect &r2) {
    return r1.center.x < r2.center.x;
}

class PanelDetector {
public:
    PanelDetector(ros::NodeHandle nh) : nodeHandle(nh) {
        marker_pub = nodeHandle.advertise<visualization_msgs::Marker>("panel_line", 10);
        tf_broadcaster = new tf::TransformBroadcaster();
    }

    geometry_msgs::Transform getPanelOrigin(std::string topic) {
        geometry_msgs::Point p1, p2;
        tf::Transform transform;
        geometry_msgs::Transform t;

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

        //get scan data from TIM
        sensor_msgs::LaserScanConstPtr msgScan = ros::topic::waitForMessage<sensor_msgs::LaserScan>(topic, nodeHandle);

        //convert laser scan message to pcl PointCloud
        sensor_msgs::PointCloud2 msgPCL;
        pcl::PCLPointCloud2 pclCloud;
        projector_.projectLaser(*msgScan, msgPCL);
        pcl_conversions::toPCL(msgPCL, pclCloud);
        pcl::fromPCLPointCloud2(pclCloud,*cloud);

        //get panel line end points from available point cloud
        getPanelLine(cloud, p1, p2);

        if(isLineValid(p1, p2)) {
            visualizePanelLine(p1, p2);
            transform.setOrigin( tf::Vector3(p2.x, p2.y, p2.z - 0.15) );
            tf::Quaternion q;
            double yaw = atan2(p1.y - p2.y, p1.x - p2.x) - M_PI_2;
            std::cout << "panel in laser frame " << p2.x << " " <<
                         p2.y << " " <<
                         p2.z - 0.15 << std::endl;
            //std::cout << "yaw : " << yaw << std::endl;
            q.setRPY(-M_PI_2, 0, M_PI_2 + yaw);
            transform.setRotation(q);

            tf_broadcaster->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "laser", "panel"));
            tf::transformTFToMsg(transform, t);
        }
        return t;
    }

private:
    ros::NodeHandle nodeHandle;
    laser_geometry::LaserProjection projector_;

    ros::Publisher marker_pub;
    tf::TransformBroadcaster *tf_broadcaster;

    double dist(geometry_msgs::Point p1, geometry_msgs::Point p2) {
        double d = pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2);
        return sqrt(d);
    }

    bool isLineValid(geometry_msgs::Point p1, geometry_msgs::Point p2) {
        double d = dist(p1, p2);
        if(d > 0.9 && d < 1.1) {    //Is line around 1m?
            return true;
        }
        return false;
    }

    void visualizePanelLine(geometry_msgs::Point p1, geometry_msgs::Point p2) {
        //draw line in rviz
        visualization_msgs::Marker lines;
        lines.header.frame_id = "laser";
        lines.header.stamp = ros::Time::now();
        lines.action = visualization_msgs::Marker::ADD;
        lines.pose.orientation.w = 1.0;
        lines.id = 1;
        lines.type = visualization_msgs::Marker::LINE_LIST;
        lines.scale.x = 0.02;
        lines.color.g = 1.0;
        lines.color.a = 1.0;

        lines.points.push_back(p1);
        lines.points.push_back(p2);

        marker_pub.publish(lines);
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
};

class WrenchDetector {
public:

    WrenchDetector(ros::NodeHandle nh) : nodeHandle(nh), canny_max(255) {
        canny_low = 0;
        canny_up = 25;
        dilation_size = 20;
        num_wrenches = 6;
        roi_height = 400;

        //Appendix-5 of MBZIRC rules
        wrench_lengths[0] = 210;
        wrench_lengths[1] = 223;
        wrench_lengths[2] = 232;
        wrench_lengths[3] = 241;
        wrench_lengths[4] = 270;
        wrench_lengths[5] = 290;

        aspect_ratio = 4;

        initCameraParams();
        panelDetector = new PanelDetector(nodeHandle);

        service = nodeHandle.advertiseService("get_wrench_pose", &WrenchDetector::getWrenchPose, this);
    }

private:
    ros::NodeHandle nodeHandle;
    ros::ServiceServer service;

    cv_bridge::CvImagePtr cv_ptr;


    //3D points projected to 2D points
    std::vector<cv::Point3d> objectPoints;
    std::vector<cv::Point2d> imagePoints;

    //Wrench ROI detection
    //Not the best way, but need to save the header
    //and the encoding for the response image

    //Camera params
    cv::Mat intrisicMat;
    cv::Mat distCoeffs;
    cv::Mat tVec;
    tf::Matrix3x3 rot;

    tf::TransformListener tf_listener;

    //wrench detection
    const int canny_max;
    int canny_low;
    int canny_up;
    int dilation_size;
    int num_wrenches;
    int roi_height;
    int wrench_lengths[6];
    int aspect_ratio;

    //panel detection
    PanelDetector *panelDetector;

    bool getWrenchPose(robot_perception::GetWrenchPose::Request  &req,
                       robot_perception::GetWrenchPose::Response &res) {

        geometry_msgs::Transform transform = panelDetector->getPanelOrigin(req.scanTopic);
        if(transform.translation.x == 0 &&
                transform.translation.y == 0 &&
                transform.translation.z == 0) {
            ROS_INFO("ERROR : Panel not detected");
            return false;
        }

        cv::Mat image = getWrenchROI(transform, req.imageTopic);
        cv::imwrite("roi.png", image);

        if(image.rows == 0 || image.cols == 0){
            ROS_INFO("ERROR : Wrench ROI is does not lie in the image");
            return false;
        }

        std::vector<cv::RotatedRect> wrenches = processImage(image, req.wrenchNum);
        if(wrenches.size() != num_wrenches) {
        //if(wrenches.size() < 1) {
            ROS_INFO("ERROR : Wrenches not detected");
            return false;
        }

        //mark this wrench in the image probably?
        res.wrenchPose = getRequestedWrenchPose(wrenches, req.wrenchNum);
        return true;
    }

    std::vector<cv::RotatedRect> processImage(cv::Mat img, int num) {
        cv::Mat edge, imgLaplacian, drawing;
        drawing = img.clone();

        std::vector<cv::RotatedRect> wrenches;
        std::vector<std::vector<cv::Point> > contours;
        std::vector<cv::Vec4i> hierarchy;

        cv::Mat kernel = (cv::Mat_<float>(3,3) <<
                          1, 0, 1,
                          1, -6, 1,
                          1, 0, 1); // an approximation of second derivative, a quite strong kernel
        // do the laplacian filtering as it is

        cv::Mat sharp = img; // copy source image to another temporary one
        cv::filter2D(sharp, imgLaplacian, CV_32F, kernel);
        img.convertTo(sharp, CV_32F);
        cv::Mat imgResult = sharp - imgLaplacian;
        // convert back to 8bits gray scale
        imgResult.convertTo(imgResult, CV_8UC3);
        imgLaplacian.convertTo(imgLaplacian, CV_8UC3);
        // imshow( "Laplace Filtered Image", imgLaplacian );

        //cv::namedWindow("sharp", CV_WINDOW_NORMAL);
        //cv::imshow( "sharp", imgResult );
        img = imgResult; // copy back
        // Create binary image from source image
        cv::cvtColor(img, img, CV_BGR2GRAY);

        cv::GaussianBlur( img, edge, cv::Size(15, 15), 0, 0 );
        cv::Canny(edge, edge, canny_low, canny_up);

        cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,
                                                    cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                                    cv::Point( dilation_size, dilation_size ) );
        /// Apply the dilation operation
        cv::morphologyEx(edge, edge, cv::MORPH_CLOSE, element );
        //cv::namedWindow("Canny", CV_WINDOW_NORMAL);
        cv::imwrite("Canny.png", edge);

        // Find contours
        cv::findContours( edge, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

        // Find the rotated rectangles for valid contours
        for( int i = 0; i < contours.size(); i++ ) {
            cv::RotatedRect r = cv::minAreaRect( cv::Mat(contours[i]) );
            if(isRectValid(r, contours[i])) {
                cv::Point2f rect_points[4];
                r.points( rect_points );

                cv::Scalar color = cv::Scalar(255, 0, 0);
                for( int j = 0; j < 4; j++ ) {
                    line( drawing, rect_points[j], rect_points[(j+1)%4], color, 14, 8 );
                }

                wrenches.push_back(r);
            }
        }
        std::cout << "no of wrenches : " << wrenches.size() << std::endl;
        //rest of the part to delete!


        if(wrenches.size() > 0) {
            std::sort(wrenches.begin(), wrenches.end(), sortByHeigth);

            cv::RotatedRect rSel = wrenches[num - 1];
            cv::Point2f rect_points[4];
            rSel.points( rect_points );

            cv::Scalar color = cv::Scalar(0, 255, 0);
            for( int j = 0; j < 4; j++ ) {
                line( drawing, rect_points[j], rect_points[(j+1)%4], color, 14, 8 );
            }

            cv::namedWindow("contours", CV_WINDOW_NORMAL);
            imwrite("contours.png", drawing);

            //cv::waitKey(0);
        }
        return wrenches;
    }

    cv::Mat getWrenchROI(geometry_msgs::Transform msgTransform, std::string imageTopic) {

        tf::StampedTransform transform = getCameraTransform("camera", "panel");
        rot = tf::Matrix3x3(transform.getRotation());
        tfScalar r, p, y;
        rot.getRPY(r, p, y);

        cv::Mat rVec(3, 1, cv::DataType<double>::type); // Rotation vector
        rVec.at<double>(0) = r;
        rVec.at<double>(1) = p;
        rVec.at<double>(2) = y;

        tVec = cv::Mat(3, 1, cv::DataType<double>::type); // Translation vector
        tVec.at<double>(0) = transform.getOrigin().x();
        tVec.at<double>(1) = transform.getOrigin().y();
        tVec.at<double>(2) = transform.getOrigin().z();

        cv::projectPoints(objectPoints, rVec, tVec, intrisicMat, distCoeffs, imagePoints);

        cv::Mat image = getCVImage(imageTopic);
        cv::Mat drawing = image.clone();

        //draw the roi

        line( drawing, imagePoints[0], imagePoints[1], cv::Scalar(0, 255, 0), 6 );
        line( drawing, imagePoints[1], imagePoints[2], cv::Scalar(0, 255, 0), 6 );
        line( drawing, imagePoints[2], imagePoints[3], cv::Scalar(0, 255, 0), 6 );
        line( drawing, imagePoints[3], imagePoints[0], cv::Scalar(0, 255, 0), 6 );


        line( drawing, imagePoints[4], imagePoints[5], cv::Scalar(255, 0, 0), 4 );
        line( drawing, imagePoints[6], imagePoints[7], cv::Scalar(255, 0, 0), 4 );
        line( drawing, imagePoints[8], imagePoints[9], cv::Scalar(255, 0, 0), 4 );
        line( drawing, imagePoints[10], imagePoints[11], cv::Scalar(255, 0, 0), 4 );
        line( drawing, imagePoints[12], imagePoints[13], cv::Scalar(255, 0, 0), 4 );
        line( drawing, imagePoints[14], imagePoints[15], cv::Scalar(255, 0, 0), 4 );

        //cv::namedWindow("roi", CV_WINDOW_NORMAL);
        //cv::imshow("roi", drawing);
        //cv::waitKey(0);
        cv::imwrite("lines.png", drawing);

        //checks if the points lie within the image
        if(isPointsValid(imagePoints, image.size())) {

            //correct the perspective before cropping
            return rectifyPerspective(imagePoints, image);
        }
        return cv::Mat();

    }

    tf::StampedTransform getCameraTransform(std::string parent, std::string child) {
        tf::StampedTransform transform;
        try{
            tf_listener.waitForTransform(parent, child,
                                         ros::Time(0), ros::Duration(3.0));
            tf_listener.lookupTransform(parent, child,
                                        ros::Time(0), transform);

        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
        return transform;
    }

    //sort wrenches by height
    geometry_msgs::Pose getRequestedWrenchPose(std::vector<cv::RotatedRect> &wrenches, int num) {
        geometry_msgs::Pose pose;
        //std::sort(wrenches.begin(), wrenches.end(), sortByHeigth);
        //cv::RotatedRect r = wrenches[num - 1];

        //sort indices to get the order of wrenches on panel
        std::sort(wrenches.begin(), wrenches.end(), sortByHeigth);

        cv::RotatedRect r = wrenches[num - 1];

        std::vector<double> w;
        for (int i = 0; i < wrenches.size(); ++i) {
            double x = wrenches[i].center.x;
            w.push_back(x);
        }

        for (int i = 0; i < wrenches.size(); ++i) {
            std::cout << wrenches[i].center << " " << w[i] <<  std::endl;
        }

        std::sort(wrenches.begin(), wrenches.end(), sortByX);
        std::cout << "===" << std::endl;
        for (int i = 0; i < wrenches.size(); ++i) {
            std::cout << wrenches[i].center << " " << w[i] <<  std::endl;
        }

        for (int i = 0; i < wrenches.size(); ++i) {
            if(r.center.x == wrenches[i].center.x ) {
                std::cout << i << " " << r.center.x << " " << w[i] <<  std::endl;

                tf::Stamped<tf::Pose> p1, p2;
                p1.frame_id_ = "panel";
                p1.setOrigin(tf::Vector3(objectPoints[i * 2 + 4].x,
                             objectPoints[i * 2 + 4].y,
                        objectPoints[i * 2 + 4].z));
                std::cout << "panel point : " << p1.getOrigin().x() << " " << p1.getOrigin().y() << " " << p1.getOrigin().z() << std::endl;
                tf::Quaternion q;
                q.setRPY(0, 0, 0);
                p1.setRotation(q);

                //change the target frame to 'base_link' -- only the sideways component is wrong by about 10cm!
                tf_listener.transformPose("base_link", p1, p2);
                std::cout << "wrench in base_link frame : " << p2.getOrigin().x() << " " <<
                             p2.getOrigin().y() << " " <<
                             p2.getOrigin().z() << std::endl;

                pose.position.x = -0.45;//p2.getOrigin().x();
                pose.position.y = p2.getOrigin().y();
                pose.position.z = 1.015;//p2.getOrigin().z();
            }
        }
        return pose;
        /*
        for (int i = 0; i < wrenches.size(); ++i) {
            cv::RotatedRect r1 = wrenches[i];
            std::cout << i << std::endl;

            get3dPose(r1.center);
        }

        return get3dPose(r.center);*/
    }

    geometry_msgs::Pose get3dPose(cv::Point2f p) {
        geometry_msgs::Pose pose;
        //convert 2D image point to 3D world point

        std::vector<cv::Vec2d> imagePts;
        std::vector<cv::Vec2d> idealPts;
        //Add the values to compensate for the ROI
        imagePts.push_back(cv::Vec2d(p.x, p.y));

        cv::undistortPoints(imagePts, idealPts, intrisicMat, distCoeffs);

        const double lambda = 1.0;
        cv::Mat cameraPt(3, 1, CV_64F);
        cameraPt.at<double>(0) = idealPts[0][0] * lambda;
        cameraPt.at<double>(1) = idealPts[0][1] * lambda;
        cameraPt.at<double>(2) = 1;
        cameraPt.push_back(1.0);

        //std::cout << "camera pt : " << cameraPt << " " << imagePts[0] << " " << idealPts[0] << " " << idealPts[1] << std::endl;

        cv::Mat camToWorld = cv::Mat::eye(4, 4, CV_64FC1);
        tf::StampedTransform _t = getCameraTransform("panel", "camera");
        tf::Matrix3x3 _r = tf::Matrix3x3(_t.getRotation());

        cv::Mat _R = cv::Mat(3, 3, CV_64F);
        for(int i = 0; i < 3; ++i)
            for(int j = 0; j < 3; ++j)
                _R.at<double>(j,i) = _r[j][i];

        //_R = cv::Mat_<double>::eye(3, 3);
        _R.copyTo(camToWorld(cv::Rect_<double>(0, 0, 3, 3)));
        camToWorld.at<double>(0, 3) = _t.getOrigin().x();
        camToWorld.at<double>(1, 3) = _t.getOrigin().y();
        camToWorld.at<double>(2, 3) = _t.getOrigin().z();
        cv::Mat worldPt = camToWorld * cameraPt;

        double t = -worldPt.at<double>(2) / _t.getOrigin().z();
        double x = t * _t.getOrigin().x() + worldPt.at<double>(0);
        double y = t * _t.getOrigin().y() + worldPt.at<double>(1);

        std::cout << x << ", " << y << std::endl;

        //transform the point in UR5's frame of ref

        tf::Stamped<tf::Pose> p1, p2;
        p1.frame_id_ = "panel";
        p1.setOrigin(tf::Vector3(x, y, 0));
        tf::Quaternion q;
        q.setRPY(0, 0, 0);
        p1.setRotation(q);

        //change the target frame to 'base_link' -- only the sideways component is wrong by about 10cm!
        tf_listener.transformPose("base_link", p1, p2);
        //std::cout << "wrench in base_link frame : " << p2.getOrigin().x() << " " <<
        //             p2.getOrigin().y() << " " <<
        //             p2.getOrigin().z() << std::endl;


        return pose;
    }

    //check length and aspect ratio of detected contours
    bool isRectValid(cv::RotatedRect r, std::vector<cv::Point> contour) {
        float height = cv_ptr->image.rows;
        float wrench_len = roi_height * r.boundingRect().height / height;
        float aspect = r.boundingRect().height / r.boundingRect().width;
        float area = r.boundingRect().area();

        cv::Moments mu = cv::moments(contour);

        for (int i = 0; i < num_wrenches; ++i) {
            //wrench detected should be between 90-110% of actual length
            bool b_len = ((wrench_lengths[i] * 1.1) > wrench_len) &&
                    ((wrench_lengths[i] * 0.9) < wrench_len);

            //aspect ratio should be between 90-110% of actual ratio
            bool b_aspect = (5 > aspect) &&
                    (1 < aspect);

            //area of the rect should be greater than a threshold
            bool b_area = area > 10000;

            bool b_moments = (0.3 < mu.nu02) && (0.9 > mu.nu02);

            //std::cout << r.boundingRect().size() << " " << aspect << " " << area  << " " << r.center << " " << mu.nu02 << std::endl;
            //std::cout << b_aspect << " " << b_area << " " << b_moments << std::endl;
            if(b_aspect && b_area && b_moments) {
                //std::cout << r.boundingRect().size() << " " << r.center << " " << mu.nu02 << std::endl;

                return true;
            }
        }
        return false;
    }

    cv::Mat getCVImage(sensor_msgs::Image &msgImage) {
        cv::Mat img;

        try {
            cv_ptr = cv_bridge::toCvCopy(msgImage, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return cv::Mat();
        }

        img = cv_ptr->image;
        return img;
    }


    //----WRENCH ROI DETECTION----//

    //should read from parameter server?
    void initCameraParams() {
        intrisicMat = cv::Mat(3, 3, cv::DataType<double>::type); // Intrisic matrix
        intrisicMat.at<double>(0, 0) = 2841.144719;
        intrisicMat.at<double>(1, 0) = 0;
        intrisicMat.at<double>(2, 0) = 0;

        intrisicMat.at<double>(0, 1) = 0;
        intrisicMat.at<double>(1, 1) = 2836.957608;
        intrisicMat.at<double>(2, 1) = 0;

        intrisicMat.at<double>(0, 2) = 1396.523199;
        intrisicMat.at<double>(1, 2) = 1119.404720;
        intrisicMat.at<double>(2, 2) = 1;

        distCoeffs = cv::Mat(5, 1, cv::DataType<double>::type);   // Distortion vector
        distCoeffs.at<double>(0) = -0.202081;
        distCoeffs.at<double>(1) = 0.140908;
        distCoeffs.at<double>(2) = 0.000522;
        distCoeffs.at<double>(3) = 0.000769;
        distCoeffs.at<double>(4) = 0;

        objectPoints.push_back(cv::Point3d(0.55, -0.30, 0));
        objectPoints.push_back(cv::Point3d(1.0, -0.30, 0));
        objectPoints.push_back(cv::Point3d(1.0, -0.55, 0));
        objectPoints.push_back(cv::Point3d(0.55, -0.55, 0));

        //won't need this.. just fancy stuff
        objectPoints.push_back(cv::Point3d(0.70, -0.30, 0));
        objectPoints.push_back(cv::Point3d(0.70, -0.55, 0));
        objectPoints.push_back(cv::Point3d(0.75, -0.30, 0));
        objectPoints.push_back(cv::Point3d(0.75, -0.55, 0));
        objectPoints.push_back(cv::Point3d(0.80, -0.30, 0));
        objectPoints.push_back(cv::Point3d(0.80, -0.55, 0));
        objectPoints.push_back(cv::Point3d(0.85, -0.30, 0));
        objectPoints.push_back(cv::Point3d(0.85, -0.55, 0));
        objectPoints.push_back(cv::Point3d(0.90, -0.30, 0));
        objectPoints.push_back(cv::Point3d(0.90, -0.55, 0));
        objectPoints.push_back(cv::Point3d(0.95, -0.30, 0));
        objectPoints.push_back(cv::Point3d(0.95, -0.55, 0));
    }

    //convert from cv::Point2d to geometry_msgs::Point
    geometry_msgs::Point pointCVToMsg(cv::Point2d _p) {
        geometry_msgs::Point p;
        p.x = _p.x;
        p.y = _p.y;

        return p;
    }

    sensor_msgs::ImagePtr imageCVToMsg(cv::Mat img) {
        cv_bridge::CvImage out_msg;
        out_msg.header   = cv_ptr->header; // Same timestamp and tf frame as input image
        out_msg.encoding = cv_ptr->encoding; // Or whatever
        out_msg.image    = img; // Your cv::Mat

        // Output modified video stream
        return out_msg.toImageMsg();
    }

    //checks if the points lie within the image
    bool isPointsValid( std::vector<cv::Point2d> points, cv::Size size) {
        bool b = true;
        for (int i = 0; i < points.size(); ++i) {
            b = b && (points[i].x >= 0) && (points[i].x <= size.width) &&
                    (points[i].y >= 0) && (points[i].y <= size.height);
            //std::cout << points[i].x << ", " << points[i].y << " " << size << " " << b << std::endl;
        }
        return b;
    }

    cv::Mat getCVImage(std::string topic) {
        cv::Mat img;
        sensor_msgs::ImageConstPtr msgImage = ros::topic::waitForMessage<sensor_msgs::Image>(topic, nodeHandle);

        try
        {
            cv_ptr = cv_bridge::toCvCopy(msgImage, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return cv::Mat();
        }

        img = cv_ptr->image;
        return img;
    }


    double dist(cv::Point2d p1, cv::Point2d p2) {
        double d = pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2);
        return sqrt(d);
    }

    cv::Mat rectifyPerspective(std::vector<cv::Point2d> points, cv::Mat image) {
        double y = dist(points[0], points[1]);
        double x = dist(points[1], points[2]);
        cv::Mat rectImage = cv::Mat::zeros(x, y, image.type() );

        // The 4 points where the mapping is to be done , from top-left in clockwise order
        //TODO: are the 'points' are in order?
        cv::Point2f inputQuad[4];
        inputQuad[0] = points[3];
        inputQuad[1] = points[2];
        inputQuad[2] = points[1];
        inputQuad[3] = points[0];

        cv::Point2f outputQuad[4];
        outputQuad[0] = cv::Point2d(0, 0);
        outputQuad[1] = cv::Point2d(y - 1, 0);
        outputQuad[2] = cv::Point2d(y - 1, x - 1);
        outputQuad[3] = cv::Point2d(0, x - 1);

        // Get the Perspective Transform Matrix i.e. lambda
        cv::Mat lambda = getPerspectiveTransform(inputQuad, outputQuad);
        // Apply the Perspective Transform just found to the src image
        warpPerspective(image, rectImage, lambda, rectImage.size());

        return rectImage;
    }
};

int main( int argc, char** argv ) {
    ros::init(argc, argv, "wrench_detector");
    ros::NodeHandle nh;
    WrenchDetector wrenchDetector(nh);

    ros::spin();
    return 0;
}
