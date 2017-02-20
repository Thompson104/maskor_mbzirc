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

#include <robot_perception/GetWrenchPose.h>
#include <robot_perception/GetWrenchROI.h>

bool sortByHeigth(const cv::RotatedRect &r1, const cv::RotatedRect &r2) {
    return r1.boundingRect().height > r2.boundingRect().height;
}

class WrenchDetector {
public:

    WrenchDetector(ros::NodeHandle nh) : nodeHandle(nh), canny_max(255) {
        canny_low = 0;
        canny_up = 100;
        dilation_size = 1;
        num_wrenches = 6;
        roi_height = 400;

        //Appendix-5 of MBZIRC rules
        wrench_lengths[0] = 210;
        wrench_lengths[1] = 223;
        wrench_lengths[2] = 232;
        wrench_lengths[3] = 241;
        wrench_lengths[4] = 270;
        wrench_lengths[5] = 290;

        aspect_ratio = 15;

        initCameraParams();
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

    //wrench detection
    const int canny_max;
    int canny_low;
    int canny_up;
    int dilation_size;
    int num_wrenches;
    int roi_height;
    int wrench_lengths[6];
    int aspect_ratio;

    bool getWrenchPose(robot_perception::GetWrenchPose::Request  &req,
                       robot_perception::GetWrenchPose::Response &res) {

        cv::Mat image = getWrenchROI(req.origin, req.imageTopic);

        cv::Mat edge;
        cv::Canny(image, edge, canny_low, canny_up);
        cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,
                                                    cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                                    cv::Point( dilation_size, dilation_size ) );
        /// Apply the dilation operation
        cv::dilate(edge, edge, element );
        cv::imshow("Canny", edge);

        std::vector<std::vector<cv::Point> > contours;
        std::vector<cv::Vec4i> hierarchy;

        /// Find contours
        cv::findContours( edge, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

        /// Find the rotated rectangles and ellipses for each contour
        std::vector<cv::RotatedRect> minRect( contours.size() );
        std::vector<cv::RotatedRect> minEllipse( contours.size() );
        for( int i = 0; i < contours.size(); i++ ) {
            minRect[i] = cv::minAreaRect( cv::Mat(contours[i]) );
            if( contours[i].size() > 50 ) {
                minEllipse[i] = cv::fitEllipse( cv::Mat(contours[i]) );
            }
        }

        //Draw contours + rotated rects + ellipses
        cv::Mat drawing = image.clone();
        cv::cvtColor(drawing, drawing, CV_GRAY2BGR);

        cv::RNG rng(12345);
        std::vector<cv::RotatedRect> wrenches;
        for( int i = 0; i< contours.size(); i++ ) {
            cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
            // contour
            cv::drawContours( drawing, contours, i, color, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point() );
            // ellipse
            cv::ellipse( drawing, minEllipse[i], color, 2, 8 );
            cv::RotatedRect r = minEllipse[i];

            if(!isRectValid(r))
                continue;

            wrenches.push_back(r);
        }

        if(wrenches.size() != num_wrenches)
            return false;

        //mark this wrench in the image probably?
        res.wrenchPose = getRequestedWrenchPose(wrenches, req.wrenchNum);

        cv::imshow("contours", drawing);
        return true;
    }

    cv::Mat getWrenchROI(geometry_msgs::Transform msgTransform, std::string imageTopic) {

        tf::Transform transform;
        tf::transformMsgToTF(msgTransform, transform);
        tf::Matrix3x3 rot(transform.getRotation());
        tfScalar r, p, y;
        rot.getRPY(r, p, y);

        cv::Mat rVec(3, 1, cv::DataType<double>::type); // Rotation vector
        rVec.at<double>(0) = 0;
        rVec.at<double>(1) = p;
        rVec.at<double>(2) = 0;

        cv::Mat tVec(3, 1, cv::DataType<double>::type); // Translation vector
        tVec.at<double>(0) = transform.getOrigin().x();
        tVec.at<double>(1) = transform.getOrigin().y();
        tVec.at<double>(2) = transform.getOrigin().z();

        cv::projectPoints(objectPoints, rVec, tVec, intrisicMat, distCoeffs, imagePoints);

        cv::Mat image = getCVImage(imageTopic);

        //checks if the points lie within the image
        if(isPointsValid(imagePoints, image.size())) {

            //correct the perspective before cropping
            return rectifyPerspective(imagePoints, image);
        }
        return cv::Mat();

    }

    //sort wrenches by height
    geometry_msgs::Pose getRequestedWrenchPose(std::vector<cv::RotatedRect> &wrenches, std_msgs::UInt8 num) {
        std::sort(wrenches.begin(), wrenches.end(), sortByHeigth);

        cv::RotatedRect r = wrenches[num.data - 1];
        return get3dPose(r.center);
    }

    geometry_msgs::Pose get3dPose(cv::Point2f p) {
        geometry_msgs::Pose pose;
        //solvePnP
        return pose;
    }

    //check length and aspect ratio of detected contours
    bool isRectValid(cv::RotatedRect r) {
        float height = cv_ptr->image.rows;
        float wrench_len = roi_height * r.boundingRect().height / height;
        float aspect = r.boundingRect().height / r.boundingRect().width;

        for (int i = 0; i < num_wrenches; ++i) {
            //wrench detected should be between 90-110% of actual length
            bool b_len = ((wrench_lengths[i] * 1.1) > wrench_len) &&
                    ((wrench_lengths[i] * 0.9) < wrench_len);

            //aspect ratio should be between 90-110% of actual ratio
            bool b_aspect = ((aspect_ratio * 1.1) > aspect) &&
                    ((aspect_ratio * 0.9) < aspect);

            if(b_len && b_aspect)
                return true;
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
        intrisicMat.at<double>(0, 0) = 2835.918714;
        intrisicMat.at<double>(1, 0) = 0;
        intrisicMat.at<double>(2, 0) = 0;

        intrisicMat.at<double>(0, 1) = 0;
        intrisicMat.at<double>(1, 1) = 2833.011084;
        intrisicMat.at<double>(2, 1) = 0;

        intrisicMat.at<double>(0, 2) = 1367.336906;
        intrisicMat.at<double>(1, 2) = 1124.028862;
        intrisicMat.at<double>(2, 2) = 1;

        distCoeffs.at<double>(0) = -0.192941;
        distCoeffs.at<double>(1) = 0.158813;
        distCoeffs.at<double>(2) = 0.001327;
        distCoeffs.at<double>(3) = 0.000015;
        distCoeffs.at<double>(4) = 0;

        objectPoints.push_back(cv::Point3d(0.55, -0.35, 0));
        objectPoints.push_back(cv::Point3d(1.0, -0.35, 0));
        objectPoints.push_back(cv::Point3d(1.0, -0.75, 0));
        objectPoints.push_back(cv::Point3d(0.55, -0.75, 0));

        //won't need this.. just fancy stuff
        objectPoints.push_back(cv::Point3d(0.70, -0.35, 0));
        objectPoints.push_back(cv::Point3d(0.70, -0.75, 0));
        objectPoints.push_back(cv::Point3d(0.75, -0.35, 0));
        objectPoints.push_back(cv::Point3d(0.75, -0.75, 0));
        objectPoints.push_back(cv::Point3d(0.80, -0.35, 0));
        objectPoints.push_back(cv::Point3d(0.80, -0.75, 0));
        objectPoints.push_back(cv::Point3d(0.85, -0.35, 0));
        objectPoints.push_back(cv::Point3d(0.85, -0.75, 0));
        objectPoints.push_back(cv::Point3d(0.90, -0.35, 0));
        objectPoints.push_back(cv::Point3d(0.90, -0.75, 0));
        objectPoints.push_back(cv::Point3d(0.95, -0.35, 0));
        objectPoints.push_back(cv::Point3d(0.95, -0.75, 0));
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

    cv::Mat rectifyPerspective(std::vector<cv::Point2d> points, cv::Mat image) {
        cv::Mat rectImage = cv::Mat::zeros( image.rows, image.cols, image.type() );

        // The 4 points where the mapping is to be done , from top-left in clockwise order
        //TODO: are the 'points' are in order?
        cv::Point2f inputQuad[4];
        inputQuad[0] = points[0];
        inputQuad[1] = points[1];
        inputQuad[2] = points[2];
        inputQuad[3] = points[3];

        cv::Point2f outputQuad[4];
        outputQuad[0] = cv::Point2d(0, 0);
        outputQuad[1] = cv::Point2d(image.cols-1, 0);
        outputQuad[2] = cv::Point2d(image.cols-1, image.rows-1);
        outputQuad[3] = cv::Point2d(0, image.rows-1);

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
