#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Float64.h>
#include <tf/transform_listener.h>

#include <robot_perception/GetWrenchROI.h>

static const std::string OPENCV_WINDOW = "Image window";

cv::Mat img;

class ImageConverter
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    image_transport::CameraSubscriber camera_sub_;
    tf::TransformListener listener;

public:

    ImageConverter()
        : it_(nh_)
    {
        camera_sub_ = it_.subscribeCamera("/camera/image_rect", 1,
                                          &ImageConverter::cameraCb, this);

        image_pub_ = it_.advertise("/image_converter/output_video", 1);
    }

    ~ImageConverter()
    {
        //cv::destroyWindow(OPENCV_WINDOW);
    }

    void cameraCb(const sensor_msgs::ImageConstPtr& imgPtr, const sensor_msgs::CameraInfoConstPtr& infoPtr)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(imgPtr, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        img = cv_ptr->image;

        // Read 3D points
        std::vector<cv::Point3d> objectPoints;
        std::vector<cv::Point2d> imagePoints;

        objectPoints.push_back(cv::Point3d(0.55, -0.35, 0));
        objectPoints.push_back(cv::Point3d(1.0, -0.35, 0));
        objectPoints.push_back(cv::Point3d(1.0, -0.75, 0));
        objectPoints.push_back(cv::Point3d(0.55, -0.75, 0));

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

        objectPoints.push_back(cv::Point3d(0, 0, 0));

        //2835.918714 0.000000 1367.336906
        //0.000000 2833.011084 1124.028862
        //0.000000 0.000000 1.000000


        cv::Mat intrisicMat(3, 3, cv::DataType<double>::type); // Intrisic matrix
        intrisicMat.at<double>(0, 0) = 2835.918714;
        intrisicMat.at<double>(1, 0) = 0;
        intrisicMat.at<double>(2, 0) = 0;

        intrisicMat.at<double>(0, 1) = 0;
        intrisicMat.at<double>(1, 1) = 2833.011084;
        intrisicMat.at<double>(2, 1) = 0;

        intrisicMat.at<double>(0, 2) = 1367.336906;
        intrisicMat.at<double>(1, 2) = 1124.028862;
        intrisicMat.at<double>(2, 2) = 1;

        cv::Mat distCoeffs(5, 1, cv::DataType<double>::type);   // Distortion vector
        distCoeffs.at<double>(0) = -0.192941;
        distCoeffs.at<double>(1) = 0.158813;
        distCoeffs.at<double>(2) = 0.001327;
        distCoeffs.at<double>(3) = 0.000015;
        distCoeffs.at<double>(4) = 0;

        //get transform from camera to panel
        tf::StampedTransform transform;
        try{
            listener.lookupTransform("/camera", "/panel",
                                     ros::Time(0), transform);

            //std::cout << transform.getOrigin().x() << " " <<
            //             transform.getOrigin().y() << " " <<
            //             transform.getOrigin().z() << std::endl;
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }


        tf::Matrix3x3 rot(transform.getRotation());
        tfScalar r, p, y;
        rot.getRPY(r, p, y);

        cv::Mat rVec(3, 1, cv::DataType<double>::type); // Rotation vector
        rVec.at<double>(0) = 0;//-3.9277902400761393e-002;
        rVec.at<double>(1) = p;//3.7803824407602084e-002;
        rVec.at<double>(2) = 0;//2.6445674487856268e-002;

        cv::Mat rvecR(3,3,cv::DataType<double>::type);//rodrigues rotation matrix
        cv::Rodrigues(rVec,rvecR);
        std::cout << "rvec: " << r << " " << p << " " << y << std::endl;

        cv::Mat tVec(3, 1, cv::DataType<double>::type); // Translation vector
        tVec.at<double>(0) = transform.getOrigin().x();//2.1158489381208221e+000;
        tVec.at<double>(1) = transform.getOrigin().y();//-7.6847683212704716e+000;
        tVec.at<double>(2) = transform.getOrigin().z();//2.6169795190294256e+001;

        cv::projectPoints(objectPoints, rVec, tVec, intrisicMat, distCoeffs, imagePoints);

        for (unsigned int i = 0; i < imagePoints.size(); ++i)
        {
            cv::circle(img, cv::Point(imagePoints[i].x, imagePoints[i].y), 10, cv::Scalar(0, 255, 0), -1);
            std::cout << "Image point: " << imagePoints[i] << " Projected from " << objectPoints[i] << " " << img.rows << " " << img.cols << std::endl;
        }

        line( img, imagePoints[0], imagePoints[1], cv::Scalar(0, 255, 0), 6 );
        line( img, imagePoints[1], imagePoints[2], cv::Scalar(0, 255, 0), 6 );
        line( img, imagePoints[2], imagePoints[3], cv::Scalar(0, 255, 0), 6 );
        line( img, imagePoints[3], imagePoints[0], cv::Scalar(0, 255, 0), 6 );

        line( img, imagePoints[4], imagePoints[5], cv::Scalar(255, 0, 0), 4 );
        line( img, imagePoints[6], imagePoints[7], cv::Scalar(255, 0, 0), 4 );
        line( img, imagePoints[8], imagePoints[9], cv::Scalar(255, 0, 0), 4 );
        line( img, imagePoints[10], imagePoints[11], cv::Scalar(255, 0, 0), 4 );
        line( img, imagePoints[12], imagePoints[13], cv::Scalar(255, 0, 0), 4 );
        line( img, imagePoints[14], imagePoints[15], cv::Scalar(255, 0, 0), 4 );

        circle( img, imagePoints[14], 5, cv::Scalar(255, 0, 0), 4 );


        cv_bridge::CvImage out_msg;
        out_msg.header   = cv_ptr->header; // Same timestamp and tf frame as input image
        out_msg.encoding = cv_ptr->encoding; // Or whatever
        out_msg.image    = img; // Your cv::Mat

        // Output modified video stream
        image_pub_.publish(out_msg.toImageMsg());
    }
};


class WrenchROIDetector {
public:
    WrenchROIDetector(ros::NodeHandle nh) : nodeHandle(nh) {
        service = nodeHandle.advertiseService("get_wrench_roi", &WrenchROIDetector::getWrenchROI, this);

        initCameraParams();
    }

    bool getWrenchROI(robot_perception::GetWrenchROI::Request &req,
                      robot_perception::GetWrenchROI::Response &res) {

        tf::Transform transform;
        tf::transformMsgToTF(req.origin, transform);
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

        cv::Mat image = getCVImage(req.rectImageTopic);

        //checks if the points lie within the image
        if(isPointsValid(imagePoints, image.size())) {

            //correct the perspective before cropping
            cv::Mat rectImage = rectifyPerspective(imagePoints, image);

            res.imageRoi = *(imageCVToMsg(rectImage));

            return true;
        }
        return false;

    }

private:
    ros::NodeHandle nodeHandle;
    ros::ServiceServer service;

    //3D points projected to 2D points
    std::vector<cv::Point3d> objectPoints;
    std::vector<cv::Point2d> imagePoints;

    //Not the best way, but need to save the header
    //and the encoding for the response image
    cv_bridge::CvImagePtr cv_ptr;

    //Camera params
    cv::Mat intrisicMat;
    cv::Mat distCoeffs;

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

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_converter");
    ImageConverter ic;
    ros::spin();
    return 0;
}
