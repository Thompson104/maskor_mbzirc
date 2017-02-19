#include <ros/ros.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <robot_perception/GetWrenchPose.h>

/*
void process_image(int, void*) {
    cv::Mat edge;

    cv::Canny(img, edge, canny_low, canny_up);

    cv::Mat element = cv::getStructuringElement(MORPH_RECT,
                                                Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                                Point( dilation_size, dilation_size ) );
    /// Apply the dilation operation
    dilate(edge, edge, element );
    imshow("Canny", edge);

    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    /// Find contours
    findContours( edge, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

    /// Find the rotated rectangles and ellipses for each contour
    vector<RotatedRect> minRect( contours.size() );
    vector<RotatedRect> minEllipse( contours.size() );
    for( int i = 0; i < contours.size(); i++ ) {
        minRect[i] = minAreaRect( Mat(contours[i]) );
        if( contours[i].size() > 50 ) {
            minEllipse[i] = fitEllipse( Mat(contours[i]) );
        }
    }

    /// Draw contours + rotated rects + ellipses
    Mat drawing = img.clone();
    cvtColor(drawing, drawing, CV_GRAY2BGR);
    //cvtColor(drawing, drawing, CV_GRAY2RGB);
    for( int i = 0; i< contours.size(); i++ )
    {
        Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
        // contour
        drawContours( drawing, contours, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
        // ellipse
        ellipse( drawing, minEllipse[i], color, 2, 8 );
        RotatedRect r = minEllipse[i];
        //cout << "ellipse : " << r.boundingRect().height << " " <<
        //        r.boundingRect().width << " " <<
        //        r.boundingRect().area() << endl;
    }
    imshow("contours", drawing);

}*/

class WrenchDetector {
public:

    WrenchDetector(ros::NodeHandle nh) : nodeHandle(nh), canny_max(255) {
        service = nodeHandle.advertiseService("get_wrench_pose", &WrenchDetector::getWrenchPose, this);

        canny_low = 0;
        canny_up = 100;
        dilation_size = 1;
    }

private:
    ros::NodeHandle nodeHandle;
    ros::ServiceServer service;

    cv_bridge::CvImagePtr cv_ptr;

    const int canny_max;
    int canny_low;
    int canny_up;
    int dilation_size;

    bool getWrenchPose(robot_perception::GetWrenchPose::Request  &req,
                       robot_perception::GetWrenchPose::Response &res) {

        cv::Mat image = getCVImage(req.imageRoi);

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

        /// Draw contours + rotated rects + ellipses
        cv::Mat drawing = image.clone();
        cv::cvtColor(drawing, drawing, CV_GRAY2BGR);
        //cvtColor(drawing, drawing, CV_GRAY2RGB);

        cv::RNG rng(12345);
        for( int i = 0; i< contours.size(); i++ )
        {
            cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
            // contour
            cv::drawContours( drawing, contours, i, color, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point() );
            // ellipse
            cv::ellipse( drawing, minEllipse[i], color, 2, 8 );
            cv::RotatedRect r = minEllipse[i];
            //cout << "ellipse : " << r.boundingRect().height << " " <<
            //        r.boundingRect().width << " " <<
            //        r.boundingRect().area() << endl;
        }
        cv::imshow("contours", drawing);

    }

    cv::Mat getCVImage(sensor_msgs::Image &msgImage) {
        cv::Mat img;

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
};


int main( int argc, char** argv ) {
    ros::init(argc, argv, "wrench_detector");
    ros::NodeHandle nh;
    WrenchDetector wrenchDetector(nh);

    ros::spin();
    return 0;
}
