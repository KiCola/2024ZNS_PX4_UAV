#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "px4_command/hough.h"

//设定畸变参数和内参
const double k1 = -0.007785629015415907, k2 = 0.04529523849487305, p1 = 0.04243659973144531, p2 = 0.0078063542023301125;
const double fx = 285.9775085449219, fy = 286.22650146484375;
const double cx = 419.4216003417969, cy = 391.2044982910156;
//设定Canny边缘检测的阈值
const int canny_thresh_min=50,canny_thresh_max=100;
//设定霍夫圆检测的半径阈值
const int hough_radius_min=30,hough_radius_max=400;
//设定霍夫圆检测的分数阈值
const int hough_grade_thresh=50;
//设定膨胀与闭操作的元素大小
const int dilate_element_len=3,close_element_len=7;
//设定帧率
const int fps=10;


cv::Mat k = (cv::Mat_<float>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
cv::Mat d = (cv::Mat_<float>(1, 4) << k1, k2, p1, p2);


cv::Mat element_clo = getStructuringElement(cv::MORPH_RECT, cv::Size(close_element_len, close_element_len));
cv::Mat element_dil = getStructuringElement(cv::MORPH_RECT, cv::Size(dilate_element_len, dilate_element_len));
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    static ros::Publisher pub = ros::NodeHandle().advertise<px4_command::hough>("hough_sender", 100);
    px4_command::hough pmsg;
    cv_bridge::CvImagePtr cv_ptr;
    
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat& src = cv_ptr->image;
    if (src.empty()) {
        ROS_ERROR("cannot load image!");
        return;
    }

    int rows = src.rows, cols = src.cols;
    cv::Mat g_map1, g_map2;
    cv::Mat newCameraMatrix = getOptimalNewCameraMatrix(k, d, cv::Size(rows, cols), 1.0, cv::Size(rows, cols), 0);

    cv::Mat mid(rows, cols, CV_8UC1),pss; // 去畸变后的图
    cv::fisheye::initUndistortRectifyMap(k, d, cv::Mat::eye(3, 3, CV_64FC1), newCameraMatrix, cv::Size(rows, cols), CV_16SC2, g_map1, g_map2);
    cv::remap(src, mid, g_map1, g_map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT);

    cv::Mat dst;
    cv::medianBlur(mid, mid, 7);


    std::vector<cv::Vec3f> circles;
    //新加入canny边缘检测
    cv::Canny(mid, pss, canny_thresh_min, canny_thresh_max);    
    cv::dilate(pss,pss,element_dil);
    cv::morphologyEx(pss,dst,cv::MORPH_CLOSE,element_clo);
    cv::HoughCircles(dst, circles, cv::HOUGH_GRADIENT, 1, 10000, 100, hough_grade_thresh, hough_radius_min, hough_radius_max);

    if (!circles.empty()) {
        cv::Vec3f circ = circles[0];
        pmsg.x = circ[0];
        pmsg.y = circ[1];
        pmsg.rows = dst.rows;
        pmsg.cols = dst.cols;
        pmsg.fx = newCameraMatrix.at<float>(0, 0);
        pmsg.fy = newCameraMatrix.at<float>(1, 1);
        pmsg.cx = newCameraMatrix.at<float>(0, 2);
        pmsg.cy = newCameraMatrix.at<float>(1, 2);
        ROS_INFO("hough circle: X=%f, Y=%f, rows=%d, cols=%d", pmsg.x, pmsg.y, pmsg.rows, pmsg.cols);
        ROS_INFO("new_camera_matrix: fx=%f, fy=%f, cx=%f, cy=%f", pmsg.fx, pmsg.fy, pmsg.cx, pmsg.cy);
        pub.publish(pmsg);
        //cv::circle(pss,cv::Point(circ[0],circ[1]),circ[2],128,10);
        //cv::circle(pss,cv::Point(circ[0],circ[1]),5,128,10);
        //cv::imshow("hough_circ",pss);
        //cv::waitKey(50);
    } else {
        ROS_WARN("No circles found");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_processor");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("camera/fisheye1/image_raw", 1, imageCallback);

    ros::Rate loop_rate(10);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
