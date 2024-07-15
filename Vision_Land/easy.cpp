#include <iostream>
#include "opencv2/opencv.hpp"
using namespace cv;
int main(){
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(11,11 ));
    cv::Mat element1 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5,5 ));
    cv::Mat src=imread("/mnt/e/不想做工程/ROS视觉/yolov3t_zns.v1/zns_dataset/zns3/IMG_20240713_115338(3).jpg"),mid,dst;
    // /mnt/e/不想做工程/ROS视觉/yolov3t_zns.v1/zns_dataset/zns3/IMG_20240713_115057(3).jpg
    //IMG_20240713_115338(3).jpg
    resize(src,mid,Size(src.cols/2,src.rows/2));
    //mid=src.clone();
    inRange(mid, Scalar(0, 0, 0), Scalar(128,128,128), dst);
    //morphologyEx(mid,dst,MORPH_OPEN,element1);
    std::vector<std::vector<Point>> contours;
    std::vector<Vec4i> hierarchy;
    findContours(dst, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_SIMPLE);
    int maxarea=0,flag=-1;
    for(int i=0;i<contours.size();i++){
        if(contourArea(contours[i])>maxarea){
            maxarea=contourArea(contours[i]);
            flag=i;
        }
    }
    if(flag>=0&&flag<contours.size()){
        //声明一个图像的矩
		Moments M;
		//计算要绘制轮廓的矩
		M = moments(contours[flag]);
		//求取轮廓重心的X坐标
		double cX = double(M.m10 / M.m00);
		//求取轮廓重心的Y坐标
		double cY = double(M.m01 / M.m00);
        circle(mid, Point(cX,cY), 2, Scalar(255, 10, 255), 2, 8, 0);
        putText(mid, "Gravity_Point", Point(cX,cY), FONT_HERSHEY_COMPLEX, 0.5, Scalar(10, 255, 255));
    }

    imshow("mid",mid);
    //imshow("dst",dst);

    waitKey();
    return 0;
}
