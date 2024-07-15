//测试  视觉降落 + 避障 的算法

/*---------------------------------------头文件----------------------------------------------*/
#include <ros/ros.h>
#include <iostream>
#include <px4_command/command.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <cmath>
#include <stdlib.h>
#include <math_utils.h>
#include <vector>
#include <random>
#include <numeric>

#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
//#include "px4_command/hough.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using namespace std;
/*---------------------------------------全局变量----------------------------------------------*/
enum Command
{
	Move_ENU,
	Move_Body,
	Hold,
	Takeoff,
	Land,
	Arm,
	Disarm,
	Failsafe_land,
	Idle
};

struct Node{
	float x, y;
	Node()
	{
		x = y = 0;
	}
};

const float Pi = acos(-1.0);
const float INF = 100;
int comid;  //指令id
Node c_target;

/*所需输入数据 */
sensor_msgs::LaserScan Laser;		  // 激光雷达点云数据
geometry_msgs::PoseStamped pos_drone; // 无人机当前位置
Eigen::Quaterniond q_fcu;
Eigen::Vector3d Euler_fcu;

float target_x; // 期望位置_x
float target_y; // 期望位置_y
float fly_height; //飞行高度

float initial_target_x; //穿环前初始位置x
float initial_target_y; //穿环前初始位置y


int range_min;			// 激光雷达探测范围 最小角度
int range_max;			// 激光雷达探测范围 最大角度
float last_time = 0;
int angle_c = 0;

//int angle_c

float search_distance[4]; // 视觉寻找降落点 [0][1] x方向--前后最大值 [2][3] y方向--左右最大值 
int flag_land;					// 降落标志位
float abs_distance;				// 到目标点的距离

float vel_sp_body[2];					 // 总速度
float vel_sp_ENU[2];					 // ENU下的总速度
float vel_sp_max;	
float bj ;//障碍物安全界限                                          //ENU下的总速度                                               //总速度限幅
px4_command::command Command_now;   
//--------------------------------------------算法相关--------------------------------------------------
/*ocvd*/
//px4_command::hough target_circ;
float theoretical_distance_to_circ;
float max_detection_distance;//最大容许理论距离，待设定
float min_detection_distance;//最小容许理论距离，待设定
float presetCir_x, presetCir_y; //预设的圆环坐标

float near_min_distance;//离最小容许理论距离较近，若last_judgement为真且本次无圆，则认为已穿越

float circ_yz_abs_distance;

int circ_adjust_patience;//调整耐心，待设定
float inaccuracy_tolerance;//允许的误差，待设定
int circ_adjust_times;
bool go_through_circ;
bool last_judgement;
float circ_coordinates[3];//发布的目标的x,y,z坐标
float now_circ_coordinates[3];//用于暂存坐标
int one_step_time;//单次穿环微调时的循环次数
int one_step_patience;//单次穿环微调时的循环耐心,待设定
float largest_stride;//单次穿环微调时容许的最大单个方向移动距离
float min_circ_height;//穿环时最低高度



/*避障算法相关*/
float R_outside,R_inside;                                       //安全半径 [避障算法相关参数]
float p_R;                                                      //大圈比例参数
float p_r;                                                      //小圈比例参数
float distance_c;                                               //最近障碍物距离
float distance_cx,distance_cy;                                  //最近障碍物距离XY
float vel_collision[2];                                        //躲避障碍部分速度
float vel_collision_max;                                        //躲避障碍部分速度限幅
float p_xy;                                                     //追踪部分位置环P
float vel_track[2];                                             //追踪部分速度
float vel_track_max;                                            //追踪部分速度限幅                                                 //降落标志位
std_msgs::Bool flag_collision_avoidance; 

/*-----------------------------视觉识别--------------------------*/
/*全局变量*/
string targetclass = "car";
darknet_ros_msgs::BoundingBox target_box; //全局,目标盒
bool spot_target;//是否寻找到目标
bool trace_confirm;//是否追踪目标
float pic_target[2];//发布的目标的x,y坐标
int reload_time;//丢失追踪目标的次数
int no_target_time;//没有目标的次数


float scan_thresh;//扫描时的置信度阈值（待设定）
float arrival_thresh;//抵达目标时的误差阈值（待设定）
int reload_patience;//丢失追踪目标的耐心(待设定）
int no_target_patience;//没有目标的耐心(待设定）

float search_safe_distance;//搜素目标点时巡航安全限度
/*相机内参*/
float fx = 533.3333;					
float fy = 533.3333;
float cx = 320;
float cy = 240;
float dy;
float dx;

/*-----------------------------函数声明--------------------------*/
void rotation_yaw(float yaw_angle, float input[2], float output[2]); 
void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
void lidar_cb(const sensor_msgs::LaserScan::ConstPtr &scan);
void darknet_box_cb(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg);//darknet回调函数
void calculate_xyz();//计算坐标
void cal_xy_max_distance();
float find_distance(float x, float y, float x0, float y0);
void printf_param(); // 打印各项参数以供检查
void cal_min_distance();  //计算前后左右四向最小距离

//collision_avoidance
void track(float target_x, float target_y);
void collision_avoidance(float target_x,float target_y);                            //避障函数。目标点位置
float satfunc(float data, float Max);                                               //饱和函数。传输数据，阈值
void printf();                                                                       //打印函数                                                              //打印各项参数以供检查
void outt(double ang);	                                        //改进避障功能函数
int test1(float, float);		                                //改进避障函数
int moveto(float, float);

//test_go_circ
//void ocvd_cb(const px4_command::hough::ConstPtr &msg);
//void calculate_circ_xyz();
void recalculate_theoretical_distance();
                    
/*-----------------------------主循环--------------------------*/
int main(int argc, char **argv)
{
    ros::init(argc, argv, "final");
    ros::NodeHandle nh("~");
    ros::Rate rate(1.0);
    ros::Publisher Command_pub = nh.advertise<px4_command::command>("/px4/command", 10);
    ros::Subscriber lidar_sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1000, lidar_cb);
	ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 100, pos_cb);
    ros::Subscriber darknet_box_sub = nh.subscribe<darknet_ros_msgs::BoundingBoxes>("/darknet_ros/bounding_boxes", 1, darknet_box_cb);
    //ros::Subscriber ocvd_sub = nh.subscribe<px4_command::hough>("/hough_sender", 100, ocvd_cb);
    /*参数载入*/    
    //读取参数表中的参数
    nh.param<float>("target_x", target_x, 1.0); //dyx
    nh.param<float>("target_y", target_y, 0.0); //dyx
    nh.param<float>("initial_target_x", initial_target_x, 0); //正对圆环方向的目标点x
    nh.param<float>("initial_target_y", initial_target_y, 0); //正对圆环方向的目标点y
    nh.param<float>("R_outside", R_outside, 1.0);
    nh.param<float>("R_inside", R_inside, 0.6);

    nh.param<float>("p_xy", p_xy, 0.5);

    nh.param<float>("vel_track_max", vel_track_max, 0.5);

    nh.param<float>("p_R", p_R, 0.0);
    nh.param<float>("p_r", p_r, 0.0);

    nh.param<float>("vel_collision_max", vel_collision_max, 0.0);
    nh.param<float>("vel_sp_max", vel_sp_max, 0.0);

    nh.param<int>("range_min", range_min, 0.0);
    nh.param<int>("range_max", range_max, 0.0);
    nh.param<float>("fly_height", fly_height, 0.5);
    nh.param<float>("bj", bj, 0.44);

    nh.param<float>("search_safe_distance", search_safe_distance, 0.4);
    //视觉识别
	nh.param<float>("scan_thresh", scan_thresh, 0.30);
	nh.param<float>("arrival_thresh", arrival_thresh, 0.05);
	nh.param<int>("reload_patience", reload_patience, 20);
	nh.param<int>("no_target_patience", no_target_patience, 200);

    //ocvd
    nh.param<float>("presetCir_x", presetCir_x, 0);
    nh.param<float>("presetCir_y", presetCir_y, 0);
    nh.param<float>("max_detection_distance", max_detection_distance, 0.7);
    nh.param<float>("min_detection_distance", min_detection_distance, 0.3);
    nh.param<float>("near_min_distance", near_min_distance, 0.3);
    
    nh.param<int>("circ_adjust_patience", circ_adjust_patience, 50);
    nh.param<float>("inaccuracy_tolerance", inaccuracy_tolerance, 0.1);
    nh.param<int>("one_step_patience", one_step_patience, 8);
    nh.param<float>("largest_stride", largest_stride, 0.4);
    nh.param<float>("min_circ_height", min_circ_height, 0.3);

    //打印现实检查参数
    printf_param();

    //check paramater
    int check_flag;
    //输入1,继续，其他，退出程序
    cout << "Please check the parameter and setting，1 for go on， else for quit: "<<endl;
    cin >> check_flag;
    if(check_flag != 1) return -1;

    //check arm
    int Arm_flag;
    cout<<"Whether choose to Arm? 1 for Arm, 0 for quit"<<endl;
    cin >> Arm_flag;
    if(Arm_flag == 1)
    {
        Command_now.command = Arm;
        Command_pub.publish(Command_now);
    }
    else return -1;

    //check takeoff
    int Take_off_flag;
	cout << "Whether choose to Takeoff? 1 for Takeoff, 0 for quit" << endl;
	cin >> Take_off_flag;
	if (Take_off_flag == 1)
	{
		Command_now.command = Takeoff;
		Command_now.pos_sp[0] = 0;
		Command_now.pos_sp[1] = 0; // ENU frame
		Command_now.pos_sp[2] = fly_height;
		Command_now.yaw_sp = 0;
		Command_pub.publish(Command_now);
	}
	else
		return -1;

    //check start collision_avoid
    int start_flag;
    cout<<"Whether choose to Start mission? 1 for start, 0 for quit"<<endl;
    cin >> start_flag;
    if(start_flag != 1) return -1;

    for(int i=0;i<7;i++)
    {   
        ros::spinOnce();

        Command_now.command = Move_ENU;
        Command_now.comid = comid;
        comid++;
        Command_now.sub_mode = 0;
        Command_now.pos_sp[0] = 0;
        Command_now.pos_sp[1] = 0; // ENU frame
        Command_now.pos_sp[2] = fly_height;
        Command_now.yaw_sp = 0;
        Command_pub.publish(Command_now);

        rate.sleep();
    }

    //初值
    vel_track[0]= 0;
    vel_track[1]= 0;

    vel_collision[0]= 0;
    vel_collision[1]= 0;

    vel_sp_body[0]= 0;
    vel_sp_body[1]= 0;

    vel_sp_ENU[0]= 0;
    vel_sp_ENU[1]= 0;

    flag_land = 0;

    int comid = 1;
    int flag1  = 0;

    float Uav_x = pos_drone.pose.position.x;
    float Uav_y = pos_drone.pose.position.y;

    while (ros::ok())
    {
        //moveto(target_x, target_y);
         ros::spinOnce();

        int flag = test1(initial_target_x, initial_target_y);
        if(flag == 2 && abs(distance_cy) > 0.1)
        {
            collision_avoidance(initial_target_x, initial_target_y);
        } //紧急避障
        else if(flag == 0){
            track(initial_target_x, initial_target_y);
        }
        else
        {
            track(c_target.x, c_target.y);
        }

        Command_now.command = Move_ENU;     //机体系下移动
        Command_now.comid = comid;
        comid++;
        Command_now.sub_mode = 2; // xy 速度控制模式 z 位置控制模式
        Command_now.vel_sp[0] =  vel_sp_ENU[0];
        Command_now.vel_sp[1] =  vel_sp_ENU[1];  //ENU frame
        Command_now.pos_sp[2] =  fly_height;
        Command_now.yaw_sp = 0 ;
        Command_pub.publish(Command_now);
        
        ros::spinOnce();
        abs_distance = find_distance(pos_drone.pose.position.x, pos_drone.pose.position.y, initial_target_x, initial_target_y);
        if(abs_distance < 0.15 )
        {
            cout << "*****************************************************************" << endl;
            cout << "UAV has gone to target point:" << initial_target_x << "," << initial_target_y << endl;
            cout << "now position:" << pos_drone.pose.position.x << "," << pos_drone.pose.position.y << endl;
            cout << "change to act circle_crossing task.......";
            cout << "*****************************************************************" << endl;


            Uav_x = pos_drone.pose.position.x;
            Uav_y = pos_drone.pose.position.y;

            for(int i = 3; i > 0 ;i--){
                Command_now.command = Move_ENU;     //机体系下移动
                Command_now.comid = comid;
                comid++;
                Command_now.sub_mode = 0; // xy 速度控制模式 z 位置控制模式
                Command_now.pos_sp[0] =  Uav_x;
                Command_now.pos_sp[1] =  Uav_y;  //ENU frame
                Command_now.pos_sp[2] =  fly_height;
                Command_now.yaw_sp = 0 ;
                Command_pub.publish(Command_now);
            }

            ros::spinOnce();
            cout << "Command_vel:" << Command_now.vel_sp[0]  << "," << Command_now.vel_sp[1]  << endl;
            cout << "current position:" << pos_drone.pose.position.x << "," << pos_drone.pose.position.y << endl;
            break;
        }
        else
        {
            cout << "acting track... comid:" << comid << endl;
            cout << "abs_distance: " << abs_distance << "\n";
			cout << "pos_x: " << pos_drone.pose.position.x << " pos_y: " << pos_drone.pose.position.y << "\n";
            cout << "now_height:  " << pos_drone.pose.position.z << endl; 
            rate.sleep();
            continue;
        } 
    }

    while(ros::ok())
    {
        Command_now.command = Move_ENU;     //机体系下移动
        Command_now.comid = comid;
        comid++;
        Command_now.sub_mode = 0; // xy 速度控制模式 z 位置控制模式
        Command_now.pos_sp[0] =  2.7;
        Command_now.pos_sp[1] =  0;  //ENU frame
        Command_now.pos_sp[2] =  fly_height;
        Command_now.yaw_sp = 0 ;
        Command_pub.publish(Command_now);

        ros::spinOnce();
        abs_distance = find_distance(pos_drone.pose.position.x, pos_drone.pose.position.y, 2.8, 0);
        if(abs_distance < 0.15 )
        {
            cout << "*****************************************************************" << endl;
            cout << "UAV has gone to target point:" << 2.7 << "," << 0 << endl;
            cout << "now position:" << pos_drone.pose.position.x << "," << pos_drone.pose.position.y << endl;
            cout << "change to act collision_avoidance task.......";
            cout << "*****************************************************************" << endl;
            break;
        }
    }


    cout << "*****************************************************************" << endl;
    cout << "UAV has crossed the circle!:" << initial_target_x << "," << now_circ_coordinates[1] << ", " << now_circ_coordinates[2] << endl;
    cout << "now position:" << pos_drone.pose.position.x << "," << pos_drone.pose.position.y << ", " << pos_drone.pose.position.z << endl;
    cout << "change to act travelling task.......";
    cout << "*****************************************************************" << endl;

    while (ros::ok())
    {
        //moveto(target_x, target_y);
         ros::spinOnce();

        int flag = test1(target_x, target_y);
        if(flag == 2 && abs(distance_cy) > 0.1)
        {
            collision_avoidance(target_x, target_y);
        } //紧急避障
        else if(flag == 0){
            track(target_x, target_y);
        }
        else
        {
            track(c_target.x, c_target.y);
        }

        Command_now.command = Move_ENU;     //机体系下移动
        Command_now.comid = comid;
        comid++;
        Command_now.sub_mode = 2; // xy 速度控制模式 z 位置控制模式
        Command_now.vel_sp[0] =  vel_sp_ENU[0];
        Command_now.vel_sp[1] =  vel_sp_ENU[1];  //ENU frame
        Command_now.pos_sp[2] =  fly_height;
        Command_now.yaw_sp = 0 ;
        Command_pub.publish(Command_now);
        
        abs_distance = find_distance(pos_drone.pose.position.x, pos_drone.pose.position.y, target_x, target_y);
        if(abs_distance < 0.15 )
        {
            cout << "*****************************************************************" << endl;
            cout << "UAV has gone to target point:" << target_x << "," << target_y << endl;
            cout << "now position:" << pos_drone.pose.position.x << "," << pos_drone.pose.position.y << endl;
            cout << "change to act travelling task.......";
            cout << "*****************************************************************" << endl;

            for(int i = 3; i > 0 ;i--){
                Command_now.command = Move_ENU;     //机体系下移动
                Command_now.comid = comid;
                comid++;
                Command_now.sub_mode = 2; // xy 速度控制模式 z 位置控制模式
                Command_now.vel_sp[0] =  0;
                Command_now.vel_sp[1] =  0;  //ENU frame
                Command_now.pos_sp[2] =  fly_height;
                Command_now.yaw_sp = 0 ;
                Command_pub.publish(Command_now);
                rate.sleep();
            }

            rate.sleep();
            ros::spinOnce();
            cout << "Command_vel:" << Command_now.vel_sp[0]  << "," << Command_now.vel_sp[1]  << endl;
            cout << "current position:" << pos_drone.pose.position.x << "," << pos_drone.pose.position.y << endl;
            break;
        }
        else
        {
            cout << "acting track... comid:" << comid << endl;
            cout << "abs_distance: " << abs_distance << "\n";
			cout << "pos_x: " << pos_drone.pose.position.x << " pos_y: " << pos_drone.pose.position.y << "\n";
            cout << "now_height:  " << pos_drone.pose.position.z << endl; 
            rate.sleep();
            continue;
        } 
    }



    //开始视觉寻找
    ros::spinOnce();
    spot_target=0;
    trace_confirm=0;
    reload_time=0;
    no_target_time=0;

    cal_xy_max_distance();
    cout << "forward_max:" << search_distance[0] <<endl;
    cout << "backward_max:" << search_distance[1] <<endl;
    cout << "Left_max:" << search_distance[2] <<endl;
    cout << "Right_max:" << search_distance[3] <<endl;
    cout << "now_flight: " << pos_drone.pose.position.z << endl;

    Uav_x = pos_drone.pose.position.x;
    Uav_y = pos_drone.pose.position.y;
    	
    while(ros::ok){
        ros::spinOnce();
	
	    cout << "now_fly_height: " << pos_drone.pose.position.z << endl;	

        if(spot_target)
        {
            cout << "spotted target! approaching..." <<endl;
            no_target_time=0;
            calculate_xyz();
	        cout << "pic_target:" << pic_target[0] << " " << pic_target[1] << endl;
            if(abs_distance < arrival_thresh)
            {
                ros::spinOnce();
                //逐步降落并继续计算与调整，此处应有循环
                //zzr:小于误差容限的话直接降落就好了吧？
                Command_now.command = Land;
                while (ros::ok())
                {
                    Command_pub.publish(Command_now);
                    rate.sleep();
                    cout << "Land"<<endl;
                }
                rate.sleep();
                //完成降落后break出去
                cout << "Mission completed"<<endl;
                return 0;
            }
            else{
                //按照pic_target的坐标发布命令并向那里前进一步,此处无需循环
                ros::spinOnce();

                Command_now.command = Move_ENU;
                Command_now.comid = comid;
                comid++;
                Command_now.sub_mode = 0; // xy 速度控制模式 z 位置控制模式
                Command_now.pos_sp[0] = pic_target[0];
                Command_now.pos_sp[1] = pic_target[1]; // ENU frame
                Command_now.pos_sp[2] = fly_height;//zzr:这里感觉可以降低高度
                Command_now.yaw_sp = 0;
                Command_pub.publish(Command_now);
		        cout << "spot target -- "<< endl;
                cout << "find target...approaching..." << endl;
                cout << "pic_target_xy:" << pic_target[0] << " " << pic_target[1] <<endl;
                cout << "pos_drone:" << pos_drone.pose.position.x << " " << pos_drone.pose.position.y << endl;
                cout << "dx:" << dx << "dy: "<< dy << endl;
                cout << "comid:" << comid << "   abs_distance:" <<abs_distance << endl;
                //可以在这里输出距离，方便操作者debug
                rate.sleep();
            }
            
        }
        else{
            no_target_time++;
        }

        if(reload_time>reload_patience)
        {
            //升高高度找或者降低scan_thresh或者随便降落算了
            //zzr:左右巡航寻找,先向右后向左
            Command_now.command = Land;
            while (ros::ok())
            {
                Command_pub.publish(Command_now);
                rate.sleep();
                cout << "Land"<<endl;
            }
            rate.sleep();
            cout << "reload_target!!!reload_target_time:"<< reload_time <<endl;
            return 0;
            //no_target_time=0;
            
        }
        if(no_target_time>no_target_patience)
        {
            ros::spinOnce();
	        //cout << "reloading... no:" << reload_time <<endl;
            Uav_x = pos_drone.pose.position.x;
            Uav_y = pos_drone.pose.position.y;
            while(ros::ok())
            {
                ros::spinOnce();
                //飞机向右平移，步长0.1，扫描
                if(pos_drone.pose.position.y > search_distance[3] && pos_drone.pose.position.y < search_distance[2] && 
                    pos_drone.pose.position.x < search_distance[0] && pos_drone.pose.position.x > search_distance[1])
                {
                    Command_now.command = Move_ENU;
                    Command_now.comid = comid;
                    comid++;
                    Command_now.sub_mode = 0;
                    Command_now.pos_sp[0] = Uav_x;
                    Command_now.pos_sp[1] = Uav_y - 0.2; // ENU frame
                    Command_now.pos_sp[2] = fly_height;//zzr:这里感觉可以降低高度
                    Command_now.yaw_sp = 0;
                    Command_pub.publish(Command_now);
                    cout << "approaching xy:" << Uav_x << " , " << Uav_y - 0.1 <<endl;
                    abs_distance = find_distance(pos_drone.pose.position.x, pos_drone.pose.position.y, Uav_x, Uav_y - 0.2);
                    if(abs_distance < 0.1)
                    {break;}
                    rate.sleep();
                }
                else
                {
                    Command_now.command = Move_ENU;
                    Command_now.comid = comid;
                    comid++;
                    Command_now.sub_mode = 2; // xy 速度控制模式 z 位置控制模式
                    Command_now.vel_sp[0] = 0;
                    Command_now.vel_sp[1] = 0; // ENU frame
                    Command_now.pos_sp[2] = fly_height;//zzr:这里感觉可以降低高度
                    Command_now.yaw_sp = 0;
                    for(int i=0; i<10; i++)
                        Command_pub.publish(Command_now);
                    cout << "missing target... current xy:" << pos_drone.pose.position.x << " , " << pos_drone.pose.position.y <<endl;
                    rate.sleep();
                    break;
                }
            }
            rate.sleep();
            no_target_time=0;
        }
    }
    return 0;
}


/*-----------------------------函数定义--------------------------*/
int moveto(float a, float b)
{
    ros::spinOnce();

    bool flag = test1(a, b);
	if(flag == 2 && abs(distance_cy) > 0.1)
	{
		collision_avoidance(a, b);
	} //紧急避障
	else if(flag == 0){
		track(a, b);
	}
	else
	{
		track(c_target.x, c_target.y);
	}

    Command_now.command = Move_ENU;     //机体系下移动
    Command_now.comid = comid;
    comid++;
    Command_now.sub_mode = 2; // xy 速度控制模式 z 位置控制模式
    Command_now.vel_sp[0] =  vel_sp_ENU[0];
    Command_now.vel_sp[1] =  vel_sp_ENU[1];  //ENU frame
    Command_now.pos_sp[2] =  fly_height;
    Command_now.yaw_sp = 0 ;

}

void rotation_yaw(float yaw_angle, float input[2], float output[2])
{
    output[0] = input[0] * cos(yaw_angle) - input[1] * sin(yaw_angle);
    output[1] = input[0] * sin(yaw_angle) + input[1] * cos(yaw_angle);
}    

void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
	pos_drone = *msg;
	// Read the Quaternion from the Mavros Package [Frame: ENU]
	Eigen::Quaterniond q_fcu_enu(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
	q_fcu = q_fcu_enu;
	// Transform the Quaternion to Euler Angles
	Euler_fcu = quaternion_to_euler(q_fcu);
}

void lidar_cb(const sensor_msgs::LaserScan::ConstPtr &scan)
{
	sensor_msgs::LaserScan Laser_tmp;
	Laser_tmp = *scan;
	Laser = *scan;
	int count; // count = 359
	count = Laser.ranges.size();

	// 剔除inf的情况
	for (int i = 0; i < count; i++)
	{
		// 判断是否为inf
		int a = isinf(Laser_tmp.ranges[i]);
		// 如果为inf,则赋值上一角度的值
		if (a == 1)
		{
			if (i == 0)
			{
				Laser_tmp.ranges[i] = Laser_tmp.ranges[count - 1];
			}
			else
			{
				Laser_tmp.ranges[i] = Laser_tmp.ranges[i - 1];
			}
		}
	}
	for (int i = 0; i < count; i++)
	{
		if (i + 180 > 359)
			Laser.ranges[i] = Laser_tmp.ranges[i - 180];
		else
			Laser.ranges[i] = Laser_tmp.ranges[i + 180];
		// cout<<"tmp: "<<i<<" l:"<<Laser_tmp.ranges[i]<<"|| Laser: "<<Laser.ranges[i]<<endl;
	}
	// cout<<"//////////////"<<endl;
	// 计算前后左右四向最小距离
	cal_min_distance();
}

void darknet_box_cb(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg){
    trace_confirm=0;
    if(spot_target)
    {
        for(int i=0;i<msg->bounding_boxes.size();i++)
	{
            if(msg->bounding_boxes[i].probability>scan_thresh&&msg->bounding_boxes[i].Class == targetclass)
	    {
                std::cout<<"tracing target"<<msg->bounding_boxes[i].Class<<std::endl;
                target_box=msg->bounding_boxes[i];
                trace_confirm=1;
                break;
            }
        }
        
        if(trace_confirm)
	{
            return;
        }
	else
	{
	    spot_target=0;
	    reload_time++;
	    std::cout<<"target lost,now rescan"<<std::endl;
	}
    }

   
    for(int i=0;i<msg->bounding_boxes.size();i++){
        if(msg->bounding_boxes[i].probability>scan_thresh&&msg->bounding_boxes[i].Class == targetclass){
            std::cout<<"found target"<<msg->bounding_boxes[i].Class<<std::endl;
            spot_target=1;
            target_box=msg->bounding_boxes[i];
            break;
        }
        spot_target=0;
    }
    return;
}

void calculate_xyz(){
	int pic_center_x,pic_center_y;
	pic_center_x = (target_box.xmin+target_box.xmax)/2;
    pic_center_y = (target_box.ymin+target_box.ymax)/2;
	dy=-fly_height*(pic_center_x-cx)/fx;
	dx=-fly_height*(pic_center_y-cy)/fy;
	cout << "xmin, ymin:" << target_box.xmin << " " << target_box.ymin << endl;
    cout << "xmax, ymax:" << target_box.xmax << " " << target_box.ymax << endl;
	pic_target[0]=pos_drone.pose.position.x+dx;
    pic_target[1]=pos_drone.pose.position.y+dy;
 	abs_distance = sqrt((pos_drone.pose.position.x - pic_target[0]) * (pos_drone.pose.position.x - pic_target[0]) + (pos_drone.pose.position.y - pic_target[1]) * (pos_drone.pose.position.y - pic_target[1]));
	
}

void cal_xy_max_distance()
{
    //int range[4] = {0, 180, 270, 90}; //270 左 90 右

    search_distance[0] =  pos_drone.pose.position.x +  Laser.ranges[0] - search_safe_distance; //yaw=0处  Laser.ranges[0]
    search_distance[1] =  pos_drone.pose.position.x -  Laser.ranges[180] + search_safe_distance; //yaw=180处  Laser.ranges[180]
    search_distance[2] =  pos_drone.pose.position.y +  Laser.ranges[270] - search_safe_distance; //yaw=270处   Laser.ranges[270]
    search_distance[3] =  pos_drone.pose.position.x -  Laser.ranges[90] + search_safe_distance; //yaw=90处    Laser.ranges[90]

}

float find_distance(float x, float y, float x0, float y0)
{
	return sqrt((x - x0) * (x - x0) + (y - y0) * (y - y0));
}

void printf_param()
{
	cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Parameter <<<<<<<<<<<<<<<<<<<<<<<<<<<" << endl;
	cout << "range_min : " << range_min << endl;
	cout << "range_max : " << range_max << endl;
    cout << "search_safe_distance : " << search_safe_distance << endl;

	cout << "target_x : " << target_x << endl;
	cout << "target_y : " << target_y << endl;
    cout << "fly_height : " << fly_height << endl;

	cout << "initial_target_x : " << initial_target_x << endl;
	cout << "initial_target_y : " << initial_target_y << endl;

    cout << "bj : " << bj << endl;

    cout << "R_outside : "<< R_outside << endl;
    cout << "R_inside : "<< R_inside << endl;
    cout << "p_xy : "<< p_xy << endl;
    cout << "p_R : "<< p_R << endl;
    cout << "p_r : "<< p_r << endl;

    cout << "vel_track_max : "<< vel_track_max << endl;
    cout << "vel_collision_max : "<< vel_collision_max << endl;
    cout << "vel_sp_max : "<< vel_sp_max << endl;

	cout << "presetCir_x : " << presetCir_x << endl;
	cout << "presetCir_y : " << presetCir_y << endl;
	cout << "max_detection_distance : " << max_detection_distance << endl;
	cout << "min_detection_distance : " << min_detection_distance << endl;
	cout << "near_min_distance : " << near_min_distance << endl;

    cout << "circ_adjust_patience : " << circ_adjust_patience << endl;
    cout << "inaccuracy_tolerance : " << inaccuracy_tolerance << endl;
    cout << "one_step_patience : " << one_step_patience << endl;
    cout << "largest_stride : " << largest_stride << endl;
    cout << "min_circ_height : " << min_circ_height << endl;

	cout << "scan_thresh:" << scan_thresh <<endl;
    cout << "arrival_thresh:" << arrival_thresh  <<endl;
    cout << "reload_patience:" << reload_patience <<endl;
    cout << "no_target_patience:" << no_target_patience <<endl;
}

void cal_min_distance()
{
	distance_c = Laser.ranges[range_min];
	angle_c = 0;
	for (int i = range_min; i <= range_max; i++)
	{
		if (Laser.ranges[i] < distance_c)
		{
			distance_c = Laser.ranges[i];
			angle_c = i;
		}
	}
}

void track(float target_x, float target_y)
{
	//3. 计算追踪速度
	vel_track[0] = p_xy * (target_x - pos_drone.pose.position.x);
	vel_track[1] = p_xy * (target_y - pos_drone.pose.position.y);
	cout<<"vel_track_x : "<<vel_track[0]<<endl;
	cout<<"vel_track_y : "<<vel_track[1]<<endl;
	
	//速度限幅
	for (int i = 0; i < 2; i++) {
		vel_track[i] = satfunc(vel_track[i], 0.3);
	}
	vel_sp_ENU[0] = vel_track[0];
	vel_sp_ENU[1] = vel_track[1]; //dyx
	cout<<"vel_sp_ENU_x : "<<vel_sp_ENU[0]<<endl;
	cout<<"vel_sp_ENU_y : "<<vel_sp_ENU[1]<<endl;
	// cout<<"-------------------------------------------------"<<endl;
}

void collision_avoidance(float target_x,float target_y)
{
    if (distance_c >= R_outside )
    {
        flag_collision_avoidance.data = false;
    }
    else
    {
        flag_collision_avoidance.data = true;
    }

    vel_track[0] = p_xy * (target_x - pos_drone.pose.position.x);
    vel_track[1] = p_xy * (target_y - pos_drone.pose.position.y);

    for (int i = 0; i < 2; i++)
    {
        vel_track[i] = satfunc(vel_track[i],vel_track_max);
    }
    vel_collision[0]= 0;
    vel_collision[1]= 0;

    //4. 避障策略
    if(flag_collision_avoidance.data == true)
    {
        distance_cx = distance_c * cos(angle_c/180*3.1415926);  //最近障碍物信息
        distance_cy = distance_c * sin(angle_c/180*3.1415926);

        float F_c;              //根据（比例系数*距离）传递的位置信息

        F_c = 0;

        if(distance_c > R_outside)
        {
            //对速度不做限制
            vel_collision[0] = vel_collision[0] + 0;
            vel_collision[1] = vel_collision[1] + 0;
            cout << " Forward Outside "<<endl;
        }

        //小幅度抑制移动速度
        if(distance_c > R_inside && distance_c <= R_outside)
        {
            F_c = p_R * (R_outside - distance_c);

        }

        //大幅度抑制移动速度
        if(distance_c <= R_inside )
        {
            F_c = p_R * (R_outside - R_inside) + p_r * (R_inside - distance_c);
        }

        if(distance_cx > 0)
        {
            vel_collision[0] = vel_collision[0] - F_c * distance_cx /distance_c;
        }else{
            vel_collision[0] = vel_collision[0] - F_c * distance_cx /distance_c;
        }

        if(distance_cy > 0)
        {
            vel_collision[1] = vel_collision[1] - F_c * distance_cy / distance_c;
        }else{
            vel_collision[1] = vel_collision[1] - F_c * distance_cy /distance_c;
        }
        //避障速度限幅
        for (int i = 0; i < 2; i++)
        {
            vel_collision[i] = satfunc(vel_collision[i],vel_collision_max);
        }
    }

    vel_sp_body[0] = vel_track[0] + vel_collision[0];
    vel_sp_body[1] = vel_track[1] + vel_collision[1]; 

    for (int i = 0; i < 2; i++)
    {
        vel_sp_body[i] = satfunc(vel_sp_body[i],vel_sp_max);
    }
    rotation_yaw(Euler_fcu[2],vel_sp_body,vel_sp_ENU);
}

float satfunc(float data, float Max)
{
    if(abs(data)>Max) return ( data > 0 ) ? Max : -Max;
    else return data;
}

int test1(float target_x, float target_y)
{//返回true则避障，false则不避障
	Node nd;//目标点
	nd.x = target_x - pos_drone.pose.position.x;
	nd.y = target_y - pos_drone.pose.position.y;
	Node nd_zaw;//障碍物
	distance_cx = distance_c * cos(angle_c / 180 * 3.1415926);
	distance_cy = distance_c * sin(angle_c / 180 * 3.1415926);
	//转系!!!!!!!
	float tt[2] = {distance_cx, distance_cy};
	float ttt[2];
	rotation_yaw(Euler_fcu[2], tt, ttt);
	distance_cx = ttt[0];
	distance_cy = ttt[1];

	nd_zaw.x = distance_cx; 
    nd_zaw.y = distance_cy;
	double alpha1 = atan2(nd_zaw.y, nd_zaw.x);
	double alpha2 = asin(abs(bj/distance_c));
	double tp1 = alpha1 + alpha2, tp2 = alpha1 - alpha2;//两个的相对的角度
	if(distance_c  <= (bj))//需要实际测量参数来决定！！！！！！！！
	{
		outt(alpha1);
		return 2;
	}
	double tmpp = sqrt(distance_c * distance_c - bj * bj) * 1.1;
	Node ans1, ans2;
	ans2.x = tmpp * cos(tp1); ans2.y = tmpp * sin(tp1);
	ans1.x = tmpp * cos(tp2); ans1.y = tmpp * sin(tp2);
	//算出切点的相对坐标
        double tp3 = atan2(nd.y , nd.x);

	if (tp3 >= tp2 && tp3 <= tp1)
	{
        cout << "New algorithm, Conical Obstacle Avoidance. "<<endl;
		if(tp3 <= alpha1)
		{
			c_target.x = ans1.x + pos_drone.pose.position.x;
			c_target.y = ans1.y + pos_drone.pose.position.y;
            cout << "c_target.x : "<< c_target.x<<endl;
			cout << "c_target.y : "<< c_target.y<<endl;
		}
		else
		{
			c_target.x = ans2.x + pos_drone.pose.position.x;
			c_target.y = ans2.y + pos_drone.pose.position.y;
            cout << "c_target.x : "<< c_target.x<<endl;
			cout << "c_target.y : "<< c_target.y<<endl;
		}
		return 1;
	}
    else
    {
        cout<< "Normal flying "<<endl;
    }
	return 0;
}

void outt(double ang)
{
	cout << " TOO CLoSE !!!!!!!!!!" << endl;
	c_target.x = pos_drone.pose.position.x - abs(bj) * cos(ang) * 0.5;
	c_target.y = pos_drone.pose.position.y - abs(bj) * sin(ang) * 0.5;
    cout << "c_target.x : "<< c_target.x<<endl;
    cout << "c_target.y : "<< c_target.y<<endl;
	return;
}

void printf()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>collision_avoidance<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    cout << "Minimun_distance : "<<endl;
    cout << "Distance : " << distance_c << " [m] "<<endl;
    cout << "Angle :    " << angle_c    << " [du] "<<endl;
    cout << "distance_cx :    " << distance_cx    << " [m] "<<endl;
    cout << "distance_cy :    " << distance_cy    << " [m] "<<endl;
    if(flag_collision_avoidance.data == true)
    {
        cout << "Collision avoidance Enabled "<<endl;
    }
    else
    {
        cout << "Collision avoidance Disabled "<<endl;
    }
    cout << "vel_track_x : " << vel_track[0] << " [m/s] "<<endl;
    cout << "vel_track_y : " << vel_track[1] << " [m/s] "<<endl;

    cout << "vel_collision_x : " << vel_collision[0] << " [m/s] "<<endl;
    cout << "vel_collision_y : " << vel_collision[1] << " [m/s] "<<endl;

    cout << "vel_sp_x : " << vel_sp_ENU[0] << " [m/s] "<<endl;
    cout << "vel_sp_y : " << vel_sp_ENU[1] << " [m/s] "<<endl;
    cout << "pos_drone.pose.position.x : " << pos_drone.pose.position.x << endl;
	cout << "pos_drone.pose.position.y : " << pos_drone.pose.position.y << endl;
	cout << "tx ty : " << target_x << "  " << target_y << endl;
}

//void ocvd_cb(const px4_command::hough::ConstPtr &msg){
//    target_circ=*msg;
//    return;
//}

/*void calculate_circ_xyz(){
    
	float circ_dy=-theoretical_distance_to_circ*(target_circ.x-target_circ.cx)/target_circ.fx + 0.07;
	float circ_dz=-theoretical_distance_to_circ*(target_circ.y-target_circ.cy)/target_circ.fy ;
	float circ_dx=theoretical_distance_to_circ;
	circ_coordinates[0]=pos_drone.pose.position.x + circ_dx;//圆环x坐标
    circ_coordinates[1]=pos_drone.pose.position.y + circ_dy;//圆环y坐标
    circ_coordinates[2]=pos_drone.pose.position.z + circ_dz;//圆环z坐标
 	circ_yz_abs_distance = sqrt((circ_dy) * (circ_dy) + (circ_dz * circ_dz));  //?

	cout << "adjusting pos!!!....................." <<endl;
    cout << "theoretical_distance_to_circ: " << theoretical_distance_to_circ <<endl;
    cout<<"tar_circ_x:"<<target_circ.x<<"    tar_circ_y:"<<target_circ.y<<endl;
	cout << "dx: " << circ_dx << "dy:" << circ_dy << "dz:" << circ_dz <<endl;
    cout << "circle position: " << circ_coordinates[0] << " , "<< circ_coordinates[1] << " , " << circ_coordinates[2] << endl;
}*/

void recalculate_theoretical_distance(){
    //大概是依据IMU，根据机体现在所在坐标与事先设定的坐标计算理论x方向距离，并保存在theoretical_distance_to_circ
    ros::spinOnce();

    float current_dis;
    current_dis = (- abs(pos_drone.pose.position.x)  + abs(presetCir_x));  
    cout << "calculating throretical_distance......  current_dis: " << current_dis << endl;
    theoretical_distance_to_circ = current_dis;
}



