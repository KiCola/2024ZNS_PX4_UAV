//测试视觉降落的算法

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

#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
using namespace std;


/*------------------------------------全局变量-----------------------------------------------*/
/*指令及常数*/
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
px4_command::command Command_now;

const float Pi = acos(-1.0);
const float INF = 100;

int comid;  //指令id

/*输入数据*/
sensor_msgs::LaserScan Laser;		  // 激光雷达点云数据
geometry_msgs::PoseStamped pos_drone; // 无人机当前位置
Eigen::Quaterniond q_fcu;
Eigen::Vector3d Euler_fcu;

float target_x; // 期望位置_x
float target_y; // 期望位置_y
float fly_height;

int range_min;			// 激光雷达探测范围 最小角度
int range_max;			// 激光雷达探测范围 最大角度
float last_time = 0;
float distance_c;
int angle_c = 0;

float search_distance[4]; // 视觉寻找降落点 [0][1] x方向--前后最大值 [2][3] y方向--左右最大值 
int flag_land;					// 降落标志位
float abs_distance;				// 到目标点的距离

float vel_sp_body[2];					 // 总速度
float vel_sp_ENU[2];					 // ENU下的总速度
float vel_sp_max;						 // 总速度限幅

/*-----------------------------视觉识别--------------------------*/
/*全局变量*/
string targetclass = "H";
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

//函数声明
void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
void lidar_cb(const sensor_msgs::LaserScan::ConstPtr &scan);
void darknet_box_cb(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg);//darknet回调函数
//void darknet_found_cb(const std_msgs::Int8::ConstPtr &msg);
void calculate_xyz();//计算坐标
void cal_xy_max_distance();
float find_distance(float x, float y, float x0, float y0);
void printf_param(); // 打印各项参数以供检查
void cal_min_distance();  //计算前后左右四向最小距离

//主函数
int main(int argc, char **argv)
{
    ros::init(argc, argv, "vision_test");
    ros::NodeHandle nh("~");
    ros::Rate rate(1.0);
    ros::Publisher Command_pub = nh.advertise<px4_command::command>("/px4/command", 10);
    ros::Subscriber lidar_sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1000, lidar_cb);
	ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 100, pos_cb);

    //ros::Subscriber darknet_found_sub = nh.subscribe<std_msgs::Int8>("/darknet_ros/found_object/", 1, darknet_found_cb);
    ros::Subscriber darknet_box_sub = nh.subscribe<darknet_ros_msgs::BoundingBoxes>("/darknet_ros/bounding_boxes", 1, darknet_box_cb);
    /*参数载入*/
    nh.param<float>("target_x", target_x, 0);//目标点位x坐标
    nh.param<float>("target_y", target_y, 0);//目标点位y坐标
    nh.param<int>("range_min", range_min, 0.0);//扫描角度最小值，一般为0
	nh.param<int>("range_max", range_max, 0.0);//扫描角度最大值，一般为359
    nh.param<float>("fly_height", fly_height, 0.5);
    nh.param<float>("search_safe_distance", search_safe_distance, 0.4);

    //视觉识别
	nh.param<float>("scan_thresh", scan_thresh, 0.30);
	nh.param<float>("arrival_thresh", arrival_thresh, 0.05);
	nh.param<int>("reload_patience", reload_patience, 20);
	nh.param<int>("no_target_patience", no_target_patience, 200);



    // check paramater
    printf_param();

	int check_flag;
	// 输入1,继续,其他,退出程序
	cout << "Please check the parameter and setting,1 for go on, else for quit: " << endl;
	cin >> check_flag;
	if (check_flag != 1)
		return -1;

	// check arm
	int Arm_flag;
	cout << "Whether choose to Arm? 1 for Arm, 0 for quit" << endl;
	cin >> Arm_flag;
	if (Arm_flag == 1)
	{
		Command_now.command = Arm;
		Command_pub.publish(Command_now);
	}
	else
		return -1;

	// check takeoff
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

    //是否开始执行任务
    int start_flag;
	cout << "Whether choose to Start mission? 1 for start, 0 for quit" << endl;
	cin >> start_flag;
	if (Take_off_flag != 1)
		return -1;

    
	comid = 1;

	vel_sp_ENU[0] = 0;
	vel_sp_ENU[1] = 0;

	flag_land = 0;

    int i=0;

    //先让飞机飞到目标位置
    while (ros::ok())
	{
		ros::spinOnce();
		Command_now.command = Move_ENU; // 本地坐标系下移动
		Command_now.comid = comid;
		comid++;
		Command_now.sub_mode = 0; // xy 位置控制模式 z 位置控制模式
		Command_now.pos_sp[0] = target_x;
		Command_now.pos_sp[1] = target_y; // ENU frame
		Command_now.pos_sp[2] = fly_height;
		Command_now.yaw_sp = 0;
		Command_pub.publish(Command_now);
		abs_distance = find_distance(pos_drone.pose.position.x, pos_drone.pose.position.y, target_x, target_y);
		if (abs_distance < 0.1) //当抵达目标点0.15m内时，结束指令发送
		{
			break;
		}
		else
		{
            cout << "approaching target..." << comid << "\n";
			cout << "abs_distance: " << abs_distance << "\n";
			cout << "pos_x: " << pos_drone.pose.position.x << " pos_y: " << pos_drone.pose.position.y << "\n";
        }
		rate.sleep();
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

    float Uav_x = pos_drone.pose.position.x;
    float Uav_y = pos_drone.pose.position.y;
    	
    check_flag=0;
	// 输入1,继续,其他,退出程序
	cout << "appearing for vision_land... check (1 for go, else for exit):" << endl;
	cin >> check_flag;
	if (check_flag != 1)
		return -1;


    while(ros::ok){
        ros::spinOnce();

        if(spot_target)
        {
            cout << "spotted target! approaching..." <<endl;
            no_target_time=0;
            calculate_xyz();
	    cout << "pic_target:" << pic_target[0] << " " << pic_target[1] << endl;
            if(abs_distance < arrival_thresh)
            {
                ros::spinOnce();
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
                ros::spinOnce();

                Command_now.command = Move_ENU;
                Command_now.comid = comid;
                comid++;
                Command_now.sub_mode = 0; 
                Command_now.pos_sp[0] = pic_target[0];
                Command_now.pos_sp[1] = pic_target[1];
                Command_now.pos_sp[2] = fly_height;
                Command_now.yaw_sp = 0;
                Command_pub.publish(Command_now);
		        cout << "spot target -- "<< endl;
                cout << "find target...approaching..." << endl;
                cout << "pic_target_xy:" << pic_target[0] << " " << pic_target[1] <<endl;
                cout << "pos_drone:" << pos_drone.pose.position.x << " " << pos_drone.pose.position.y << endl;
                cout << "dx:" << dx << "dy: "<< dy << endl;
                cout << "comid:" << comid << "   abs_distance:" <<abs_distance << endl;
                rate.sleep();
            }
            
        }
        else{
            no_target_time++;
        }

        if(reload_time>reload_patience)
        {
            //升高高度找或者降低scan_thresh或者随便降落算了
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
            
        }
        if(no_target_time>no_target_patience)
        {
            
            ros::spinOnce();
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
                    Command_now.pos_sp[1] = Uav_y - 0.2; 
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
                }
            }
            rate.sleep();
            no_target_time=0;
        }
    }
}


//函数定义
void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
	pos_drone = *msg;
	Eigen::Quaterniond q_fcu_enu(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
	q_fcu = q_fcu_enu;
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
		
	}
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
	float dy=-fly_height*(pic_center_x-cx)/fx;
	float dx=-fly_height*(pic_center_y-cy)/fy;
	
	pic_target[0]=pos_drone.pose.position.x+dx;
    pic_target[1]=pos_drone.pose.position.y+dy;
 	abs_distance = sqrt((pos_drone.pose.position.x - pic_target[0]) * (pos_drone.pose.position.x - pic_target[0]) + (pos_drone.pose.position.y - pic_target[1]) * (pos_drone.pose.position.y - pic_target[1]));
	
}

//计算x,y最大距离
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
	 cout << "target_x : " << target_x << endl;
	 cout << "target_y : " << target_y << endl;

	cout << "range_min : " << range_min << endl;
	cout << "range_max : " << range_max << endl;

	cout << "fly height: " << fly_height << endl;

    cout << "search_safe_distance:" << search_safe_distance << endl;

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






