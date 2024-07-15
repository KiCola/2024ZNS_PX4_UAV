
/*全局变量*/
darknet_ros_msgs::BoundingBox target_box; //全局,目标盒
bool spot_target;//是否寻找到目标
bool trace_confirm;//是否追踪目标
float abs_distance;//离目标的平面欧氏距离
float pic_target[2];//发布的目标的x,y坐标
int reload_time;//丢失追踪目标的次数
int no_target_time;//没有目标的次数

float scan_thresh;//扫描时的置信度阈值（待设定）
float arrival_thresh;//抵达目标时的误差阈值（待设定）
int reload_patience;//丢失追踪目标的耐心(待设定）
int no_target_patience;//没有目标的耐心(待设定）


/*相机内参*/
float fx;
float fy;
float cx;
float cy;
/*------*/
/*--------*/

/*回调函数*/
void darknet_box_cb(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg){
    trace_confirm=0;
    if(spot_target){
        for(int i=0;i<msg->bounding_boxes.size();i++){
            if(msg->bounding_boxes[i].probability>scan_thresh&&msg->bounding_boxes[i].id==target_box.id){
                std::cout<<"tracing target"<<msg->bounding_boxes[i].Class<std::<endl;
                trace_confirm=1;
                break;
            }
        }
        
        if(trace_confirm){
            return;
        }
        else{
            spot_target=0;
            reload_time++;
            std::cout<<"target lost,now rescan"<<std::endl;
        }
    }

    

    for(int i=0;i<msg->bounding_boxes.size();i++){
        if(msg->bounding_boxes[i].probability>scan_thresh&&msg->bounding_boxes[i].id==0){
            std::cout<<"found target"<<msg->bounding_boxes[i].Class<std::<endl;
            spot_target=1;
            target_box=msg->bounding_boxes[i];
            break;
        }
        spot_target=0;
    }
    return;
}

/*计算坐标函数*/
void calculate_xyz(){
	int pic_center_x,pic_center_y;
	pic_center_x = (target_box.xmin+target_box.xmax)/2;
    pic_center_y = (target_box.ymin+target_box.ymax)/2;
	float dy=-nowheight*(pic_center_x-cx)/fx;
	float dx=-nowheight*(pic_center_y-cy)/fy;
	
	pic_target[0]=pos_drone.pose.position.x+dx;
    pic_target[1]=pos_drone.pose.position.y+dy;
 	abs_distance = sqrt((pos_drone.pose.position.x - pic_target[0]) * (pos_drone.pose.position.x - pic_target[0]) + (pos_drone.pose.position.y - pic_target[1]) * (pos_drone.pose.position.y - pic_target[1]));
	
}
/*--------*/

/*主循环*/

spot_target=0;
trace_confirm=0;
reload_time=0;
no_target_time=0;
while(ros::ok){
    ros::spinOnce();

    if(spot_target)
    {
        no_target_time=0;
        calculate_xyz();
        if(abs_distance < arrival_thresh){
            //逐步降落并继续计算与调整，此处应有循环
            Command_now.command = Land;
            flag_land = 1;
            command_pub.publish(Command_now);
            printf();
            rate.sleep()
            //完成降落后break出去
            return 0;
        }
        else{
            //按照pic_target的坐标发布命令并向那里前进一步,此处无需循环
            Command_now.command = Move_ENU;
            Command_now.comid = comid;
            comid++;
            Command_now.pos_sp[0] = pic_target[0];
            Command_now.pos_sp[1] = pic_target[1]; // ENU frame
            Command_now.pos_sp[2] = fly_height;
            Command_now.yaw_sp = 0;
            command_pub.publish(Command_now);
            //可以在这里输出距离，方便操作者debug
            rate.sleep();
        }
        
    }
    else{
        no_target_time++;
    }

    if(reload_time>reload_patience){
        //升高高度找或者降低scan_thresh或者随便降落算了
        reload_time=0;
    }
    if(no_target_time>no_target_patience){
        //升高高度找或者降低scan_thresh或者随便降落算了
        no_target_time=0;
    }

}