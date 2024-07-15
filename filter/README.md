# 最简易滤波器的实现

## 任务需求

由于UAV处于不断地运动过程中，T265鱼眼相机通过OpenCV识别到的圆环可能存在虚空圆，造成穿环识别失败（尽管这种可能性可以尽可能的通过操控无人机距离圆环的位置以及环境光线来规避）

鉴于此，我们开发了filter组件，用以对获取的圆环圆心位置信息做滤波

## 滤波策略

在这个仓库中我们构建了两个最基本的滤波器

### 滤波器1：平均值随机滤波

构建一个数组/队列，初始化存入十个数据，计算数据的平均值

更新滤波数据，根据初始滤波的均值和设定的剔除数据的范围，进行滤波器内数据的更新

对更新后的滤波器内数据做平均值计算

```c++
#include <vector>
#include <numeric>

//vector<float> filtered_queue_y;
//vector<float> filtered_queue_z;
float filtered_queue_y[10];
float filtered_queue_z[10];
float filtered_cir_y;
float filtered_cir_z;
float filter_dy_range = 0.2;
float filter_dz_range = 0.2;
int open_filter;

void filter_circle_pos_init(int key)
{
    if(key > 0)
    {
        int m = 0;
        while(m < 10)
        {
            ros::spinOnce();
            calculate_circ_xyz();

            if(target_circ.y > 0 && target_circ.y < target_circ.rows && target_circ.z >0 && target_circ.z < target_circ.cols)
            {
                //filtered_queue_y.push_back(circ_coordinates[1]);
                //filtered_queue_z.push_back(circ_coordinates[2]);
                filtered_queue_y[m] = circ_coordinate[1];
                filtered_queue_z[m] = circ_coordinate[2];
                cout << "filter initializing ...  now filter queue :" <<  m+1 << endl;
                m++;
            }
            else
            {
                cout << "filter initialing, detected no circle... redetect..." << endl;
                continue;
            }
        }

        cout << "filtered_queue_y.size():" << m << endl;
        cout << "filtered_queue_z.size():" << m << endl; 

        //float aver_y = (float)accumulate(filtered_queue_y.begin(), filtered_queue_y.end(), 0) /10.0;
        //float aver_z = (float)accumulate(filtered_queue_z.begin(). filtered_queue_z.end(), 0) /10.0;

        float sum_y = 0;
        float sum_z = 0;
        for(int j=0;j<10;j++)
        {
            sum_y += filtered_queue_y[j];
            sum_z += filtered_queue_z[j];
        }        
        float aver_y = sum_y/10.0;
        float aver_z = sum_z/10.0;

        filtered_cir_y = aver_y;
        filtered_cir_z = aver_z;

        cout << "init_filtered:" << filtered_cir_y << "   " << filtered_cir_z << endl;
    }

}

void filter_queue_update(bool key)
{
    if(target_circ.y > 0 && target_circ.y < target_circ.rows && target_circ.z >0 && target_circ.z < target_circ.cols)
    {
        if(circ_coordinates[1] > filtered_cir_y - filter_dy_range && circ_coordinates[1] < filtered_cir_y + filter_dy_range
                && circ_coordinates[2] > filtered_cir_z - filter_dz_range && circ_coordinates[2] < filtered_cir_z + filter_dz_range)
        {
            int a = rand()%10;
            //filtered_queue_y.erase(filtered_queue_y.begin() + rand()%10);
            filtered_queue_y[a] = circ_coordinates[1];
            filtered_queue_z[a] = circ_coordinates[2];
            //filtered_queue_y.push_back(circ_coordinates[1]);
        }
        else
        {
            cout << "the flitering data is wrong..." << endl;
            return;
        }

        float sum_y = 0;
        float sum_z = 0;
        for(int j=0;j<10;j++)
        {
            sum_y += filtered_queue_y[j];
            sum_z += filtered_queue_z[j];
        }        
        float aver_y = sum_y/10.0;
        float aver_z = sum_z/10.0;


        filtered_cir_y = aver_y;
        filtered_cir_z = aver_z;
    }
    else
    {
        ROS_WARN("filter data out of scene!!! filter not update!!!");
        return;
    }
} 
```



### 滤波器2： 极差范围滤波

在初始化滤波器的过程中，寻找滤波器数组中距离平均值最远的数据，判断极差是否大于预设的超参数范围，

```c++
float filtered_queue_y[10];
float filtered_queue_z[10];
float filtered_cir_y;
float filtered_cir_z;
float filter_dy_range = 0.2;
float filter_dz_range = 0.2;
float ddd_y_range;
float ddd_z_range;
int open_filter;

void filter_circle_pos_init(int key)
{
    if(key > 0)
    {
        int m = 0;
        while(m < 10)
        {
            ros::spinOnce();
            recalculate_theoretical_distance();
            calculate_circ_xyz();

            if(target_circ.y > 0 && target_circ.y < target_circ.rows && target_circ.x >0 && target_circ.x < target_circ.cols&&abs(circ_coordinates[1])<0.1&&abs(circ_coordinates[2]-0.90)<0.1)
            {
                //filtered_queue_y.push_back(circ_coordinates[1]);
                //filtered_queue_z.push_back(circ_coordinates[2]);
                filtered_queue_y[m] = circ_coordinates[1];
                filtered_queue_z[m] = circ_coordinates[2];
                cout << "filter initializing ...  now filter queue :" <<  m+1 << endl;
                if(m>1)
                {
                    if(abs(filtered_queue_y[m]-filtered_queue_y[m-1]) <1e-6 )
                    {
                    cout << "circ_coordinates didn't change!" << endl;
                    continue;
                    }
                }
                m++;
            }
            else
            {
                
                cout << "filter initialing, detected no valid circle... redetect..." << endl;
                continue;
            }
        }

        cout << "filtered_queue_y.size():" << m << endl;
        cout << "filtered_queue_z.size():" << m << endl; 

        //float aver_y = (float)accumulate(filtered_queue_y.begin(), filtered_queue_y.end(), 0) /10.0;
        //float aver_z = (float)accumulate(filtered_queue_z.begin(). filtered_queue_z.end(), 0) /10.0;

        float sum_y = 0;
        float sum_z = 0;
        for(int j=0;j<10;j++)
        {
            sum_y += filtered_queue_y[j];
            sum_z += filtered_queue_z[j];
        }        
        float aver_y = sum_y/10.0;
        float aver_z = sum_z/10.0;
        int refilter_time = 0;
        int filter_finish_flag = 0;
        cout << "first_filter:" << aver_y << " , " << aver_z << endl;


        while(filter_finish_flag == 0)
        {
            int pos = 0;
            float ddd_y=0;
            float ddd_z=0;
            ddd_y = abs(filtered_queue_y[0] - aver_y);
            ddd_z = abs(filtered_queue_z[0] - aver_z);
            for(int n=1;n<10;n++)
            {
                if(abs(filtered_queue_y[n] - aver_y) > ddd_y)
                {
                    ddd_y = abs(filtered_queue_y[n] - aver_y);
                    pos = n;
                }

                if(abs(filtered_queue_z[n] - aver_z) > ddd_z)
                {
                    ddd_z = abs(filtered_queue_z[n] - aver_z);
                    pos = n;
                }
            }
            
            cout << "ddd_y : " << ddd_y << " , ddd_z : " << ddd_z << endl; 

            if(ddd_y > ddd_y_range || ddd_z > ddd_z_range)
            {
                ros::spinOnce();
                calculate_circ_xyz();
                filtered_queue_y[pos] = circ_coordinates[1];
                filtered_queue_z[pos] = circ_coordinates[2];

                sum_y = 0;
                sum_z = 0;
                for(int j=0;j<10;j++)
                {
                    sum_y += filtered_queue_y[j];
                    sum_z += filtered_queue_z[j];
                }        
                aver_y = sum_y/10.0;
                aver_z = sum_z/10.0;

                refilter_time++;
                cout << "filter[" << refilter_time << "] :" << aver_y << " , " << aver_z << endl;

            }
            else
            {
                filter_finish_flag = 1;
                filtered_cir_y = aver_y;
                filtered_cir_z = aver_z;
            }
        }

        ros::spinOnce();
        cout << "final_filtered:" << filtered_cir_y << "   " << filtered_cir_z << endl;
        cout << "dy : " << filtered_cir_y - pos_drone.pose.position.y << endl;
        cout << "dz : " << filtered_cir_z - pos_drone.pose.position.z << endl;
    }
}



void filter_queue_update(int key)
{
    if(key > 0)
    {
        if(target_circ.y > 0 && target_circ.y < target_circ.rows && target_circ.x >0 && target_circ.x < target_circ.cols)
        {
            if(circ_coordinates[1] > filtered_cir_y - filter_dy_range && circ_coordinates[1] < filtered_cir_y + filter_dy_range
                    && circ_coordinates[2] > filtered_cir_z - filter_dz_range && circ_coordinates[2] < filtered_cir_z + filter_dz_range)
            {
                int a = rand()%10;
                //filtered_queue_y.erase(filtered_queue_y.begin() + rand()%10);
                filtered_queue_y[a] = circ_coordinates[1];
                filtered_queue_z[a] = circ_coordinates[2];
                //filtered_queue_y.push_back(circ_coordinates[1]);
            }
            else
            {
                cout << "the flitering data is wrong..." << endl;
                return;
            }

            float sum_y = 0;
            float sum_z = 0;
            for(int j=0;j<10;j++)
            {
                sum_y += filtered_queue_y[j];
                sum_z += filtered_queue_z[j];
            }        
            float aver_y = sum_y/10.0;
            float aver_z = sum_z/10.0;


            filtered_cir_y = aver_y;
            filtered_cir_z = aver_z;
        }
        else
        {
            ROS_WARN("filter data out of scene!!! filter not update!!!");
            return;
        }
    }
}
             
```

