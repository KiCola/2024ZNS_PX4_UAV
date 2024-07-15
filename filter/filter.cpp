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