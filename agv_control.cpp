#include "agv_control/agv_control.h"

AGVCONTROL::AGVCONTROL()
{
    circle_out_angle = circle_out_angle * Degree2Radian;
}

void AGVCONTROL::Manual_Control()
{   
    if(agv_cmd.charge_cmd == 1 || charge_action_step > 0)
    {
        manual_Action_1();
    }
    else if(charge_action_step == 0)
    {
        manual_vx = agv_cmd.manual_vx / 1000.0;
        manual_vy = agv_cmd.manual_vy / 1000.0;
        manual_vth = agv_cmd.manual_vth / 100.0 * Degree2Radian;
        manual_dir = agv_cmd.manual_dir;
        manual_updown = agv_cmd.manual_act;

        if(manual_vx > 1.6)
            manual_vx = 1.6;
        if(manual_vy > 1.6)
            manual_vy = 1.6;
        if(manual_vth > PI/2.0)
            manual_vth = PI/2.0;

        switch (manual_dir)
        {
        case 1:
            control_cmd.vx = manual_vx;
            control_cmd.vy = 0;
            control_cmd.vth = 0;
            break;
        case 2:
            control_cmd.vx = -manual_vx;
            control_cmd.vy = 0;
            control_cmd.vth = 0;
            break;
        case 3:
            control_cmd.vx = 0;
            control_cmd.vy = 0;
            control_cmd.vth = manual_vth;
            break;
        case 4:
            control_cmd.vx = 0;
            control_cmd.vy = 0;
            control_cmd.vth = -manual_vth;
            break;
        case 5:
            control_cmd.vx = manual_vx * cos(manual_vth);
            control_cmd.vy = 0;
            control_cmd.vth = manual_vth;
            break;
        case 6:
            control_cmd.vx = manual_vx * cos(manual_vth);
            control_cmd.vy = 0;
            control_cmd.vth = -manual_vth;
            break;
        case 7:
            control_cmd.vx = -manual_vx * cos(manual_vth);
            control_cmd.vy = 0;
            control_cmd.vth = -manual_vth;
            break;
        case 8:
            control_cmd.vx = -manual_vx * cos(manual_vth);
            control_cmd.vy = 0;
            control_cmd.vth = manual_vth;
            break;
        default:
            control_cmd.vx = 0;
            control_cmd.vy = 0;
            control_cmd.vth = 0;
            break;
        }
        last_manual_dir = manual_dir;

        switch (manual_updown)
        {
        case 1:
            control_cmd.lifter_dir = 1;
            break;
        case 2:
            control_cmd.lifter_dir = -1;
            break;
        default:
            control_cmd.lifter_dir = 0;
            break;
        }
        control_cmd.Yout[5-1] = 1;
        control_cmd.Yout[6-1] = 1;
        control_cmd.Yout[7-1] = 0;
    }
}

void AGVCONTROL::Stop_AGV()
{
    if(control_state.stripe)
    {
        ROS_WARN_STREAM_THROTTLE(1, GetSystemTime() << "Stripe!");
        control_cmd.lifter_dir = 0;
        control_cmd.vth = 0;
        if(control_cmd.vx > 0)
            control_cmd.vx = 0;
    }
    if(charge_action_step > 0)
    {
        ROS_INFO_STREAM_THROTTLE(30, GetSystemTime() << "Charging! Now:" << (int)agv_state.battery);
        control_cmd.vth = 0;
        control_cmd.vx = 0;
        control_cmd.vy = 0;
    }
    // if(control_state.front_dec && control_cmd.vx>0 && agv_state.manual_auto == 0)
    // {
    //     control_cmd.vx = control_cmd.vx * 0.4;
    //     control_cmd.vy = control_cmd.vy * 0.4;
    //     control_cmd.vth = control_cmd.vth * 0.4;
    //     if(control_cmd.vx < 0.1)
    //         control_cmd.vx = 0.1;
    // }
    // if(control_state.front_stop && control_cmd.vx>0 && agv_state.manual_auto == 0)
    // {
    //     control_cmd.vx = 0;
    //     control_cmd.vth = 0;
    //     control_cmd.vy = 0;
    // }
    // control_cmd.vx = 0;

    if(control_state.fork_max && control_cmd.lifter_dir == 1)
        control_cmd.lifter_dir = 0;
    if(control_state.fork_min && control_cmd.lifter_dir == -1)
        control_cmd.lifter_dir = 0;
}

void AGVCONTROL::Get_AUTO_CMD()
{
    if(agv_cmd.manual_auto == 1)
    {
        if(task_state == 0 && agv_cmd.task_start == 1) // 任务空闲，开始任务按下，就开始读路线
        {
            task_state = 1;
        }
        if(task_state == 2 && agv_cmd.task_resume == 1) // 路线读取完毕，恢复任务按下，则开始自动运行
        {
            task_state = 3;
        }
        if(task_state == 3 && agv_cmd.task_pause == 1) // 自动运行的过程中，暂停按钮按下，则不运行
        {
            task_state = 2;
            control_cmd.vx = 0;
            control_cmd.vy = 0;
            control_cmd.vth = 0;
            std::cout << GetSystemTime() << "pause!" << std::endl;
        }
        if((task_state == 1 || task_state==2) && agv_cmd.task_cancel == 1) // 如果在读路线、等待自动运行（暂停）中触发任务取消，则回到等待路线阶段
        {
            task_state = 0;
        }
    }
    else if(agv_cmd.manual_auto == 0)
    {
        if(task_state > 0 && agv_cmd.manual_auto == 0) // 如果任务执行阶段，按下手动按钮，则再启动自动跑的时候，需要按下恢复按钮
        {
            task_state = 2;
        }
    }
}

void AGVCONTROL::Get_Path()
{
    memset(&get_path_msg,0,sizeof(get_path_msg));
    memset(&get_action_msg,0,sizeof(get_action_msg));
    int get_point_num = 0;
    std::ifstream ifs;
	std::string PathFile = "/home/slamopto/path_file/path.txt";
    ifs.open(PathFile, std::ios::in);
    if(!ifs.is_open())
        ROS_WARN_STREAM(GetSystemTime() << "Path File Open Failed!");
    else
    {
        while(!ifs.eof())
        {
            std::string str;
            getline(ifs,str);
            std::string str_head = str.substr(0,str.find_first_of("="));
            if(str_head == "id")
            {
                sscanf((const char *)str.c_str(),"id=%d,x=%f,y=%f,dir=%hd,speed=%f,act=%hd,act_param=%hd",
                                                  &get_path_msg.path_point[get_point_num].id,
                                                  &get_path_msg.path_point[get_point_num].coor.x,
                                                  &get_path_msg.path_point[get_point_num].coor.y,
                                                  &get_path_msg.path_point[get_point_num].dir,
                                                  &get_path_msg.path_point[get_point_num].speed,
                                                  &get_path_msg.path_point[get_point_num].act,
                                                  &get_path_msg.path_point[get_point_num].act_param);
                get_point_num++;
            }
        }
        get_path_msg.num = get_point_num;
    }

	ROS_INFO_STREAM(GetSystemTime() << "Get Path Num=" << get_path_msg.num);
    for(int i=0; i<get_path_msg.num; i++)
    {
        ROS_INFO_STREAM(GetSystemTime() << "id=" << get_path_msg.path_point[i].id
                                        << ",x=" << get_path_msg.path_point[i].coor.x
                                        << ",y=" << get_path_msg.path_point[i].coor.y
                                        << ",dir=" << get_path_msg.path_point[i].dir
                                        << ",speed=" << get_path_msg.path_point[i].speed
                                        << ",wait=" << get_path_msg.path_point[i].wait
                                        << ",act=" << get_path_msg.path_point[i].act
                                        << ",param=" << get_path_msg.path_point[i].act_param);
    }
    int bezier_count = 0;

    // 解析成贝塞尔数据类型
    for(int i=1; i<get_path_msg.num-1; i++)
    {
        if((get_path_msg.path_point[i-1].dir==1 || get_path_msg.path_point[i-1].dir==2 || get_path_msg.path_point[i-1].dir==3 || get_path_msg.path_point[i-1].dir==4
         || get_path_msg.path_point[i-1].dir==5 || get_path_msg.path_point[i-1].dir==6)
         &&(get_path_msg.path_point[i].dir==7 || get_path_msg.path_point[i].dir==8 || get_path_msg.path_point[i].dir==9 || get_path_msg.path_point[i].dir==10))
         {
            int bezier_dir = get_path_msg.path_point[i].dir;
            if(get_path_msg.path_point[i+1].dir != bezier_dir)
            {
                ROS_WARN_STREAM(GetSystemTime() << "Get Wrong PathMsg! BezierMsg Wrong!");
                memset(&get_path_msg,0,sizeof(get_path_msg));
                task_state = 0;
                auto_state = 0;
                return;
            }
            else
            {
                get_bezier_msg.bezier[bezier_count].start_id = i;
                get_path_msg.path_point[i].bezier_id = bezier_count;
                int order_count = 0;
                for(int j=i; j<get_path_msg.num-1; j++)
                {
                    if(get_path_msg.path_point[j].dir == bezier_dir)
                    {
                        get_bezier_msg.bezier[bezier_count].bezier_curve[order_count].order_id = order_count;
                        get_bezier_msg.bezier[bezier_count].bezier_curve[order_count].start_id = j;
                        get_bezier_msg.bezier[bezier_count].bezier_curve[order_count].length = CalculateP2PDis(get_path_msg.path_point[j].coor,get_path_msg.path_point[j+1].coor);
                        get_bezier_msg.bezier[bezier_count].length += get_bezier_msg.bezier[bezier_count].bezier_curve[order_count].length;
                        get_path_msg.path_point[j].bezier_id = bezier_count;
                        get_path_msg.path_point[j].bezier_order = order_count;
                        order_count++;
                    }
                    else
                    {
                        if(order_count >= 4)
                        {
                            ROS_WARN_STREAM(GetSystemTime() << "Get Wrong PathMsg! BezierMsg Too much!");
                            memset(&get_path_msg,0,sizeof(get_path_msg));
                            memset(&get_bezier_msg,0,sizeof(get_bezier_msg));
                            task_state = 0;
                            auto_state = 0;
                            return;
                        }
                        else
                        {
                            get_bezier_msg.bezier[bezier_count].order_num = order_count+1;
                            get_bezier_msg.bezier[bezier_count].end_id = j;
                            for(int k=1; k<get_bezier_msg.bezier[bezier_count].order_num; k++)
                            {
                                float cal_ratiolength = 0;
                                for(int l=0; l<k; l++)
                                {
                                    cal_ratiolength += get_bezier_msg.bezier[bezier_count].bezier_curve[l].length;
                                }
                                get_bezier_msg.bezier[bezier_count].bezier_curve[k].order_ratio = cal_ratiolength / get_bezier_msg.bezier[bezier_count].length;
                            }
                            bezier_count++;
                            i = j;
                            break;
                        }
                    }
                }
            }
         }
    }
	ROS_INFO_STREAM(GetSystemTime() << "Total bezier num=" << bezier_count);
    for(int i=0; i<bezier_count; i++)
    {
        std::cout << "--------------------------------" << std::endl;
        std::cout << "bezier_id=" << i << ",si=" << get_bezier_msg.bezier[i].start_id << ",ei=" << get_bezier_msg.bezier[i].end_id
                  << ",on=" << get_bezier_msg.bezier[i].order_num << ",lenth=" << get_bezier_msg.bezier[i].length << std::endl;
        for(int j=0; j<get_bezier_msg.bezier[i].order_num; j++)
        {
            std::cout << "order_id=" << j << ",ratio=" << get_bezier_msg.bezier[i].bezier_curve[j].order_ratio
                      << ",si=" << get_bezier_msg.bezier[i].bezier_curve[j].start_id << ",lenth=" << get_bezier_msg.bezier[i].bezier_curve[j].length << std::endl;
        }
        if(get_bezier_msg.bezier[i].order_num == 3)
        {
            if(get_path_msg.path_point[get_bezier_msg.bezier[i].start_id].dir == 7)
            {
                get_path_msg.path_point[get_bezier_msg.bezier[i].start_id].dir = 11;
                get_path_msg.path_point[get_bezier_msg.bezier[i].start_id+1].dir = 11;
            }
            else if(get_path_msg.path_point[get_bezier_msg.bezier[i].start_id].dir == 8)
            {
                get_path_msg.path_point[get_bezier_msg.bezier[i].start_id].dir = 12;
                get_path_msg.path_point[get_bezier_msg.bezier[i].start_id+1].dir = 12;
            }
        }
    }
    std::cout << "--------------------------------" << std::endl;


    std::cout << "++++++++++++++++++++++++++++++++" << std::endl;
    for(int i=0; i<get_path_msg.num; i++)
    {
        std::cout << "id=" << get_path_msg.path_point[i].id
                  << ",dir=" << get_path_msg.path_point[i].dir
                  << ",x=" << get_path_msg.path_point[i].coor.x
                  << ",y=" << get_path_msg.path_point[i].coor.y
                  << ",sp=" << get_path_msg.path_point[i].speed
                  << ",bid=" << get_path_msg.path_point[i].bezier_id
                  << ",bor=" << get_path_msg.path_point[i].bezier_order << std::endl;
    }
    std::cout << "++++++++++++++++++++++++++++++++" << std::endl;

    auto_state = 1;
    task_state = 2;
    current_site = 0;
}

void AGVCONTROL::Automatic_Motion_Control()
{
    ROS_INFO_STREAM_THROTTLE(1,GetSystemTime() << "auto_state=" << auto_state << ",site=" << current_site << ",dir=" << get_path_msg.path_point[current_site].dir);
    // ROS_INFO_STREAM(GetSystemTime() << "auto_state=" << auto_state << ",site=" << current_site << ",dir=" << get_path_msg.path_point[current_site].dir);
    if(auto_state == 0)
    {
        agv_state.targetx = 0;
        agv_state.targety = 0;
    }
    if(task_state == 0)
    {
        site_start_coor.x = agv_state.slamx / 1000.0;
        site_start_coor.y = agv_state.slamy / 1000.0;
        memset(&get_path_msg,0,sizeof(get_path_msg));
        memset(&get_bezier_msg,0,sizeof(get_bezier_msg));
        current_site = 0;
        site_state = 0;
        auto_state = 0;
        action_step = 0;
        agv_state.move_dir =0;
    }
    if(task_state == 1)
    {
        Get_Path();
        agv_state.move_dir = 0;
    }
    if(task_state == 2 || task_state == 3)
    {
        agv_state.targetx = (int32_t)(get_path_msg.path_point[current_site+1].coor.x * 1000);
        agv_state.targety = (int32_t)(get_path_msg.path_point[current_site+1].coor.y * 1000);
        if(get_path_msg.path_point[current_site].dir == 1 || get_path_msg.path_point[current_site].dir == 7)
        {
            agv_state.move_dir = 1;
        }
        else if(get_path_msg.path_point[current_site].dir == 2 || get_path_msg.path_point[current_site].dir == 8)
        {
            agv_state.move_dir = 2;
        }
        else if(get_path_msg.path_point[current_site].dir == 5)
        {
            agv_state.move_dir = 5;
        }
        else if(get_path_msg.path_point[current_site].dir == 6)
        {
            agv_state.move_dir = 6;
        }
    }
    agv_state.state = 0;
    if(task_state == 2)
    {
        control_cmd.vx = 0;
        control_cmd.vy = 0;
        control_cmd.vth = 0;
    }
    if(task_state == 3 && auto_state == 1)
    {
        agv_state.state = 1;
        current_coor.x = agv_state.slamx / 1000.0;
        current_coor.y = agv_state.slamy / 1000.0;
        current_loc.point.x = agv_state.slamx / 1000.0;
        current_loc.point.y = agv_state.slamy / 1000.0;
        current_loc.yaw = agv_state.angle / 100.0 * Degree2Radian;
        if(fabs(current_loc.yaw + PI) < 0.0001)
            current_loc.yaw = -1.0 * current_loc.yaw;

        // std::cout << "csite=" << current_site << ",num-1=" << get_path_msg.num-1 << "spin_task=" << spin_task << std::endl;
        if(current_site >= 0 && current_site < get_path_msg.num-1 && spin_task == 0)
        {
            current_line_th = atan2(get_path_msg.path_point[current_site+1].coor.y-get_path_msg.path_point[current_site].coor.y,
                                            get_path_msg.path_point[current_site+1].coor.x-get_path_msg.path_point[current_site].coor.x);
            if(get_path_msg.path_point[current_site+1].coor.y == get_path_msg.path_point[current_site].coor.y)
            {
                if(get_path_msg.path_point[current_site+1].coor.x >= get_path_msg.path_point[current_site].coor.x)
                    current_line_th = 0;
                else
                    current_line_th = PI;
            }
            if(get_path_msg.path_point[current_site].dir == 2 || get_path_msg.path_point[current_site].dir == 8)
            {
                current_line_th += PI;
                if(current_line_th > PI)
                    current_line_th -= 2.0 * PI;
            }
            if(current_site >0)
            {
                last_line_th = atan2(get_path_msg.path_point[current_site].coor.y-get_path_msg.path_point[current_site-1].coor.y,
                                            get_path_msg.path_point[current_site].coor.x-get_path_msg.path_point[current_site-1].coor.x);
                if(get_path_msg.path_point[current_site].coor.y == get_path_msg.path_point[current_site-1].coor.y)
                {
                    if(get_path_msg.path_point[current_site].coor.x >= get_path_msg.path_point[current_site-1].coor.x)
                        last_line_th = 0;
                    else
                        last_line_th = PI;
                }
                std::cout << "111last_th=" << last_line_th << ",current_th=" << current_line_th << std::endl;
                if(get_path_msg.path_point[current_site-1].dir == 2 || get_path_msg.path_point[current_site-1].dir == 8)
                {
                    last_line_th += PI;
                    if(last_line_th > PI)
                        last_line_th -= 2.0 * PI;
                }
                float _th_dif = current_line_th - last_line_th;
                if(_th_dif > PI)
                    _th_dif = _th_dif - 2.0*PI;
                else if(_th_dif < -PI)
                    _th_dif = _th_dif + 2.0*PI;
                if(fabs(_th_dif) <= 0.1 || (fabs(fabs(current_line_th)-PI)<0.01 && fabs(fabs(last_line_th)-PI)<0.01))
                    spin_task = 1;
                else if(fabs(_th_dif) > 0.1)
                    spin_task = 2;
                if((get_path_msg.path_point[current_site-1].dir == 1 && get_path_msg.path_point[current_site].dir == 7)
                || (get_path_msg.path_point[current_site-1].dir == 2 && get_path_msg.path_point[current_site].dir == 8)
                || (get_path_msg.path_point[current_site-1].dir == 7 && get_path_msg.path_point[current_site].dir == 1)
                || (get_path_msg.path_point[current_site-1].dir == 8 && get_path_msg.path_point[current_site].dir == 2)
                || ((get_path_msg.path_point[current_site-1].dir == 1 || get_path_msg.path_point[current_site-1].dir == 2
                || get_path_msg.path_point[current_site-1].dir == 11 || get_path_msg.path_point[current_site-1].dir == 12)
                && (get_path_msg.path_point[current_site].dir == 1 || get_path_msg.path_point[current_site].dir == 2
                || get_path_msg.path_point[current_site].dir == 11 || get_path_msg.path_point[current_site].dir == 12)))
                {
                    spin_task = 1;
                }
            }
        }
        // std::cout << "last_th=" << last_line_th << ",current_th=" << current_line_th << ",locth=" << current_loc.yaw << ",stask=" << spin_task << ",sstate" << spin_state << std::endl;
        double angle_dif = current_loc.yaw - current_line_th;
        if(angle_dif > PI)
            angle_dif = angle_dif - 2*PI;
        else if(angle_dif < -PI)
            angle_dif = angle_dif + 2*PI;
        if(spin_task == 2 && spin_state == 0)
        {
            ROS_INFO_STREAM_THROTTLE(1,GetSystemTime() << "Spin!");
            control_cmd.vx = 0;
            float set_spin_vth = 0;
            if((current_line_th - current_loc.yaw > 0 && current_line_th - current_loc.yaw <= PI) || current_line_th - current_loc.yaw <= -PI)
            {
                set_spin_vth = auto_spin_vth;
            }
            else if((current_line_th - current_loc.yaw < 0 && current_line_th - current_loc.yaw >= -PI) || current_line_th - current_loc.yaw >= PI)
            {
                set_spin_vth = -auto_spin_vth;
            }
            if(fabs(angle_dif) > auto_spin_finish*2)
            {
                control_cmd.vth = set_spin_vth;
            }
            else if(fabs(angle_dif) > auto_spin_finish/3.0)
            {
                control_cmd.vth = fabs(angle_dif) / (auto_spin_finish*2) * set_spin_vth;
                if(fabs(control_cmd.vth) <= auto_spin_vth_min)
                {
                    control_cmd.vth = auto_spin_vth_min * control_cmd.vth / fabs(control_cmd.vth);
                }
            }
            else
            {
                // std::cout << "Finish Spin!clth=" << current_line_th << ",clocth=" << current_loc.yaw << ",minus=" << fabs(current_line_th - current_loc.yaw) << std::endl;
                control_cmd.vth = 0;
                spin_state = 1;
            }
            // std::cout << "clth=" << current_line_th << ",th=" << current_loc.yaw << ",vx=" << control_cmd.vx << ",vth=" << control_cmd.vth << std::endl;
        }
        // std::cout << "spin_task=" << spin_task << ",spin_state=" << spin_state << std::endl;
        if(current_site == 0)
        {
            spin_task = 1;
        }

        if(spin_task == 1 || (spin_task == 2 && spin_state == 1))
        {
            float eld = 0;
            float eth = 0;
            float cal_v = 0, cal_vth = 0;
            int closest_site = 0;
            PointMsg current_clos_point;
            // std::cout << "Now site=" << current_site << ".Now dir=" << get_path_msg.path_point[current_site].dir << std::endl;

            if(get_path_msg.path_point[current_site].dir == 7 || get_path_msg.path_point[current_site].dir == 8)
            {
                // ROS_INFO_STREAM(GetSystemTime() << "Now Curve");
                int b_order_num = get_bezier_msg.bezier[get_path_msg.path_point[current_site].bezier_id].order_num;
                if(bezier_task == 0)
                {
                    // ROS_INFO_STREAM(GetSystemTime() << "Curve Calculate");
                    if(b_order_num == 3)
                    {
                        for(int i=0; i<=100; i++)
                        {
                            float _t = i / 100.0;
                            current_bezier_points[i].x = pow(1-_t,2) * get_path_msg.path_point[current_site].coor.x
                                                       + 2 * _t * (1-_t) * get_path_msg.path_point[current_site+1].coor.x
                                                       + pow(_t,2) * get_path_msg.path_point[current_site+2].coor.x;
                            current_bezier_points[i].y = pow(1-_t,2) * get_path_msg.path_point[current_site].coor.y
                                                       + 2 * _t * (1-_t) * get_path_msg.path_point[current_site+1].coor.y
                                                       + pow(_t,2) * get_path_msg.path_point[current_site+2].coor.y;
                        }
                    }
                    else if(b_order_num == 4)
                    {
                        for(int i=0; i<=100; i++)
                        {
                            float _t = i / 100.0;
                            current_bezier_points[i].x = pow(1-_t,3) * get_path_msg.path_point[current_site].coor.x
                                                       + 3 * _t * pow(1-_t,2) * get_path_msg.path_point[current_site+1].coor.x
                                                       + 3 * pow(_t,2) * (1-_t) * get_path_msg.path_point[current_site+2].coor.x
                                                       + pow(_t,3) * get_path_msg.path_point[current_site+3].coor.x;
                            current_bezier_points[i].y = pow(1-_t,3) * get_path_msg.path_point[current_site].coor.y
                                                       + 3 * _t * pow(1-_t,2) * get_path_msg.path_point[current_site+1].coor.y
                                                       + 3 * pow(_t,2) * (1-_t) * get_path_msg.path_point[current_site+2].coor.y
                                                       + pow(_t,3) * get_path_msg.path_point[current_site+3].coor.y;
                        }
                    }
                    std::cout << "b_n=" << b_order_num << std::endl;
                    // for(int i=0; i<=100; i++)
                    // {
                    //     std::cout << i << "," << current_bezier_points[i].x << "," << current_bezier_points[i].y << std::endl;
                    // }
                    bezier_task = 1;
                }
                else if(bezier_task == 1)
                {
                    // ROS_INFO_STREAM(GetSystemTime() << "Curve Finish Calculate");
                    agv_state.targetx = (int32_t)(get_path_msg.path_point[current_site+b_order_num].coor.x * 1000);
                    agv_state.targety = (int32_t)(get_path_msg.path_point[current_site+b_order_num].coor.y * 1000);
                    int bezier_clost_num = 0;
                    float  bezier_clost_dis = 9999;
                    int whole_length = get_bezier_msg.bezier[get_path_msg.path_point[current_site].bezier_id].length;
                    for(int i=last_bezier_t; i<=100; i++)
                    {
                        float dis = sqrt(pow(current_loc.point.x - current_bezier_points[i].x,2)
                                        +pow(current_loc.point.y - current_bezier_points[i].y,2));
                        if(dis <= bezier_clost_dis)
                        {
                            bezier_clost_dis = dis;
                            bezier_clost_num = i;
                        }
                    }
                    current_clos_point.x = current_bezier_points[bezier_clost_num].x;
                    current_clos_point.y = current_bezier_points[bezier_clost_num].y;
                    last_bezier_t = bezier_clost_num;

                    if(CalculateP2PDis(current_clos_point,current_coor) < derail_dis)
                    {
                        if(get_path_msg.path_point[current_site].dir == 2 || get_path_msg.path_point[current_site].dir==8)
                        {
                            ld0_curve = ld0_curve_back;
                        }
                        else
                        {
                            ld0_curve = ld0_curve_front;
                        }
                        if(site_state == 0)
                        {
                            cal_v = get_path_msg.path_point[current_site].speed;
                            float ld = kld_curve*cal_v + ld0_curve;
                            if(ld > ldmax)
                                ld = ldmax;
                            int ld_num = (int)(ld/whole_length*100);
                            if(ld_num == 0)
                                ld_num = 1;
                            if(bezier_clost_num >= 99)
                            {
                                ROS_INFO_STREAM(GetSystemTime() << "Got Point!");
                                ROS_INFO_STREAM(GetSystemTime() << "X=" << current_loc.point.x << ",Y=" << current_loc.point.y << ",A=" << current_loc.yaw*Radian2Degree);
                                site_state = 1;
                            }
                            else
                            {
                                PointMsg goal_point;
                                // std::cout << "bezier_clost_num + ld_num=" << bezier_clost_num + ld_num << std::endl;
                                if(bezier_clost_num + ld_num >= 99)
                                {
                                    float reference_line_th = atan2(get_path_msg.path_point[current_site+b_order_num].coor.y-get_path_msg.path_point[current_site+b_order_num-1].coor.y,get_path_msg.path_point[current_site+b_order_num].coor.x-get_path_msg.path_point[current_site+b_order_num-1].coor.x);
                                    goal_point.x = get_path_msg.path_point[current_site+b_order_num].coor.x + ld0 * cos(reference_line_th);
                                    goal_point.y = get_path_msg.path_point[current_site+b_order_num].coor.y + ld0 * sin(reference_line_th);
                                    ld = kld*cal_v + ld0;
                                    // std::cout << "reference_line_th=" << reference_line_th << ",gx=" <<    goal_point.x << ",gy=" << goal_point.x << std::endl;
                                }
                                else
                                {
                                    goal_point.x = current_bezier_points[bezier_clost_num+ld_num].x;
                                    goal_point.y = current_bezier_points[bezier_clost_num+ld_num].y;
                                }
                                // std::cout << "bcn=" << bezier_clost_num << ",ldn=" << ld_num << ",gx=" << goal_point.x << ",gy=" << goal_point.y << std::endl;
                                if(get_path_msg.path_point[current_site].dir == 2 || get_path_msg.path_point[current_site].dir==8)
                                {
                                    eld = Error_ld(current_loc.yaw+PI, current_coor.x, current_coor.y, goal_point.x, goal_point.y);
                                }
                                else
                                {
                                    eld = Error_ld(current_loc.yaw, current_coor.x, current_coor.y, goal_point.x, goal_point.y);
                                }
                                cal_vth = (2*eld * cal_v) / pow(ld,2) ;
                                if(cal_vth > max_auto_vth_curve)
                                    cal_vth = max_auto_vth_curve;
                                else if(cal_vth < -max_auto_vth_curve)
                                    cal_vth = -max_auto_vth_curve;
                                if(bezier_clost_num + ld_num >= 99)
                                {
                                    if(cal_vth > max_auto_vth)
                                        cal_vth = max_auto_vth;
                                    else if(cal_vth < -max_auto_vth)
                                        cal_vth = -max_auto_vth;
                                }
                                if(get_path_msg.path_point[current_site].dir==8)
                                {
                                    cal_v = -1.0 * cal_v;
                                }
                                control_cmd.vx = cal_v * cos(cal_vth);
                                control_cmd.vth = cal_vth;
                                // std::cout << "site=" << current_site << "X=" << current_loc.point.x << ",Y=" << current_loc.point.y << ",A=" << current_loc.yaw 
                                //         << ",clx=" << current_clos_point.x << ",cly=" << current_clos_point.y << ",gx=" << goal_point.x << ",gy=" << goal_point.y
                                //         << ",eld=" << eld << ",v=" << cal_v << ",vth=" << cal_vth << std::endl;
                            }
                        }
                    }
                    else
                    {
                        ROS_ERROR_STREAM_THROTTLE(1,GetSystemTime() << "tuogui!ClostX=" << current_clos_point.x << ",ClostY=" << current_clos_point.y);
                        control_cmd.vx = 0;
                        control_cmd.vth = 0;
                        task_state = 2;
                    }
                    // std::cout << "site=" << current_site << ",cx=" << current_clos_point.x << ",cy=" << current_clos_point.y
                    //         << ",eld=" << eld << ",eth=" << eth << ",v=" << cal_v << ",vth=" << cal_vth << std::endl;
                    if(site_state == 1)
                    {
                        // std::cout << "1111csite=" << current_site << std::endl;
                        current_site += (b_order_num-1);
                        // std::cout << "2222csite=" << current_site << std::endl;
                        site_state = 0;
                        spin_state = 0;
                        spin_task = 0;
                        bezier_task = 0;
                        last_bezier_t = 0;
                        site_start_coor.x = agv_state.slamx / 1000.0;
                        site_start_coor.y = agv_state.slamy / 1000.0;
                        memset(&current_bezier_points,0,sizeof(current_bezier_points));
                        if(current_site+1 == get_path_msg.num)
                        {
                            task_state = 0;
                            site_state = 0;
                            spin_task = 0;
                            spin_state = 0;
                            bezier_task = 0;
                            ROS_INFO_STREAM(GetSystemTime() << "Finish Path!");
                            if(current_site+1 == get_path_msg.num)
                            {
                                control_cmd.vx = 0;
                                control_cmd.vth = 0;
                            }
                        }
                    }
                }
            }
            else if(get_path_msg.path_point[current_site].dir == 11 || get_path_msg.path_point[current_site].dir == 12) // 11为前向圆弧，12为后向圆弧
            {
                LocMsg _end_loc;
                _end_loc.point = get_path_msg.path_point[current_site+2].coor;
                _end_loc.yaw = atan2(get_path_msg.path_point[current_site+2].coor.y-get_path_msg.path_point[current_site+1].coor.y,
                                     get_path_msg.path_point[current_site+2].coor.x-get_path_msg.path_point[current_site+1].coor.x);
                StraightLine _out_line_ver, _in_line_ver;
                if(get_path_msg.path_point[current_site].dir == 12)
                {
                    current_loc.yaw = current_loc.yaw + PI;
                    if(current_loc.yaw >= PI)
                        current_loc.yaw -= 2.0 * PI;
                    else if(current_loc.yaw < -PI)
                        current_loc.yaw += 2.0 * PI;
                }
                if(_end_loc.yaw >= PI)
                    _end_loc.yaw -= 2.0 * PI;
                else if(_end_loc.yaw < -PI)
                    _end_loc.yaw += 2.0 * PI;
                _in_line_ver = Cal_Line_From_Point_Th(current_loc.point,current_loc.yaw + PI/2.0);
                _out_line_ver = Cal_Line_From_Point_Th(_end_loc.point,_end_loc.yaw + PI/2.0);
                // std::cout << GetSystemTime() << "in_line_ver:th=" << _in_line_ver.th * Radian2Degree << ",x=" << _in_line_ver.start_coor.x << ",y=" << _in_line_ver.start_coor.y
                //                              << ",A=" << _in_line_ver.A << ",B=" << _in_line_ver.B << ",C=" << _in_line_ver.C << std::endl;
                // std::cout << GetSystemTime() << "out_line_ver:th=" << _out_line_ver.th * Radian2Degree << ",x=" << _out_line_ver.start_coor.x << ",y=" << _out_line_ver.start_coor.y
                //                              << ",A=" << _out_line_ver.A << ",B=" << _out_line_ver.B << ",C=" << _out_line_ver.C << std::endl;
                PointMsg _center;
                _center = Cal_Point_Line_Line(_in_line_ver, _out_line_ver);
                float _radius = CalculateP2PDis(current_loc.point, _center);

                cal_v = get_path_msg.path_point[current_site].speed;
                if(get_path_msg.path_point[current_site].dir == 12)
                {
                    cal_v = -cal_v;
                }
                if(get_path_msg.path_point[current_site].dir == 11)
                {
                    float _bias = 0;
                    // if(_radius >= 3)
                    //     _bias = -0.4;
                    // else if(_radius >= 2 && _radius < 3)
                    //     _bias = -0.8;
                    cal_vth = cal_v / (_radius + _bias);
                }
                else if(get_path_msg.path_point[current_site].dir == 12)
                {
                    float _bias = 0;
                    // if(_radius >= 2  && _radius < 3)
                    //     _bias = -0.02;
                    cal_vth = cal_v / (_radius + _bias);
                }
                float _angle_dif = _end_loc.yaw - current_loc.yaw;
                if(_angle_dif >= PI)
                    _angle_dif = _angle_dif - 2.0 * PI;
                else if(_angle_dif < -PI)
                    _angle_dif = _angle_dif + 2.0 * PI;
                if(_angle_dif <= 0 && get_path_msg.path_point[current_site].dir == 11)
                    cal_vth = -cal_vth;
                else if(_angle_dif > 0 && get_path_msg.path_point[current_site].dir == 12)
                    cal_vth = -cal_vth;
                control_cmd.vx = cal_v;
                control_cmd.vy = 0;
                control_cmd.vth = cal_vth;
                // std::cout << GetSystemTime() << "endth=" << _end_loc.yaw*Radian2Degree << ",nowth=" << current_loc.yaw*Radian2Degree << ",ad=" << _angle_dif*Radian2Degree << std::endl;
                // std::cout << GetSystemTime() << "cx=" << _center.x << ",cy=" << _center.y << ",r=" << _radius << ",vx=" << control_cmd.vx << ",vth=" << control_cmd.vth << std::endl;

                if(fabs(_angle_dif) <= circle_out_angle)
                {
                    current_site += 2;
                    ROS_INFO_STREAM(GetSystemTime() << "Got Point! Site=" << current_site << ",X=" << current_loc.point.x << ",Y=" << current_loc.point.y << ",th=" << current_loc.yaw*Radian2Degree);
                    site_state = 0;
                    spin_state = 0;
                    spin_task = 0;
                    site_start_coor.x = agv_state.slamx / 1000.0;
                    site_start_coor.y = agv_state.slamy / 1000.0;

                    if(current_site == get_path_msg.num-1)
                    {
                        ROS_INFO_STREAM(GetSystemTime() << "Finish Path!" << "Site_num=" << get_path_msg.num << ".ACT_NUM=" << get_path_msg.path_point[current_site].act);
                        if(get_path_msg.path_point[current_site].act != 0)
                        {
                            task_state = 3;
                            auto_state = 2;
                            ROS_WARN("ACT!");
                            if(get_path_msg.path_point[current_site].act == 1 || get_path_msg.path_point[current_site].act == 3)
                            {
                                control_cmd.vx = 0;
                                control_cmd.vth = 0;
                                control_cmd.vy = 0;
                            }
                        }
                        else
                        {
                            task_state = 0;
                            site_state = 1;
                            spin_task = 0;
                            spin_state = 0;
                            agv_state.targetx = 0;
                            agv_state.targety = 0;
                            control_cmd.vx = 0;
                            control_cmd.vth = 0;
                        }
                    }
                }
            }
            else
            {
                // ROS_INFO_STREAM(GetSystemTime() << "Now Straight");
                current_clos_point = GetClosestPoint(current_coor,get_path_msg.path_point[current_site].coor,get_path_msg.path_point[current_site+1].coor);
                // if(CalculateP2PDis(current_clos_point,current_coor) < derail_dis)
                if(Cal_Point_to_DoublePoints(current_coor,get_path_msg.path_point[current_site].coor,get_path_msg.path_point[current_site+1].coor) < derail_dis)
                {
                    // std::cout << "num=" << get_path_msg.num << ",current=" << current_site << std::endl;
                    if(current_coor,get_path_msg.path_point[current_site].dir == 2)
                        parking_dis = parking_dis_back;
                    else if(current_coor,get_path_msg.path_point[current_site].dir == 1)
                        parking_dis = parking_dis_front;
                    float totaldis = CalculateP2PDis(site_start_coor,get_path_msg.path_point[current_site+1].coor);
                    float disfromstart = CalculateP2PDis(site_start_coor,current_coor);
                    // if(CalculateP2PDis(current_clos_point,get_path_msg.path_point[current_site+1].coor) < parking_dis)
                    if(disfromstart + parking_dis >= totaldis)
                    {
                        ROS_INFO_STREAM(GetSystemTime() << "Got Point! Site=" << current_site);
                        if(site_state == 0)
                            site_finish = ros::Time::now();
                        site_state = 1;
                        if(site_state == 1)
                            ROS_INFO_STREAM(GetSystemTime() << "X=" << current_loc.point.x << ",Y=" << current_loc.point.y << ",A=" << current_loc.yaw*Radian2Degree);
                    }
                    if(site_state == 0)
                    {
                        float acc_dis = get_path_msg.path_point[current_site].speed/2.0;
                        float dec_dis = get_path_msg.path_point[current_site].speed*2.0;
                        if(current_site == get_path_msg.num-1)
                        {
                            if(dec_dis <= 0.2)
                                dec_dis = 0.2;
                        }
                        if(dec_dis > 1.8)
                            dec_dis = 1.8;
                        cal_v = CalculateControlSpeed(totaldis-parking_dis, disfromstart, get_path_msg.path_point[current_site].speed, acc_dis, dec_dis);
                        if(current_site == 0 && ((get_path_msg.path_point[current_site].dir == get_path_msg.path_point[current_site+1].dir)
                        || (get_path_msg.path_point[current_site+1].dir == 7 && get_path_msg.path_point[current_site].dir == 1)
                        || (get_path_msg.path_point[current_site+1].dir == 8 && get_path_msg.path_point[current_site].dir == 2)
                        || (get_path_msg.path_point[current_site+1].dir == 11 && get_path_msg.path_point[current_site].dir == 1)
                        || (get_path_msg.path_point[current_site+1].dir == 12 && get_path_msg.path_point[current_site].dir == 2)))
                        {
                            if(totaldis-disfromstart <= dec_dis)
                                cal_v = get_path_msg.path_point[current_site+1].speed;
                        }
                        if(current_site > 0 || current_site < get_path_msg.num-1)
                        {
                            if((get_path_msg.path_point[current_site-1].dir == 7 && get_path_msg.path_point[current_site].dir == 1)
                            || (get_path_msg.path_point[current_site-1].dir == 8 && get_path_msg.path_point[current_site].dir == 2)
                            || (get_path_msg.path_point[current_site-1].dir == 11 && get_path_msg.path_point[current_site].dir == 2)
                            || (get_path_msg.path_point[current_site-1].dir == 12 && get_path_msg.path_point[current_site].dir == 2)
                            || get_path_msg.path_point[current_site-1].dir == get_path_msg.path_point[current_site].dir)
                            {
                                if(disfromstart <= acc_dis)
                                    cal_v = get_path_msg.path_point[current_site].speed;
                            }
                            if((get_path_msg.path_point[current_site+1].dir == 7 && get_path_msg.path_point[current_site].dir == 1)
                            || (get_path_msg.path_point[current_site+1].dir == 8 && get_path_msg.path_point[current_site].dir == 2)
                            || (get_path_msg.path_point[current_site+1].dir == 11 && get_path_msg.path_point[current_site].dir == 1)
                            || (get_path_msg.path_point[current_site+1].dir == 12 && get_path_msg.path_point[current_site].dir == 2)
                            || get_path_msg.path_point[current_site+1].dir == get_path_msg.path_point[current_site].dir)
                            {
                                if(totaldis-disfromstart <= dec_dis)
                                    cal_v = get_path_msg.path_point[current_site+1].speed;
                            }
                        }
                        if(current_site == get_path_msg.num-2 && get_path_msg.path_point[get_path_msg.num-1].act == 4)
                        {
                           if(totaldis-disfromstart <= dec_dis)
                                cal_v = 0.4; 
                            // std::cout << "cal_v=" << cal_v << std::endl;
                        }
                        float ld = kld*cal_v + ld0;
                        if(ld > ldmax)
                            ld = ldmax;

                        PointMsg goal_point;
                        goal_point.x = current_clos_point.x + (get_path_msg.path_point[current_site+1].coor.x-get_path_msg.path_point[current_site].coor.x) * ld / totaldis;
                        goal_point.y = current_clos_point.y + (get_path_msg.path_point[current_site+1].coor.y-get_path_msg.path_point[current_site].coor.y) * ld / totaldis;
                        // std::cout << "cx=" << current_loc.point.x << ",cy=" << current_loc.point.y << ",cth=" << current_loc.yaw << std::endl;
                        if(get_path_msg.path_point[current_site].dir == 2 || get_path_msg.path_point[current_site].dir==8)
                        {
                            eld = Error_ld(current_loc.yaw+PI, current_coor.x, current_coor.y, goal_point.x, goal_point.y);
                        }
                        else
                        {
                            eld = Error_ld(current_loc.yaw, current_coor.x, current_coor.y, goal_point.x, goal_point.y);
                        }
                        // std::cout << "locth=" << current_loc.yaw << ",lineth=" << current_line_th << ",eth=" << eth << ",ld=" << ld << ",eld=" << eld << std::endl;
                        cal_vth = (2*eld * cal_v) / pow(ld,2);
                        if(cal_vth > max_auto_vth)
                            cal_vth = max_auto_vth;
                        else if(cal_vth < -max_auto_vth)
                            cal_vth = -max_auto_vth;
                        if(disfromstart < 0.3 && current_site == 0)
                        {
                            cal_vth = cal_vth / 2.0;
                        }
                        if(totaldis-disfromstart <= dec_dis && current_site == get_path_msg.num-1)
                        {
                            if(cal_vth > 0.01)
                                cal_vth = 0.01;
                            else if(cal_vth < -0.01)
                                cal_vth = -0.01;
                        }
                        // if(CalculateP2PDis(current_clos_point,get_path_msg.path_point[current_site+1].coor) < 2*parking_dis)
                        // {
                        //     if(angle_dif < -0.5 * Degree2Radian)
                        //         cal_vth = max_auto_vth;
                        //     else if(angle_dif > 0.5 * Degree2Radian)
                        //         cal_vth = -max_auto_vth;
                        // }
                        if(get_path_msg.path_point[current_site].dir == 2 || get_path_msg.path_point[current_site].dir==8)
                        {
                            cal_v = -1.0 * cal_v;
                            // cal_vth = -1.0 * cal_vth;
                        }
                        control_cmd.vx = cal_v;
                        control_cmd.vth = cal_vth;
                        // if(control_cmd.vx <= parking_speed)
                        //     control_cmd.vth += eth;
                        // control_cmd.vx = 0;
                        // control_cmd.vth = 0;
                        // ROS_INFO_STREAM(GetSystemTime() << "X=" << current_coor.x << ",Y=" << current_coor.y << ",A=" << agv_state.slamth
                        //     << ",CX=" << current_clos_point.x << ",CY=" << current_clos_point.y 
                        //     << ",gx=" << goal_point.x << ",gy=" << goal_point.y << ",e=" << eld << ",cth=" << control_cmd.vth);
                    }
                }
                else
                {
                    ROS_ERROR_STREAM_THROTTLE(1,GetSystemTime() << "tuogui!SX=" << get_path_msg.path_point[current_site].coor.x
                                                                << ",SY=" << get_path_msg.path_point[current_site].coor.y
                                                                << ",EX=" << get_path_msg.path_point[current_site+1].coor.x
                                                                << ",EY=" << get_path_msg.path_point[current_site+1].coor.y);
                    control_cmd.vx = 0;
                    control_cmd.vth = 0;
                    task_state = 2;

                }
                // ROS_INFO_STREAM("Site=" << current_site << ",X=" << current_coor.x << ",Y=" << current_coor.y << ",A=" << current_loc.yaw
                //     << ",CX=" << current_clos_point.x << ",CY=" << current_clos_point.y 
                //     << ",cv=" << cal_v << ",e=" << eld << ",cth=" << cal_vth << ",state=" << site_state);
                // std::cout << "site=" << current_site << ",cx=" << current_clos_point.x << ",cy=" << current_clos_point.y
                //         << ",eld=" << eld << ",eth=" << angle_dif << ",v=" << cal_v << ",vth=" << cal_vth << std::endl;
                if(site_state == 1)
                {
                    if(get_path_msg.path_point[current_site+1].wait > 0)
                    {
                        control_cmd.vx = 0;
                        control_cmd.vth = 0;
                        ros::Time site_start=ros::Time::now();
                        double tim_dif = (site_start - site_finish).toSec();
                        // std::cout << "time_dif=" << tim_dif << "f=" << site_finish << "s=" << site_start << std::endl;
                        if(tim_dif > 3)
                        {
                            current_site++;
                            site_state = 0;
                            spin_state = 0;
                            spin_task = 0;
                            site_start_coor.x = agv_state.slamx / 1000.0;
                            site_start_coor.y = agv_state.slamy / 1000.0;
                        }
                    }
                    else
                    {
                        current_site++;
                        site_state = 0;
                        spin_state = 0;
                        spin_task = 0;
                        site_start_coor.x = agv_state.slamx / 1000.0;
                        site_start_coor.y = agv_state.slamy / 1000.0;
                    }
                    if(current_site+1 == get_path_msg.num)
                    {
                        ROS_INFO_STREAM(GetSystemTime() << "Finish Path! ACT_NUM=" << get_path_msg.path_point[current_site].act);
                        if(get_path_msg.path_point[current_site].act != 0)
                        {
                            task_state = 3;
                            auto_state = 2;
                            ROS_WARN("ACT!");
                            if(get_path_msg.path_point[current_site].act == 1 || get_path_msg.path_point[current_site].act == 3)
                            {
                                control_cmd.vx = 0;
                                control_cmd.vth = 0;
                                control_cmd.vy = 0;
                            }
                        }
                        else
                        {
                            task_state = 0;
                            site_state = 1;
                            spin_task = 0;
                            spin_state = 0;
                            agv_state.targetx = 0;
                            agv_state.targety = 0;
                            control_cmd.vx = 0;
                            control_cmd.vth = 0;
                        }
                    }
                }
            }
        }
    }
}

float AGVCONTROL::CalculateP2PDis(PointMsg p1, PointMsg p2)
{
    float result_dis;
    result_dis = sqrt(pow(p1.x-p2.x,2) + pow(p1.y-p2.y,2));
    return result_dis;
}

PointMsg AGVCONTROL::GetClosestPoint(PointMsg now_coor, PointMsg start_coor, PointMsg end_coor)
{
    // std::cout << "cx=" << now_coor.x
    //           << ",cy=" << now_coor.y
    //           << "c1x=" << start_coor.x
    //           << ",c1y=" <<  start_coor.y
    //           << ",c2x=" << end_coor.x
    //           << ",c2y=" << end_coor.y << std::endl;

    PointMsg result_point;
    // 计算线段向量
    float segmentX = end_coor.x - start_coor.x;
    float segmentY = end_coor.y - start_coor.y;

    // 线段长度的平方
    float segmentLengthSquared = segmentX * segmentX + segmentY * segmentY;

    // 如果线段长度为0，则直接返回起点作为最近的点
    if (segmentLengthSquared == 0) 
    {
        result_point = start_coor;
    }

    // 计算当前点到起点的向量
    float vectorX = now_coor.x - start_coor.x;
    float vectorY = now_coor.y - start_coor.y;

    // 计算点到线段的投影长度
    float projectionLength = (vectorX * segmentX + vectorY * segmentY) / segmentLengthSquared;
    if (projectionLength <= 0) // 如果投影长度小于0，表明最近的点是M
    {
        result_point = start_coor;
    }
    else if (projectionLength >= 1) // 如果投影长度大于1，表明最近的点是N
    {
        result_point = end_coor;
    }
    else
    {
        result_point.x = start_coor.x + projectionLength * segmentX;
        result_point.y = start_coor.y + projectionLength * segmentY;
    }
    // std::cout << "PL=" << projectionLength << ",Cx=" << result_point.x << ",Cy=" << result_point.y << std::endl;
    return result_point;
}

float AGVCONTROL::Cal_Point_to_DoublePoints(PointMsg now_coor, PointMsg start_coor, PointMsg end_coor)
{
    float result;
    double A=0, B=0, C=0;
    A = end_coor.y - start_coor.y;
    B = -(end_coor.x - start_coor.x);
    C= start_coor.y * (end_coor.x - start_coor.x) - start_coor.x * (end_coor.y - start_coor.y);
    result = fabs((A*now_coor.x + B*now_coor.y + C) / sqrt(A*A + B*B));
}

float AGVCONTROL::Error_ld(float theta, float m, float n, float x, float y) // 假定点在向量的逆时针向，则Eld为负；假定点在向量的顺时针向，则Eld为正
{
    double distance = 0;
    double A = 0, B = 0, C = 0;

    if(theta < -PI)
        theta += 2.0 * PI;
    else if(theta > PI)
        theta -= 2.0 * PI;

    if(fabs(theta - PI/2.0) < 0.000001 || fabs(theta + PI/2.0) < 0.000001)
    {
        A = 1;
        B = 0;
        C = -m;
    }
    else
    {
        // 计算直线的斜率
        double k = tan(theta);
        double b = n - m * k;
        A = k;
        B = -1;
        C = b;
    }
    distance = (A*x + B*y + C) / sqrt(A*A + B*B);

    // std::cout << "11111,th=" << theta << ",dis=" << distance << ",gx=" << x << ",gy=" << y << ",m=" << m << ",n=" << n << std::endl;
    if(theta > -PI/2.0 && theta <= PI/2.0)
        distance = -1.0 * distance;

    // std::cout << "22222,th=" << theta << ",dis=" << distance << ",gx=" << x << ",gy=" << y << ",m=" << m << ",n=" << n << std::endl;

    return distance;
}

// 计算S曲线加速减速的控制速度
float AGVCONTROL::CalculateControlSpeed(float totalDistance, float distanceFromStart, float targetVelocity, float accelerationDistance, float decelerationDistance)
{
    float controlSpeed;
    float sigmoidArg;

    // 判断是否需要进行加速或减速
    if (distanceFromStart <= accelerationDistance) // 加速阶段
    {
        float x_ratio = distanceFromStart / accelerationDistance;
        float e_param = exp(-6.0 * (x_ratio - 0.5));
        controlSpeed = targetVelocity / 2.0 * ((1-e_param) / (1+e_param) + 1);
        if(controlSpeed < parking_speed*2)
            controlSpeed = parking_speed*2;
    } else if (distanceFromStart <= totalDistance - decelerationDistance) // 匀速阶段
    {
        controlSpeed = targetVelocity;
    } else // 减速阶段
    {
        float remainingDistance = totalDistance - distanceFromStart;
        float x_ratio = remainingDistance / decelerationDistance;
        float e_param = exp(-6.0 * (x_ratio - 0.5));
        controlSpeed = targetVelocity / 2.0 * ((1-e_param) / (1+e_param) + 1);
        if(controlSpeed < parking_speed)
            controlSpeed = parking_speed;
    }

    return controlSpeed;
}

StraightLine AGVCONTROL::Cal_Line_From_Point_Th(PointMsg _p, float _th)
{
    StraightLine _result;
    double A, B, C;
    if(_th >= PI)
        _th = _th - 2.0 * PI;
    else if(_th < -PI)
        _th = _th + 2.0 * PI;
    _result.th = _th;
    _result.start_coor = _p;
    if(fabs(_th - PI/2.0) < 0.0001 || fabs(_th + PI/2.0) < 0.0001)
    {
        A = 1;
        B = 0;
        C = -_p.x;
    }
    else
    {
        A = tan(_th);
        B = -1;
        C = _p.y - _p.x * A;
    }
    _result.A = A;
    _result.B = B;
    _result.C = C;
    return _result;
}

PointMsg AGVCONTROL::Cal_Point_Line_Line(StraightLine l1, StraightLine l2)
{
    PointMsg _result;
    double D = l1.A * l2.B - l2.A * l1.B;
    if(fabs(D) < 0.001)
    {
        _result.x = 0;
        _result.y = 0;
    }
    else
    {
        _result.x = (l1.B*l2.C - l2.B*l1.C) / D;
        _result.y = (l1.C*l2.A - l2.C*l1.A) / D;
    }
    return _result;
}


void AGVCONTROL::Automatic_Action_Control()
{
    // std::cout << "task_state=" << task_state << ",auto_state=" << auto_state << std::endl;
    if(auto_state == 0)
    {
        control_cmd.charge_cmd = 0;
    }
    if(task_state == 0 || task_state == 1 || task_state == 2)
    {
        control_cmd.Yout[5-1] = 0;
        control_cmd.Yout[6-1] = 1;
        control_cmd.Yout[7-1] = 0;
        control_cmd.charge_cmd = 0;
    }
    if(task_state == 3 && auto_state == 1)
    {
        if(current_site+2 == get_path_msg.num 
       && (get_path_msg.path_point[get_path_msg.num-1].act == 1 
       || get_path_msg.path_point[get_path_msg.num-1].act == 3  
       || get_path_msg.path_point[get_path_msg.num-1].act == 5))
       {
            control_cmd.lifter_dir = -1;
       }
    }
    if(task_state == 3 && auto_state == 2)
    {
        ROS_INFO_STREAM_THROTTLE(2,GetSystemTime() << "auto_state=" << auto_state << ",act=" << get_path_msg.path_point[get_path_msg.num-1].act << ",param=" << get_path_msg.path_point[get_path_msg.num-1].act_param);
        control_cmd.Yout[5-1] = 0;
        control_cmd.Yout[6-1] = 0;
        control_cmd.Yout[7-1] = 1;
        switch (get_path_msg.path_point[get_path_msg.num-1].act)
        {
        case 1:
            Action_1();
            break;
        case 2:
            Action_2();
            break;
        case 3:
            Action_3();
            break;
        case 4:
            Action_4();
            break;
        case 5:
            Action_5();
            break;
        default:
            break;
        }
    }
}

void AGVCONTROL::Action_1()
{
    agv_state.state = 2;
    PointMsg current_action_point;
    current_action_point.x = control_state.slamx;
    current_action_point.y = control_state.slamy;
    current_loc.point.x = agv_state.slamx / 1000.0;
    current_loc.point.y = agv_state.slamy / 1000.0;
    current_loc.yaw = agv_state.angle / 100.0 * Degree2Radian;
    if(fabs(current_loc.yaw + PI) < 0.0001)
        current_loc.yaw = -1.0 * current_loc.yaw;

    if(action_step == 0)
    {
        control_cmd.vx = 0;
        control_cmd.vy = 0;
        control_cmd.vth = 0;
        control_cmd.lifter_dir = -1;
        if(control_state.fork_min)
        {
            control_cmd.lifter_dir = 0;
            action_step = 1;
        }
    }
    else if(action_step == 1)
    {
        control_cmd.vth = 0;
        control_cmd.vy = 0;
        if(sqrt(pow(current_loc.point.x-get_path_msg.path_point[get_path_msg.num-1].coor.x,2)
              + pow(current_loc.point.y-get_path_msg.path_point[get_path_msg.num-1].coor.y,2)) < 1.15)
        {
            control_cmd.vx = -0.2;
        }
        else if(sqrt(pow(current_loc.point.x-get_path_msg.path_point[get_path_msg.num-1].coor.x,2)
              + pow(current_loc.point.y-get_path_msg.path_point[get_path_msg.num-1].coor.y,2)) >= 1.15)
        {
            control_cmd.vx = -0.06;
        }
        if(control_state.left_tuopan || control_state.right_tuopan)
        {
            control_cmd.vx = 0;
            ROS_INFO_STREAM(GetSystemTime() << "Got Action Point!");
            ROS_INFO_STREAM(GetSystemTime() << "X=" << current_loc.point.x << ",Y=" << current_loc.point.y << ",A=" << current_loc.yaw*Radian2Degree);
            action_step = 2;
        }
    }
    else if(action_step == 2)
    {
        control_cmd.vx = 0;
        control_cmd.vy = 0;
        control_cmd.vth = 0;
        control_cmd.lifter_dir = 1;
        if(control_state.fork_max)
        {
            control_cmd.lifter_dir = 0;
            action_step = 3;
        }
    }
    else if(action_step == 3)
    {
        control_cmd.vth = 0;
        control_cmd.vy = 0;
        control_cmd.vx = 0.2;
        if(sqrt(pow(current_loc.point.x-get_path_msg.path_point[get_path_msg.num-1].coor.x,2)
              + pow(current_loc.point.y-get_path_msg.path_point[get_path_msg.num-1].coor.y,2)) < 0.05)
        {
            ROS_INFO_STREAM(GetSystemTime() << "Action Finish!");
            ROS_INFO_STREAM(GetSystemTime() << "X=" << current_loc.point.x << ",Y=" << current_loc.point.y << ",A=" << current_loc.yaw*Radian2Degree);
            control_cmd.vx = 0;
            control_cmd.vy = 0;
            control_cmd.vth = 0;
            action_step = 0;
            auto_state = 0;
            task_state = 0;
            agv_state.state = 0;
            agv_state.move_dir = 0;
            agv_state.targetx = 0;
            agv_state.targety = 0;
        }     
    }
}

void AGVCONTROL::Action_2()
{
    agv_state.state = 2;
    PointMsg current_action_point;
    current_action_point.x = control_state.slamx;
    current_action_point.y = control_state.slamy;
    current_loc.point.x = agv_state.slamx / 1000.0;
    current_loc.point.y = agv_state.slamy / 1000.0;
    current_loc.yaw = agv_state.angle / 100.0 * Degree2Radian;
    if(fabs(current_loc.yaw + PI) < 0.0001)
        current_loc.yaw = -1.0 * current_loc.yaw;

    if(action_step == 0)
    {
        control_cmd.vth = 0;
        control_cmd.vy = 0;
        if(sqrt(pow(current_loc.point.x-get_path_msg.path_point[get_path_msg.num-1].coor.x,2)
              + pow(current_loc.point.y-get_path_msg.path_point[get_path_msg.num-1].coor.y,2)) < 1.15)
        {
            control_cmd.vx = -0.2;
        }
        else if(sqrt(pow(current_loc.point.x-get_path_msg.path_point[get_path_msg.num-1].coor.x,2)
              + pow(current_loc.point.y-get_path_msg.path_point[get_path_msg.num-1].coor.y,2)) >= 1.15)
        {
            control_cmd.vx = -0.06;
        }
        if(fabs(sqrt(pow(current_loc.point.x-get_path_msg.path_point[get_path_msg.num-1].coor.x,2)
              + pow(current_loc.point.y-get_path_msg.path_point[get_path_msg.num-1].coor.y,2)) - 1.35) <= 0.005)
        {
            control_cmd.vx = 0;
            ROS_INFO_STREAM(GetSystemTime() << "Got Action Point!");
            ROS_INFO_STREAM(GetSystemTime() << "X=" << current_loc.point.x << ",Y=" << current_loc.point.y << ",A=" << current_loc.yaw*Radian2Degree);
            action_step = 1;
        }
    }
    else if(action_step == 1)
    {
        control_cmd.vx = 0;
        control_cmd.vy = 0;
        control_cmd.vth = 0;
        control_cmd.lifter_dir = -1;
        if(control_state.fork_min)
        {
            control_cmd.lifter_dir = 0;
            action_step = 2;
        }
    }
    else if(action_step == 2)
    {
        control_cmd.vth = 0;
        control_cmd.vy = 0;
        control_cmd.vx = 0.2;
        if(sqrt(pow(current_loc.point.x-get_path_msg.path_point[get_path_msg.num-1].coor.x,2)
              + pow(current_loc.point.y-get_path_msg.path_point[get_path_msg.num-1].coor.y,2)) < 0.02)
        {
            ROS_INFO_STREAM(GetSystemTime() << "Got Action Point!");
            ROS_INFO_STREAM(GetSystemTime() << "X=" << current_loc.point.x << ",Y=" << current_loc.point.y << ",A=" << current_loc.yaw*Radian2Degree);
            control_cmd.vx = 0;
            control_cmd.vy = 0;
            control_cmd.vth = 0;
            action_step = 3;
        }
    }
    else if(action_step == 3)
    {
        control_cmd.vx = 0;
        control_cmd.vy = 0;
        control_cmd.vth = 0;
        control_cmd.lifter_dir = 1;
        if(control_state.fork_max)
        {
            control_cmd.lifter_dir = 0;
            ROS_INFO_STREAM(GetSystemTime() << "Action Finish!");
            ROS_INFO_STREAM(GetSystemTime() << "X=" << current_loc.point.x << ",Y=" << current_loc.point.y << ",A=" << current_loc.yaw*Radian2Degree);
            control_cmd.vx = 0;
            control_cmd.vy = 0;
            control_cmd.vth = 0;
            action_step = 0;
            auto_state = 0;
            task_state = 0;
            agv_state.state = 0;
            agv_state.move_dir = 0;
            agv_state.targetx = 0;
            agv_state.targety = 0;
        }   
    }
}

void AGVCONTROL::Action_3()
{
    agv_state.state = 2;
    PointMsg current_action_point;
    current_action_point.x = control_state.slamx;
    current_action_point.y = control_state.slamy;
    current_loc.point.x = agv_state.slamx / 1000.0;
    current_loc.point.y = agv_state.slamy / 1000.0;
    current_loc.yaw = agv_state.angle / 100.0 * Degree2Radian;
    if(fabs(current_loc.yaw + PI) < 0.0001)
        current_loc.yaw = -1.0 * current_loc.yaw;

    if(action_step == 0)
    {
        control_cmd.vx = 0;
        control_cmd.vy = 0;
        control_cmd.vth = 0;
        control_cmd.lifter_dir = -1;
        if(control_state.fork_min)
        {
            control_cmd.lifter_dir = 0;
            action_step = 1;
        }
    }
    else if(action_step == 1)
    {
        control_cmd.vth = 0;
        control_cmd.vy = 0;
        if(sqrt(pow(current_loc.point.x-get_path_msg.path_point[get_path_msg.num-1].coor.x,2)
              + pow(current_loc.point.y-get_path_msg.path_point[get_path_msg.num-1].coor.y,2)) < 1.15)
        {
            control_cmd.vx = -0.2;
        }
        else if(sqrt(pow(current_loc.point.x-get_path_msg.path_point[get_path_msg.num-1].coor.x,2)
              + pow(current_loc.point.y-get_path_msg.path_point[get_path_msg.num-1].coor.y,2)) >= 1.15)
        {
            control_cmd.vx = -0.06;
        }
        if(control_state.left_tuopan || control_state.right_tuopan)
        {
            control_cmd.vx = 0;
            ROS_INFO_STREAM(GetSystemTime() << "Got Action Point!");
            ROS_INFO_STREAM(GetSystemTime() << "X=" << current_loc.point.x << ",Y=" << current_loc.point.y << ",A=" << current_loc.yaw*Radian2Degree);
            action_step = 2;
        }
    }
    else if(action_step == 2)
    {
        control_cmd.vx = 0;
        control_cmd.vy = 0;
        control_cmd.vth = 0;
        control_cmd.lifter_dir = 1;
        if(control_state.fork_max)
        {
            control_cmd.lifter_dir = 0;
            action_step = 3;
        }
    }
    else if(action_step == 3)
    {
        control_cmd.vth = 0;
        control_cmd.vy = 0;
        control_cmd.vx = 0.4;
        if(sqrt(pow(current_loc.point.x-get_path_msg.path_point[get_path_msg.num-1].coor.x,2)
              + pow(current_loc.point.y-get_path_msg.path_point[get_path_msg.num-1].coor.y,2)) < 0.05)
        {
            ROS_INFO_STREAM(GetSystemTime() << "Action Finish!");
            ROS_INFO_STREAM(GetSystemTime() << "X=" << current_loc.point.x << ",Y=" << current_loc.point.y << ",A=" << current_loc.yaw*Radian2Degree);
            control_cmd.vx = 0;
            control_cmd.vy = 0;
            control_cmd.vth = 0;
            action_step = 0;
            auto_state = 0;
            task_state = 0;
            agv_state.state = 0;
            agv_state.move_dir = 0;
            agv_state.targetx = 0;
            agv_state.targety = 0;
        }     
    }
}

void AGVCONTROL::Action_4()
{
    agv_state.state = 2;
    PointMsg current_action_point;
    current_action_point.x = control_state.truck_loc_x;
    current_action_point.y = control_state.slamy;
    current_loc.point.x = control_state.truck_loc_x;
    current_loc.point.y = agv_state.slamy / 1000.0;
    current_loc.yaw = control_state.truck_loc_th + PI;
    LocMsg now_slam;
    now_slam.point.x = control_state.slamx;
    now_slam.point.y = control_state.slamy;
    now_slam.yaw = control_state.slamth;
    if(current_loc.yaw > PI)
        current_loc.yaw = current_loc.yaw - 2*PI;
    if(now_slam.yaw > PI)
        now_slam.yaw = now_slam.yaw - 2*PI;

    // std::cout << GetSystemTime() << "actstep=" << action_step << ",x=" << current_loc.point.x << ",y=" << current_loc.point.y << ",th=" << current_loc.yaw*Radian2Degree << std::endl;

    if(fabs(current_loc.yaw + PI) < 0.0001)
        current_loc.yaw = -1.0 * current_loc.yaw;
    if(fabs(now_slam.yaw + PI) < 0.0001)
        now_slam.yaw = -1.0 * now_slam.yaw;

    PointMsg start_p, target_p, end_p, slamstart_p, slamend_p;
    start_p.x = 0;
    start_p.y = get_path_msg.path_point[get_path_msg.num-1].coor.y;
    target_p.x = 0;
    switch (get_path_msg.path_point[get_path_msg.num-1].act_param)
    {
    case 1:
        target_p.y = 53.78;
        break;
    case 2:
        target_p.y = 52.53;
        break;
    case 3:
        target_p.y = 51.27;
        break;
    case 4:
        target_p.y = 49.98;
        break;
    case 5:
        target_p.y = 48.7;
        break;
    default:
        break;
    }
    end_p.x = 0;
    end_p.y = get_path_msg.path_point[get_path_msg.num-1].coor.y + 1.5;

    slamstart_p.x = get_path_msg.path_point[get_path_msg.num-1].coor.x;
    slamstart_p.y = get_path_msg.path_point[get_path_msg.num-1].coor.y + 1.5;

    slamend_p.x = get_path_msg.path_point[get_path_msg.num-1].coor.x;
    slamend_p.y = get_path_msg.path_point[get_path_msg.num-1].coor.y;

    if(action_step == 0)
    {
        ROS_INFO_STREAM(GetSystemTime() << "TargetY=" << target_p.y);
        action_step = 1;
    }
    if(action_step == 1)
    {
        PointMsg clos_point = GetClosestPoint(current_action_point,start_p,target_p);
        float totaldis = CalculateP2PDis(start_p,target_p);
        float acc_dis = 0.6 / 2.0;
        float dec_dis = 0.6 * 2.0;
        if(dec_dis > 1.8)
            dec_dis = 1.8;
        float cal_v = 0.6;
        if(current_loc.point.y - start_p.y < 0.9)
            cal_v = 0.5;
        float ld = kld*cal_v + 6;
        if(ld > ldmax)
            ld = ldmax;

        PointMsg goal_point;
        goal_point.x = clos_point.x + (target_p.x-start_p.x) * ld / totaldis;
        goal_point.y = clos_point.y + (target_p.y-start_p.y) * ld / totaldis;
        float eld = Error_ld(current_loc.yaw+PI, current_action_point.x, current_action_point.y, goal_point.x, goal_point.y);
        float cal_vth = (2*eld * cal_v) / pow(ld,2);
        if(cal_vth > 0.15)
            cal_vth = 0.15;
        else if(cal_vth < -0.15)
            cal_vth = -0.15;
        cal_v = -1.0 * cal_v;
        control_cmd.vx = cal_v;
        control_cmd.vth = cal_vth;

        if(!control_state.back_dec)
        {
            if(control_state.back_left_dec && !control_state.back_right_dec && !control_state.back_left_stop && !control_state.back_right_stop)
            {
                control_cmd.vth = 0.03;
            }
            else if(control_state.back_right_dec && !control_state.back_left_dec && !control_state.back_left_stop && !control_state.back_right_stop)
            {
                control_cmd.vth = -0.03;
            }
            else if(control_state.back_right_dec && control_state.back_left_dec && !control_state.back_left_stop && !control_state.back_right_stop)
            {
                
            }
            else if(control_state.back_left_stop && !control_state.back_right_stop)
            {
                control_cmd.vth = 0.06;
            }
            else if(!control_state.back_left_stop && control_state.back_right_stop)
            {
                control_cmd.vth = -0.06;
            }
        }

        if(control_state.back_dec && fabs(clos_point.y - target_p.y) < 0.8)
        {
            control_cmd.vx = -0.08;
            if(current_loc.yaw + PI/2.0 < 0)
                control_cmd.vth = 0.01;
            else if(current_loc.yaw + PI/2.0 > 0)
                control_cmd.vth = -0.01;
        }

        if(control_state.back_stop)
        {
            control_cmd.vx = 0;
            control_cmd.vy = 0;
            control_cmd.vth = 0;
            ROS_WARN_STREAM_THROTTLE(1,GetSystemTime() << "CRASH!");
        }

        if(fabs(clos_point.y - target_p.y) < 0.4 && control_state.back_stop)
        {
            ROS_INFO_STREAM(GetSystemTime() << "Got Action Point!");
            ROS_INFO_STREAM(GetSystemTime() << "X=" << current_action_point.x << ",Y=" << current_action_point.y << ",A=" << current_loc.yaw*Radian2Degree);
            control_cmd.vx = 0;
            control_cmd.vy = 0;
            control_cmd.vth = 0;
            action_step = 2;
        }
    }
    else if(action_step == 2)
    {
        control_cmd.vx = 0;
        control_cmd.vy = 0;
        control_cmd.vth = 0;
        control_cmd.lifter_dir = -1;
        ROS_WARN_STREAM_THROTTLE(2, GetSystemTime() << "XiaJiang!");
        if(control_state.fork_min || control_state.fork_height < 20/1000.0)
        {
            control_cmd.lifter_dir = 0;
            action_step = 3;
        }
    }
    else if(action_step == 3)
    {
        control_cmd.vx = 0.5;
        control_cmd.vy = 0;
        control_cmd.vth = 0;
        if(fabs(current_action_point.y - (target_p.y - 1.8)) < 0.1)
        {
            action_step = 4;
        }
    }
    else if(action_step == 4)
    {
        control_cmd.vx = 0.5;
        control_cmd.vy = 0;
        control_cmd.vth = 0;
        control_cmd.lifter_dir = 1;
        if(control_state.fork_max || control_state.fork_height > 0.118)
        {
            action_step = 5;
        }
    }
    else if(action_step == 5)
    {                            
        PointMsg clos_point = GetClosestPoint(current_action_point,target_p,end_p);
        float totaldis = CalculateP2PDis(target_p,end_p);
        float acc_dis = 0.5 / 2.0;
        float dec_dis = 0.5 * 2.0;
        if(dec_dis > 1.8)
            dec_dis = 1.8;
        float cal_v = 0.6;
        if(get_path_msg.path_point[get_path_msg.num-1].act_param == 5)
            cal_v = 0.8;
        float ld = kld*cal_v + 2;
        if(ld > ldmax)
            ld = ldmax;

        PointMsg goal_point;
        goal_point.x = clos_point.x + (end_p.x-target_p.x) * ld / totaldis;
        goal_point.y = clos_point.y + (end_p.y-target_p.y) * ld / totaldis;
        float eld = Error_ld(current_loc.yaw+PI, current_action_point.x, current_action_point.y, goal_point.x, goal_point.y);
        float cal_vth = (2*eld * cal_v) / pow(ld,2);
        if(cal_vth > 0.12)
            cal_vth = 0.12;
        else if(cal_vth < -0.12)
            cal_vth = -0.12;
        cal_v = cal_v;
        control_cmd.vx = cal_v;
        control_cmd.vth = -cal_vth;

        if(fabs(clos_point.y - end_p.y) < 0.3)
        {
            ROS_INFO_STREAM(GetSystemTime() << "Got Action Point!");
            ROS_INFO_STREAM(GetSystemTime() << "X=" << current_action_point.x << ",Y=" << current_action_point.y << ",A=" << current_loc.yaw*Radian2Degree);
            action_step = 6;
        }
    }
    else if(action_step == 6)
    {
        PointMsg clos_point = GetClosestPoint(now_slam.point,slamstart_p,slamend_p);
        float totaldis = CalculateP2PDis(slamstart_p,slamend_p);
        float acc_dis = 0.5 / 2.0;
        float dec_dis = 0.5 * 2.0;
        if(dec_dis > 1.8)
            dec_dis = 1.8;
        float cal_v = 0.6;
        float ld = kld*cal_v + 2;
        if(ld > ldmax)
            ld = ldmax;

        PointMsg goal_point;
        goal_point.x = clos_point.x + (slamend_p.x-slamstart_p.x) * ld / totaldis;
        goal_point.y = clos_point.y + (slamend_p.y-slamstart_p.y) * ld / totaldis;
        float eld = Error_ld(now_slam.yaw+PI, now_slam.point.x, now_slam.point.y, goal_point.x, goal_point.y);
        float cal_vth = (2*eld * cal_v) / pow(ld,2);
        if(cal_vth > 0.12)
            cal_vth = 0.12;
        else if(cal_vth < -0.12)
            cal_vth = -0.12;
        cal_v = cal_v;
        control_cmd.vx = cal_v;
        control_cmd.vth = -cal_vth;

        if(fabs(clos_point.y - slamend_p.y) < 0.2)
        {
            ROS_INFO_STREAM(GetSystemTime() << "Action Finish!");
            ROS_INFO_STREAM(GetSystemTime() << "X=" << now_slam.point.x << ",Y=" << now_slam.point.y << ",A=" << now_slam.yaw*Radian2Degree);
            control_cmd.vx = 0;
            control_cmd.vy = 0;
            control_cmd.vth = 0;
            action_step = 0;
            auto_state = 0;
            task_state = 0;
            agv_state.state = 0;
            agv_state.move_dir = 0;
            agv_state.targetx = 0;
            agv_state.targety = 0;
        }
    }
    if(control_state.back_stop)
    {
        if(control_cmd.vx < 0)
        {
            control_cmd.vx = 0;
            control_cmd.vth = 0;
        }
    }    
    if(control_state.front_left_stop && (action_step == 3 || action_step == 4 || action_step == 5 || action_step == 6))
    {
        control_cmd.vth = -0.07;
    }
    else if(control_state.front_right_stop && (action_step == 3 || action_step == 4 || action_step == 5 || action_step == 6))
    {
        control_cmd.vth = 0.07;
    }
    if((fabs(current_loc.yaw + PI/2.0) > PI/6.0) && (action_step == 1 || action_step == 5))
    {
        control_cmd.vx = 0;
        control_cmd.vth = 0;
        std::cout << "TruckLoc Miss!!!!!!" <<  std::endl;
    }
}

void AGVCONTROL::Action_5()
{
    agv_state.state = 2;
    if(control_state.fork_min)
    {
        if(action_step == 0 && agv_state.charge==0)
        {
            if(control_state.charge_recv_light)
            {   
                charge_action_step = 1;
                action_step = 1;
            }
        }
        else if(charge_action_step == 1)
        {
            control_cmd.Yout[4-1] = 1;
            control_cmd.charge_cmd = 1;
            if(agv_state.charge)
                charge_action_step = 2;
        }
        else if(charge_action_step == 2)
        {
            control_cmd.Yout[4-1] = 1;
            control_cmd.charge_cmd = 1;
            control_cmd.Yout[5-1] = 0;
            control_cmd.Yout[6-1] = 1;
            control_cmd.Yout[7-1] = 1;
            if(agv_cmd.task_pause == 1 || agv_cmd.task_cancel == 1)
                charge_action_step = 3;
        }
        else if(charge_action_step == 3)
        {
            control_cmd.Yout[4-1] = 0;
            control_cmd.charge_cmd = 0;
            control_cmd.Yout[5-1] = 0;
            control_cmd.Yout[6-1] = 1;
            control_cmd.Yout[7-1] = 1;
            if(control_state.charge_recv_light)
                charge_action_step = 4;
        }
        else if(charge_action_step == 4)
        {
            control_cmd.Yout[5-1] = 0;
            control_cmd.Yout[6-1] = 0;
            control_cmd.Yout[7-1] = 0;
            charge_action_step = 0;
            action_step = 0;
        }
    }
    else
    {
        ROS_WARN_STREAM(GetSystemTime() << "Fork! Cannot Charge!");
    }
}

void AGVCONTROL::manual_Action_1()
{
    control_cmd.vx = 0;
    control_cmd.vy = 0;
    control_cmd.vth = 0;
    control_cmd.lifter_dir = 0;
    if(control_state.fork_min)
    {
        // std::cout << "step=" << charge_action_step << std::endl;
        if(charge_action_step == 0 && agv_state.charge==0)
        {
            if(control_state.charge_recv_light)
                charge_action_step = 1;
        }
        else if(charge_action_step == 1)
        {
            control_cmd.Yout[4-1] = 1;
            control_cmd.charge_cmd = 1;
            if(agv_state.charge)
                charge_action_step = 2;
        }
        else if(charge_action_step == 2)
        {
            control_cmd.Yout[4-1] = 1;
            control_cmd.charge_cmd = 1;
            control_cmd.Yout[5-1] = 0;
            control_cmd.Yout[6-1] = 1;
            control_cmd.Yout[7-1] = 1;
            if(agv_cmd.charge_cmd == 0)
                charge_action_step = 3;
        }
        else if(charge_action_step == 3)
        {
            control_cmd.Yout[4-1] = 0;
            control_cmd.charge_cmd = 0;
            control_cmd.Yout[5-1] = 0;
            control_cmd.Yout[6-1] = 1;
            control_cmd.Yout[7-1] = 1;
            if(control_state.charge_recv_light)
                charge_action_step = 4;
        }
        else if(charge_action_step == 4)
        {
            control_cmd.Yout[5-1] = 0;
            control_cmd.Yout[6-1] = 0;
            control_cmd.Yout[7-1] = 0;
            charge_action_step = 0;
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"agv_control");
    ros::NodeHandle n;

    // int pass = key_pass();
	// if(pass == 0)
	// {
	// 	ROS_ERROR("Serial Key Wrong!");
	// 	return 0;
	// }

    n.getParam("/MCU_frequency",MCU_frequency);
    n.getParam("/destIP_MCU",destIP_MCU);
	n.getParam("/dest_PORT_MCU",dest_PORT_MCU);
	n.getParam("/localIP_MCU",localIP_MCU);
	n.getParam("/local_PORT_MCU",local_PORT_MCU);
    n.getParam("/SLAM_frequency",SLAM_frequency);
    n.getParam("/destIP_SLAM",destIP_SLAM);
	n.getParam("/dest_PORT_SLAM",dest_PORT_SLAM);
	n.getParam("/localIP_SLAM",localIP_SLAM);
	n.getParam("/local_PORT_SLAM",local_PORT_SLAM);
    n.getParam("/GENERAL_frequency",GENERAL_frequency);
    n.getParam("/destIP_GENERAL",destIP_GENERAL);
	n.getParam("/dest_PORT_GENERAL",dest_PORT_GENERAL);
	n.getParam("/localIP_GENERAL",localIP_GENERAL);
	n.getParam("/local_PORT_GENERAL",local_PORT_GENERAL);
    n.getParam("/SECLOC_frequency",SECLOC_frequency);
    n.getParam("/destIP_SECLOC",destIP_SECLOC);
	n.getParam("/dest_PORT_SECLOC",dest_PORT_SECLOC);
	n.getParam("/localIP_SECLOC",localIP_SECLOC);
	n.getParam("/local_PORT_SECLOC",local_PORT_SECLOC);

    AGVCONTROL_MCU_UDP      mcu_udp;
    AGVCONTROL_SLAM_UDP     slam_udp;
    AGVCONTROL_GENERAL_UDP  general_udp;
    AGVCONTROL_SECLOC_UDP   secloc_udp;
    AGVCONTROL              agv_control;

    ros::Timer timer_mcu_udp        = n.createTimer(ros::Duration(1.0/MCU_frequency), boost::bind(&AGVCONTROL_MCU_UDP::AGVCONTROL_MCU_UDP_Recv_Send, &mcu_udp));
    ros::Timer timer_slam_udp       = n.createTimer(ros::Duration(1.0/SLAM_frequency), boost::bind(&AGVCONTROL_SLAM_UDP::AGVCONTROL_SLAM_UDP_Recv_Send, &slam_udp));
    ros::Timer timer_general_udp    = n.createTimer(ros::Duration(1.0/GENERAL_frequency), boost::bind(&AGVCONTROL_GENERAL_UDP::AGVCONTROL_GENERAL_UDP_Recv_Send, &general_udp));
    ros::Timer timer_secloc_udp     = n.createTimer(ros::Duration(1.0/SECLOC_frequency), boost::bind(&AGVCONTROL_SECLOC_UDP::AGVCONTROL_SECLOC_UDP_Recv_Send, &secloc_udp));

    ros::AsyncSpinner async_spinner(5);
    async_spinner.start();

    ros::Rate loop_rate(100);

    agv_cmd.manual_auto = 0;

    while(ros::ok())
    {
        agv_control.Get_AUTO_CMD();
        if(agv_cmd.manual_auto == 1)
        {    
            agv_state.manual_auto = 0;
            agv_control.Automatic_Motion_Control();
            agv_control.Automatic_Action_Control();
        }
        if(agv_cmd.manual_auto == 0)
        {
            agv_state.manual_auto = 1;
            agv_control.Manual_Control();
        }
        agv_control.Stop_AGV();
        ros::spinOnce();
        loop_rate.sleep();
    }

    async_spinner.stop();
    ros::shutdown();
    return 0;
}