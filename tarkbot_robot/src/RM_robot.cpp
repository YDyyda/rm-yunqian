/*
 * Copyright 2022 XTARK ROBOTIS CO., LTD.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#include "tarkbot_robot.h"

/*
 * @功  能  主函数，ROS初始化，调用构造函数初始化
 */
int main(int argc, char** argv)
{
    //ROS初始化 并设置节点名称 
    ros::init(argc,argv,"tarkbot_robot_node");

    ROS_INFO("RM Robot is Starting ......  ");
    
    //实例化一个对象
    RMRobot robot;

    return 0;
} 

/*
 * @功  能  构造函数，机器人初始化
 */
RMRobot::RMRobot()
{
        //创建节点句柄
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_("~");

    //frame初始化
    private_nh_.param<std::string>("odom_frame", odom_frame_, "odom"); 
    private_nh_.param<std::string>("base_frame", base_frame_, "base_footprint");
    private_nh_.param<std::string>("imu_frame", imu_frame_, "imu_link");

    //话题消息初始化
    private_nh_.param<std::string>("odom_topic", odom_topic_, "odom"); 
    private_nh_.param<std::string>("imu_topic", imu_topic_, "imu");
    // private_nh_.param<std::string>("battery_topic", bat_topic_, "bat_vol");
    private_nh_.param<std::string>("cmd_vel_topic", cmd_vel_topic_, "cmd_vel");
    // private_nh_.param<std::string>("beep_topic", beep_topic_, "beep");

    //参数初始化
    private_nh_.param<std::string>("robot_port", serial_port_, "/dev/ttyTHS1");
    private_nh_.param<int>        ("robot_port_baud", serial_port_baud_, 230400);
    private_nh_.param<bool>       ("pub_odom_tf", pub_odom_tf_, true);
    
    //车型选择参数
    private_nh_.param<std::string>("robot_type_send", robot_type_send_, "r20_mec");

    //实例化发布者对象
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>(odom_topic_, 50);
    imu_pub_  = nh_.advertise<sensor_msgs::Imu>(imu_topic_, 50);    
    // bat_pub_  = nh_.advertise<std_msgs::Float32>(bat_topic_, 10);

    //实例化订阅者对象
    cmd_vel_sub_ = nh_.subscribe<geometry_msgs::Twist>(cmd_vel_topic_, 100, &RMRobot::cmdVelCallback, this);
    // beep_sub_ = nh_.subscribe<std_msgs::Int8>(beep_topic_, 10,&RMRobot::beep_callback,this);
    // light_server_ = nh_.advertiseService("Light_Server",&RMRobot::light_ser_back, this);
    //数据初始化
    memset(&pos_data_, 0, sizeof(pos_data_));

    //提示信息，串口端口号和波特率
    ROS_INFO(" RM Set serial %s at %d baud", serial_port_.c_str(), serial_port_baud_);

    //初始化串口
    if(openSerialPort())
    {
        try
        {
            //启动串口接收线程
            boost::thread recvSerial_thread(boost::bind(&RMRobot::recvCallback,this));
        }
        catch(...)
        {
            ROS_INFO("RM Robot can not open recvSerial_thread, Please check the serial port cable ");

            //关闭节点
            ros::shutdown();
        }
    }
    else
    {
        //关闭节点
        ros::shutdown();
    } 

    ROS_INFO("  RM Robot is Connected to C board ");

    //动态参数配置  
    // dynamic_reconfigure::Server<tarkbot_robot::robotConfig> reconfig_server;
	// dynamic_reconfigure::Server<tarkbot_robot::robotConfig>::CallbackType f;
	// f = boost::bind(&RMRobot::dynamicReconfigCallback,this,_1,_2);
	// reconfig_server.setCallback(f);

    //向底层发送车型数据  
    // static uint8_t cartype_data[1];
    // if(robot_type_send_== "r20_mec")
    // cartype_data[0] = 1;
    // if(robot_type_send_== "r20_fwd")
    // cartype_data[0] = 2;
    // if(robot_type_send_== "r20_akm")
    // cartype_data[0] = 3;
    // if(robot_type_send_== "r20_twd")
    // cartype_data[0] = 4;
    // if(robot_type_send_== "r20_tak")
    // cartype_data[0] = 5;
    // if(robot_type_send_== "r20_omni")
    // cartype_data[0] = 6;
    // sendSerialPacket(cartype_data, 1, ID_ROS2CTR_RTY);
    // ROS_INFO("Xtark Robot Set car type %s", robot_type_send_.c_str());

    //提示信息，发布订阅的话题消息
    ROS_INFO("RM Robot setup publisher  on odom [nav_msgs/Odometry]");
    ROS_INFO("RM Robot setup publisher  on imu [sensor_msgs/Imu]");
    // ROS_INFO("RN Robot setup publisher  on bat_vol [float]");
    ROS_INFO("RM Robot setup subscriber on cmd_vel [geometry_msgs/Twist]");

    //机器人启动完成提示信息
    ROS_INFO("RM Robot initialization completed, is Running!");

    //发送0数据
    uint8_t init_vel_data[11];
    //数据转换
    init_vel_data[0] = 0;
    init_vel_data[1] = 0;
    init_vel_data[2] = 0;
    init_vel_data[3] = 0;
    init_vel_data[4] = 0;
    init_vel_data[5] = 0;
    //发送串口数据
    sendSerialPacket(init_vel_data, 6, ID_ROS2CTR_VEL);


    //进入循环处理回调函数
    ros::spin();
}



/*RMRobot
 * @功  能  析构函数，对象生命周期结束时系统会调用这个函数
 */
RMRobot::~()
{ 
    static uint8_t vel_data[11];
    // static uint8_t beep_data[1];

    //发送静止指令
    vel_data[0] = 0;
    vel_data[1] = 0;
    vel_data[2] = 0;
    vel_data[3] = 0;
    vel_data[4] = 0;
    vel_data[5] = 0;
    sendSerialPacket(vel_data, 6, ID_ROS2CTR_VEL);

    //数据转换
    // beep_data[0] = 0;

    //发送串口数据
    // sendSerialPacket(beep_data, 1, ID_ROS2CTR_BEEP);

    //关闭串口
    closeSerialPort();

    //提示信息
    ROS_INFO("Tarkbot Robot shutting down."); 
}

/*
 * @功  能  初始化串口
 */
bool RMRobot::openSerialPort()
{
    //检查串口是否已经被打开
    if(serial_ptr_)
    {
        ROS_INFO("The SerialPort is already opened!\r\n");
        return false;
    }

    //开打串口
    serial_ptr_ = serial_ptr(new boost::asio::serial_port(io_service_));
    serial_ptr_->open(serial_port_,err_code_);

    //串口是否正常打开
    if(err_code_)
    {
        ROS_INFO("Open Port: %s Failed! Aboart!",serial_port_.c_str());
        return false;
    }

    //初始化串口参数
    serial_ptr_->set_option(boost::asio::serial_port_base::baud_rate(serial_port_baud_));
    serial_ptr_->set_option(boost::asio::serial_port_base::character_size(8));
    serial_ptr_->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    serial_ptr_->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    serial_ptr_->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
    
    return true;
}

/*
 * @功  能  关闭串口
 */
void RMRobot::closeSerialPort()
{
    //如果串口被打开，则关闭串口
    if(serial_ptr_)
    {
        serial_ptr_->cancel();
        serial_ptr_->close();
        serial_ptr_.reset();
    }

    //
    io_service_.stop();
    io_service_.reset();
}


//协议解析变量
uint8_t rx_con = 0;  //接收计数器
uint8_t rx_checksum; //帧头部分校验和
uint8_t rx_buf[60];  //接收缓冲
/*
 * @功  能  串口接收回调函数，接收塔克X-Protocol
 */
void RMRobot::recvCallback()
{
    //接收数据
	uint8_t res;

    while(1)
    {
        //读取串口数据
        boost::asio::read(*serial_ptr_.get(), boost::asio::buffer(&res, 1), err_code_);

        //塔克X-Protocol协议解析数据
        if(rx_con < 3)    //=========接收帧头 + 长度
        {
            if(rx_con == 0)  //接收帧头1 0xAA
            {
                if(res == 0xAA)
                {
                    rx_buf[0] = res;
                    rx_con = 1;					
                }
                else
                {
                    
                }
            }else if(rx_con == 1) //接收帧头2 0x55
            {
                if(res == 0x55)
                {
                    rx_buf[1] = res;
                    rx_con = 2;
                }
                else
                {
                    rx_con = 0;						
                }				
            }
            else  //接收数据长度
            {
                rx_buf[2] = res;
                rx_con = 3;
                rx_checksum = (0xAA+0x55) + res;	//计算校验和
            }
        }
        else    //=========接收数据
        {
            if(rx_con < (rx_buf[2]-1) )
            {
                rx_buf[rx_con] = res;
                rx_con++;
                rx_checksum = rx_checksum + res;					
            }
            else    //判断最后1位
            {
                //接收完成，恢复初始状态
                rx_con = 0;					
                
                //数据校验
                if( res == rx_checksum )  //校验正确
                {	
                    //调用串口数据处理函数
                    recvDataHandle(rx_buf);
                }	
            }
        }         
    }
}

/*
 * @功  能  串口接收数据处理
 */
void RMRobot::recvDataHandle(uint8_t* buffer_data)
{
    //机器人数据
    if(buffer_data[3] == ID_CPR2ROS_DATA)
    {
        //解析IMU加速度数据
        imu_data_.acc_x =  ((double)((int16_t)(buffer_data[4]*256+buffer_data[5]))*ACC_RATIO);
        imu_data_.acc_y =  ((double)((int16_t)(buffer_data[6]*256+buffer_data[7]))*ACC_RATIO);
        imu_data_.acc_z =  ((double)((int16_t)(buffer_data[8]*256+buffer_data[9]))*ACC_RATIO);

        //解析IMU陀螺仪数据
        imu_data_.gyro_x  = ((double)((int16_t)(buffer_data[10]*256+buffer_data[11]))*GYRO_RATIO);
        imu_data_.gyro_y  = ((double)((int16_t)(buffer_data[12]*256+buffer_data[13]))*GYRO_RATIO);
        imu_data_.gyro_z  = ((double)((int16_t)(buffer_data[14]*256+buffer_data[15]))*GYRO_RATIO);

        //解析机器人速度
        vel_data_.linear_x = ((double)((int16_t)(buffer_data[16]*256+buffer_data[17]))/1000);
        vel_data_.linear_y = ((double)((int16_t)(buffer_data[18]*256+buffer_data[19]))/1000);
        vel_data_.angular_z = ((double)((int16_t)(buffer_data[20]*256+buffer_data[21]))/1000);

        //解析电压值
        bat_vol_data_ = (double)(((buffer_data[22]<<8)+buffer_data[23]))/100;

        //计算里程计数据
        pos_data_.pos_x += (vel_data_.linear_x*cos(pos_data_.angular_z) - vel_data_.linear_y*sin(pos_data_.angular_z)) * DATA_PERIOD; 
        pos_data_.pos_y += (vel_data_.linear_x*sin(pos_data_.angular_z) + vel_data_.linear_y*cos(pos_data_.angular_z)) * DATA_PERIOD; 
        pos_data_.angular_z += vel_data_.angular_z * DATA_PERIOD;   //绕Z轴的角位移，单位：rad 

        //计算IMU四元数数据
        calculateImuQuaternion(imu_data_);

        //发布话题消息
        publishOdom();     //发布里程计话题
		publishImu();      //发布IMU传感器话题
		publishBatVol();   //发布电池电压话题

        //发布odom里程计TF坐标变换（odom --> base_footprint）
        publishOdomTF();
    }
}

/*
 * @功  能  发布IMU话题消息
 */
void RMRobot::publishImu()
{
    //获取数据
    imu_msgs_.header.stamp = ros::Time::now();
    imu_msgs_.header.frame_id = imu_frame_;
    imu_msgs_.angular_velocity.x = imu_data_.gyro_x;
    imu_msgs_.angular_velocity.y = imu_data_.gyro_y;
    imu_msgs_.angular_velocity.z = imu_data_.gyro_z;
    imu_msgs_.linear_acceleration.x = imu_data_.acc_x;
    imu_msgs_.linear_acceleration.y = imu_data_.acc_y;
    imu_msgs_.linear_acceleration.z = imu_data_.acc_z;
    imu_msgs_.orientation.x = orient_data_.x; 
    imu_msgs_.orientation.y = orient_data_.y; 
    imu_msgs_.orientation.z = orient_data_.z;
    imu_msgs_.orientation.w = orient_data_.w;

    //不使用姿态角度
    imu_msgs_.orientation.x = 0; 
    imu_msgs_.orientation.y = 0; 

    //协防差矩阵
    imu_msgs_.orientation_covariance = {  1e6,    0,    0,
                                            0,  1e6,    0,
                                            0,    0, 1e-6};
    imu_msgs_.angular_velocity_covariance = { 1e6,    0,    0,
                                                0,  1e6,    0,
                                                0,    0, 1e-6};
    imu_msgs_.linear_acceleration_covariance = {    0,    0,    0,
                                                    0,    0,    0,
                                                    0,    0,    0};

    //发布
    imu_pub_.publish(imu_msgs_);
}

/*
 * @功  能  发布odom里程计话题消息
 */
void RMRobot::publishOdom()
{
    //计算里程计四元数
    tf2::Quaternion odom_quat;
    odom_quat.setRPY(0,0,pos_data_.angular_z);       

    //获取数据
    odom_msgs_.header.stamp    = ros::Time::now();
    odom_msgs_.header.frame_id = odom_frame_;
    odom_msgs_.child_frame_id  = base_frame_;
    odom_msgs_.pose.pose.position.x = pos_data_.pos_x;
    odom_msgs_.pose.pose.position.y = pos_data_.pos_y;
    odom_msgs_.pose.pose.position.z = 0;  //高度为0
    odom_msgs_.pose.pose.orientation.x = odom_quat.getX();
    odom_msgs_.pose.pose.orientation.y = odom_quat.getY();
    odom_msgs_.pose.pose.orientation.z = odom_quat.getZ();
    odom_msgs_.pose.pose.orientation.w = odom_quat.getW();
    odom_msgs_.twist.twist.linear.x = vel_data_.linear_x;
    odom_msgs_.twist.twist.linear.y = vel_data_.linear_y;
    odom_msgs_.twist.twist.angular.z = vel_data_.angular_z;

    //里程计协防差矩阵，用于robt_pose_ekf功能包，静止和运动使用不同的参数
    if(vel_data_.linear_x==0 && vel_data_.linear_y==0 && vel_data_.angular_z==0)
    {
        //机器人静止时，IMU水平陀螺仪会存在零飘，编码器没有误差，编码器数据权重增加
        odom_msgs_.pose.covariance = {   1e-9,    0,    0,    0,    0,    0, 
                                            0, 1e-3, 1e-9,    0,    0,    0,
                                            0,    0,  1e6,    0,    0,    0,
                                            0,    0,    0,  1e6,    0,    0,
                                            0,    0,    0,    0,  1e6,    0,
                                            0,    0,    0,    0,    0, 1e-9 };

        odom_msgs_.twist.covariance = {  1e-9,    0,    0,    0,    0,    0, 
                                            0, 1e-3, 1e-9,    0,    0,    0,
                                            0,    0,  1e6,    0,    0,    0,
                                            0,    0,    0,  1e6,    0,    0,
                                            0,    0,    0,    0,  1e6,    0,
                                            0,    0,    0,    0,    0, 1e-9 };
    }
    else
    {
        //机器人运动时，轮子滑动编码器误差增加，IMU陀螺仪数据更加准确，IMU数据权重增加
        odom_msgs_.pose.covariance = {   1e-3,    0,    0,    0,    0,    0, 
                                            0, 1e-3,    0,    0,    0,    0,
                                            0,    0,  1e6,    0,    0,    0,
                                            0,    0,    0,  1e6,    0,    0,
                                            0,    0,    0,    0,  1e6,    0,
                                            0,    0,    0,    0,    0,  1e3 };

        odom_msgs_.twist.covariance = {  1e-3,    0,    0,    0,    0,    0, 
                                            0, 1e-3,    0,    0,    0,    0,
                                            0,    0,  1e6,    0,    0,    0,
                                            0,    0,    0,  1e6,    0,    0,
                                            0,    0,    0,    0,  1e6,    0,
                                            0,    0,    0,    0,    0,  1e3 };
    }

    //发布
    odom_pub_.publish(odom_msgs_);
}

/*
 * @功  能  发布电池电压话题消息
 */
// void RMRobot::publishBatVol()
// {
//     //获取数据
//     bat_msgs_.data = bat_vol_data_;

//     //发布
//     bat_pub_.publish(bat_msgs_);
// }

/*
 * @功  能  发布里程计到base_footprint的TF坐标变换
 */
void RMRobot::publishOdomTF()
{
    //发布里程计到footprint坐标变换
    if(pub_odom_tf_ == true)
    {
        //计算里程计TF四元数
        tf2::Quaternion q;
        q.setRPY(0,0,pos_data_.angular_z);

        //填充数据
        transform_stamped_.header.stamp    = ros::Time::now();
        transform_stamped_.header.frame_id = odom_frame_;
        transform_stamped_.child_frame_id  = base_frame_;

        transform_stamped_.transform.translation.x = pos_data_.pos_x;
        transform_stamped_.transform.translation.y = pos_data_.pos_y;
        transform_stamped_.transform.translation.z = 0.0;

        transform_stamped_.transform.rotation.x = q.x();
        transform_stamped_.transform.rotation.y = q.y();
        transform_stamped_.transform.rotation.z = q.z();
        transform_stamped_.transform.rotation.w = q.w();

        //发布TF坐标变换
        transform_broadcaster_.sendTransform(transform_stamped_);
    }
}

/*
 * @功  能  cmd_vel 速度话题订阅回调函数，根据订阅的指令通过串口发指令控制下位机
 */
void RMRobot::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    static uint8_t vel_data[11];

    //数据转换
    vel_data[0] = (int16_t)(msg->linear.x*1000)>>8;
    vel_data[1] = (int16_t)(msg->linear.x*1000);
    vel_data[2] = (int16_t)(msg->linear.y*1000)>>8;
    vel_data[3] = (int16_t)(msg->linear.y*1000);
    vel_data[4] = (int16_t)(msg->angular.z*1000)>>8;
    vel_data[5] = (int16_t)(msg->angular.z*1000);

    //发送串口数据
    sendSerialPacket(vel_data, 6, ID_ROS2CTR_VEL);
}

/*
 * @功  能  灯光服务订阅回调函数，根据订阅的指令通过串口发指令控制下位机
 */
// bool RMRobot::light_ser_back(tarkbot_robot::Light_Set::Request& req, tarkbot_robot::Light_Set::Response& resp)
// {

//     static uint8_t send_buff[11];

//     //数据转换
//     send_buff[0] = (uint8_t)(req.RGB_M_);
//     send_buff[1] = (uint8_t)(req.RGB_S_);
//     send_buff[2] = (uint8_t)(req.RGB_T_);
//     send_buff[3] = (uint8_t)(req.RGB_R_);
//     send_buff[4] = (uint8_t)(req.RGB_G_);
//     send_buff[5] = (uint8_t)(req.RGB_B_);

//     if (send_buff[0] < 0 || send_buff[3] < 0 || send_buff[4] < 0 || send_buff[5] < 0 || send_buff[0] > 6)
//     {
//         ROS_ERROR("提交的数据异常:数据不可以为负数");
//         return false;
//     }

//     //发送串口数据
//     sendSerialPacket(send_buff, 6, ID_ROS2CTR_LGT);
//     ROS_INFO("Light Control : M: %d S: %d T: %d R: %d, G: %d, B: %d", send_buff[0],send_buff[1],send_buff[2],send_buff[3],send_buff[4],send_buff[5]);


//     //如果没有异常，那么将结果赋值给 resp
//     resp.result = "Success change light";
//     return true;
// }

/*
 * @功  能  beep 蜂鸣器话题订阅回调函数，根据订阅的指令通过串口发指令控制下位机
 */
// void RMRobot::beep_callback(const std_msgs::Int8::ConstPtr& beep)
// {
//     static uint8_t beep_data[1];

//     //数据转换
//     beep_data[0] = (uint8_t)(beep->data);

//     //发送串口数据
//     sendSerialPacket(beep_data, 1, ID_ROS2CTR_BEEP);
// }


/*
 * @功  能  动态参数配置回调函数
 */
// void RMRobot::dynamicReconfigCallback(tarkbot_robot::robotConfig &config,uint32_t level)
// {       
//     uint8_t data[1];  
//     uint8_t data_light[6];
//     static bool change_enble;
//     //IMU校准指令
//     if(config.imu_calibrate == true)
//     {
//         ROS_INFO("Calibrating the IMU, Please hold the robot stationary for 5 seconds. ");

//         //发送IMU校准指令
//         data[0] = 0x55;
//         sendSerialPacket(data, 2, ID_ROS2CTR_IMU);   

//         ros::Duration(0.5).sleep();

//         //校准复位
//         config.imu_calibrate = false;
//     }

//     if(config.light_calibrate == true)
//     {
//         ROS_INFO("Calibrating the light. ");

//         //发送灯光校准指令
//         data[0] = 0x55;
//         sendSerialPacket(data, 2, ID_ROS2CTR_LST);   

//         ros::Duration(0.5).sleep();

//         //校准复位
//         config.light_calibrate = false;
//     }


//     //发送常亮灯光控制
//     if(((config.RGB_M != RGB_M_)||(config.RGB_R != RGB_R_)||(config.RGB_G != RGB_G_)||(config.RGB_G != RGB_B_))&&(change_enble == 1))
//     {
        
//         data_light[0] = (uint8_t)(config.RGB_M);
//         data_light[1] = (uint8_t)(config.RGB_S);
//         data_light[2] = (uint8_t)(config.RGB_T);
//         data_light[3] = (uint8_t)(config.RGB_R);
//         data_light[4] = (uint8_t)(config.RGB_G);
//         data_light[5] = (uint8_t)(config.RGB_B);
//         sendSerialPacket(data_light, 6, ID_ROS2CTR_LGT);
//         ROS_INFO("Tarkbot Robot Set RGB_M_reconfig:%d, RGB_S_reconfig:%d, RGB_T_reconfig:%d, color_R:%d, color_G:%d, color_B:%d", config.RGB_M,config.RGB_S,config.RGB_T,config.RGB_R,config.RGB_G,config.RGB_B);         
//     }
//     if(change_enble==0)
//     {
//         change_enble = 1;
//     }
// }

/*
 * @功  能  发送串口数据包，塔克X-Protocol协议
 * @参  数  *pbuf：发送数据指针
 *          len：发送数据长度个数，长度小于64字节
 *          num：帧号，帧编码
 * @返回值	 无
 */
void RMRobot::sendSerialPacket(uint8_t *pbuf, uint8_t len, uint8_t num)
{
	uint8_t i,cnt;	
    uint8_t tx_checksum = 0;//发送校验和
    uint8_t tx_buf[64];
	
    //判断是否超出长度
	if(len <= 64)
	{
		//获取数据
		tx_buf[0] = 0xAA;    //帧头
		tx_buf[1] = 0x55;    //
		tx_buf[2] = len+5;  //根据输出长度计算帧长度
		tx_buf[3] = num;    //帧编码
		
		for(i=0; i<len; i++)
		{
			tx_buf[4+i] = *(pbuf+i);
		}
		
		//计算校验和	
		cnt = 4+len;
		for(i=0; i<cnt; i++)
		{
			tx_checksum = tx_checksum + tx_buf[i];
		}
		tx_buf[i] = tx_checksum;

        //计算帧长度
        cnt = len+5;
		
        //发送数据
        boost::asio::write(*serial_ptr_.get(),boost::asio::buffer(tx_buf,cnt),err_code_);
	}
}



/***************四元数计算**************************************************/
volatile float twoKp = 1.0f;     // 2 * proportional gain (Kp)
volatile float twoKi = 0.0f;     // 2 * integral gain (Ki)
 // quaternion of sensor frame relative to auxiliary frame
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;         
 // integral error terms scaled by Ki
volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;
volatile const float sampling_period  = DATA_PERIOD;

float invSqrt(float number);

/*
 * @功  能  计算IMU四元数
 */
void RMRobot::calculateImuQuaternion(struct imu_data imu_cel)
{
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;
    float roll,pitch,yaw ;

    //首先把加速度计采集到的值(三维向量)转化为单位向量，即向量除以模
    recipNorm = invSqrt(imu_cel.acc_x * imu_cel.acc_x + imu_cel.acc_y * imu_cel.acc_y + imu_cel.acc_z * imu_cel.acc_z);

    imu_cel.acc_x *= recipNorm;
    imu_cel.acc_y *= recipNorm;
    imu_cel.acc_z *= recipNorm;      

    // 把四元数换算成方向余弦中的第三行的三个元素
    halfvx = q1 * q3 - q0 * q2;
    halfvy = q0 * q1 + q2 * q3;
    halfvz = q0 * q0 - 0.5f + q3 * q3;

    //误差是估计的重力方向和测量的重力方向的交叉乘积之和
    halfex = (imu_cel.acc_y * halfvz - imu_cel.acc_z * halfvy);
    halfey = (imu_cel.acc_z * halfvx - imu_cel.acc_x * halfvz);
    halfez = (imu_cel.acc_x * halfvy - imu_cel.acc_y * halfvx);

    // 计算并应用积分反馈（如果启用）
    if(twoKi > 0.0f) 
    {
        integralFBx += twoKi * halfex * sampling_period;  // integral error scaled by Ki
        integralFBy += twoKi * halfey * sampling_period;
        integralFBz += twoKi * halfez * sampling_period;
        imu_cel.gyro_x += integralFBx;        // apply integral feedback
        imu_cel.gyro_y += integralFBy;
        imu_cel.gyro_z += integralFBz;
    }
    else 
    {
        integralFBx = 0.0f;       // prevent integral windup
        integralFBy = 0.0f;
        integralFBz = 0.0f;
    }
    // Apply proportional feedback
    imu_cel.gyro_x += twoKp * halfex;
    imu_cel.gyro_y += twoKp * halfey;
    imu_cel.gyro_z += twoKp * halfez;        

    // Integrate rate of change of quaternion
    imu_cel.gyro_x *= (0.5f * sampling_period);   // pre-multiply common factors
    imu_cel.gyro_y *= (0.5f * sampling_period);
    imu_cel.gyro_z *= (0.5f * sampling_period);

    qa = q0;
    qb = q1;
    qc = q2;

    q0 += (-qb * imu_cel.gyro_x - qc * imu_cel.gyro_y - q3 * imu_cel.gyro_z);
    q1 += (qa * imu_cel.gyro_x + qc * imu_cel.gyro_z - q3 * imu_cel.gyro_y);
    q2 += (qa * imu_cel.gyro_y - qb * imu_cel.gyro_z + q3 * imu_cel.gyro_x);
    q3 += (qa * imu_cel.gyro_z + qb * imu_cel.gyro_y - qc * imu_cel.gyro_x); 

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);

    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    //计算结果赋值到
    orient_data_.w = q0;
    orient_data_.x = q1;
    orient_data_.y = q2;
    orient_data_.z = q3;

    //计算欧拉角
    // roll = atan2f(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2);
	// pitch = asinf(-2.0f * (q1*q3 - q0*q2));
	// yaw = atan2f(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3);
    // ROS_INFO("IMU:%f  %f  %f ",roll,pitch,yaw);

}

/*
 * @功  能  平方根倒数 求四元数
 */
float invSqrt(float x)
{
    volatile long i;
    volatile float halfx, y;
    volatile const float f = 1.5F;

    halfx = x * 0.5F;
    y = x;
    i = * (( long * ) &y);
    
    i = 0x5f375a86 - ( i >> 1 );
    y = * (( float * ) &i);
    y = y * ( f - ( halfx * y * y ) );

    return y;
}