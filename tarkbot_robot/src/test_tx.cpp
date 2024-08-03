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


#include "RM_robot.h"

/*
 * @功  能  主函数，ROS初始化，调用构造函数初始化
 */
int main(int argc, char** argv)
{
    //ROS初始化 并设置节点名称 
    ros::init(argc,argv,"tarkbot_robot_node");

    ROS_INFO("Tarkbot Robot is Starting ......  ");
    
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

    //参数初始化
    private_nh_.param<std::string>("robot_port", serial_port_, "/dev/ttyTHS1");
    private_nh_.param<int>        ("robot_port_baud", serial_port_baud_, 230400);
    
    //数据初始化
    memset(&pos_data_, 0, sizeof(pos_data_));

    //提示信息，串口端口号和波特率
    ROS_INFO("Tarkbot Robot Set serial %s at %d baud", serial_port_.c_str(), serial_port_baud_);

    //初始化串口
    if(openSerialPort())
    {
        try
        {
            //启动串口接收线程
            boost::thread recvSerial_thread(boost::bind(&RMRobot::recvCallback,this));
            // 启动串口发送线程
            // RMRobot::cmdVelCallback();
        }
        catch(...)
        {
            ROS_INFO("Tarkbot Robot can not open recvSerial_thread, Please check the serial port cable ");

            //关闭节点
            ros::shutdown();
        }
    }
    else
    {
        //关闭节点
        ros::shutdown();
    } 
}


/*RMRobot
 * @功  能  析构函数，对象生命周期结束时系统会调用这个函数
 */
RMRobot::~RMRobot()
{ 
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
uint8_t rx_checksum = 0x66; //帧头部分校验和
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
                // rx_checksum = (0xAA+0x55) + res;	//计算校验和
            }
        }
        else    //=========接收数据
        {
            if(rx_con < 9) // 最后一位
            {
                rx_buf[rx_con] = res;
                rx_con++;
                // rx_checksum = rx_checksum + res;		
                ROS_INFO("res: %d",res); 			
            }
            else    //判断最后1位
            {
                //接收完成，恢复初始状态
                rx_con = 0;					
                
                ROS_INFO("res: %d, rx_checksum: %d",res, rx_checksum);
                //数据校验
                if( res == rx_checksum )  //校验正确
                {	
                    //调用串口数据处理函数

                    recvDataHandle(rx_buf);
                    ROS_INFO("rx_buf: %u", *rx_buf);
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
    if(buffer_data[2] == 0x01)
    {
        //解析机器人速度
        vel_data_.linear_x = ((double)((int16_t)(buffer_data[3]*256+buffer_data[4]))/1000);
        vel_data_.linear_y = ((double)((int16_t)(buffer_data[5]*256+buffer_data[6]))/1000);
        vel_data_.angular_z = ((double)((int16_t)(buffer_data[7]*256+buffer_data[8]))/1000);

        // imu_data_.acc_x =  ((double)((int16_t)(buffer_data[15]*256+buffer_data[16]))/1000);
        // imu_data_.acc_y =  ((double)((int16_t)(buffer_data[17]*256+buffer_data[18]))/1000);
        // imu_data_.acc_z =  ((double)((int16_t)(buffer_data[19]*256+buffer_data[20]))/1000);

        // //解析IMU陀螺仪数据
        // imu_data_.gyro_x  = ((double)((int16_t)(buffer_data[9]*256+buffer_data[10]))/1000);
        // imu_data_.gyro_y  = ((double)((int16_t)(buffer_data[11]*256+buffer_data[12]))/1000);
        // imu_data_.gyro_z  = ((double)((int16_t)(buffer_data[13]*256+buffer_data[14]))/1000);    

        ROS_INFO("linear_x: %f, linear_y: %f, angular_z: %f", vel_data_.linear_x, vel_data_.linear_y, vel_data_.angular_z);
        // ROS_INFO("acc_x: %f, acc_y: %f, acc_z: %f", imu_data_.acc_x, imu_data_.acc_y, imu_data_.acc_z);
        // ROS_INFO("gyro_x: %f, gyro_y: %f, gyro_z: %f", imu_data_.gyro_x, imu_data_.gyro_y, imu_data_.gyro_z);

        // 计算里程计数据
        // pos_data_.pos_x += (vel_data_.linear_x*cos(pos_data_.angular_z) - vel_data_.linear_y*sin(pos_data_.angular_z)) * DATA_PERIOD; 
        // pos_data_.pos_y += (vel_data_.linear_x*sin(pos_data_.angular_z) + vel_data_.linear_y*cos(pos_data_.angular_z)) * DATA_PERIOD; 
        // pos_data_.angular_z += vel_data_.angular_z * DATA_PERIOD;   //绕Z轴的角位移，单位：rad 

    }
}


void RMRobot::cmdVelCallback()
{
    static uint8_t vel_data[11];
    //数据转换
    vel_data[0] = (int16_t)(0.5*1000)>>8;
    vel_data[1] = (int16_t)(0.5*1000);
    vel_data[2] = (int16_t)(0*1000)>>8;
    vel_data[3] = (int16_t)(0*1000);
    vel_data[4] = (int16_t)(0*1000)>>8;
    vel_data[5] = (int16_t)(0*1000);
    ROS_INFO("is in while");
    while(1){
    sendSerialPacket(vel_data, 6, 0x01);
    }
    
}
 
/*
 * @功  能  发送串口数据包，塔克X-Protocol协议
 * @参  数  *pbuf：发送数据指针
 *          len：发送数据长度个数，长度小于64字节
 *          num：帧号，帧编码
 * @返回值	 无
 */
void RMRobot::sendSerialPacket(uint8_t *pbuf, uint8_t len, uint8_t num)
{
	uint8_t i;	
    uint8_t tx_checksum = 0x66;//发送校验和
    uint8_t tx_buf[64];
	
    //判断是否超出长度
	if(len <= 64)
	{
		//获取数据
		tx_buf[0] = 0xAA;    //帧头
		tx_buf[1] = 0x55;    //
		tx_buf[2] = num;    //帧编码
		
		for(i=0; i<len; i++)
		{
			tx_buf[3+i] = *(pbuf+i);
		}
		
		//计算校验和	
		// cnt = 4+len;
		// for(i=0; i<cnt; i++)
		// {
		// 	tx_checksum = tx_checksum + tx_buf[i];
		// }
		tx_buf[i+3] = tx_checksum;

        //计算帧长度
        // cnt = len+5;
		
        //发送数据
        boost::asio::write(*serial_ptr_.get(),boost::asio::buffer(tx_buf, 10 ),err_code_);
	}
}