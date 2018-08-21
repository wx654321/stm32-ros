/*********************************
 * 串口节点，订阅cmd_vel话题并发布odometry话题
 * 从cmd_vel话题中分解出速度值通过串口送到移动底盘
 * 从底盘串口接收里程消息整合到odometry话题用于发布
 * 
 * @StevenShi
 * *******************************/
#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#define	sBUFFERSIZE	15//send buffer size 串口发送缓存长度
#define	rBUFFERSIZE	15//receive buffer size 串口接收缓存长度
unsigned char s_buffer[sBUFFERSIZE];//发送缓存
unsigned char r_buffer[rBUFFERSIZE];//接收缓存

/************************************
 * 串口数据发送格式共15字节
 * head head linear_v_x  linear_v_y angular_v  CRC
 * 0xff 0xff float       float      float      u8
 * **********************************/
/**********************************************************
 * 串口接收数据格式共15字节
 * head head  x-speed y-speed angular-speed  CRC
 * 0xaa 0xaa  float   float   float          u8
 * ********************************************************/

//联合体，用于浮点数与16进制的快速转换
typedef union{
	unsigned char cvalue[4];
	float fvalue;
}float_union;

typedef union{
	unsigned char cvalue[4];
	int ivalue;
}int_union;

serial::Serial ser;

/**********************************************************
 * 数据打包，将获取的cmd_vel信息打包并通过串口发送
 * ********************************************************/
void data_pack(const geometry_msgs::Twist& cmd_vel){
	//unsigned char i;
	//float_union Vx,Vy,Ang_v;
	int_union Vx,Vy,Ang_v;
	float x,y,z;
	int x1,y1,z1;
	x=cmd_vel.linear.x;
	y=cmd_vel.linear.y;
	z=cmd_vel.angular.z;

	//将小数转化为整数发送

	Vx.ivalue = int(100*cmd_vel.linear.x);
	Vy.ivalue = int(100*cmd_vel.linear.y);
	Ang_v.ivalue = int(100*cmd_vel.angular.z);
	
	memset(s_buffer,0,sizeof(s_buffer));

	//正负标志位判断
	if(x>=0)
	{
		s_buffer[5] = 0xaa;
		Vx.ivalue = Vx.ivalue;
	}
	else
	{
		s_buffer[5] = 0xbb;
		Vx.ivalue = -Vx.ivalue;
	}

	if(y>=0)
	{
		s_buffer[9] = 0xaa;
		Vy.ivalue = Vy.ivalue;
	}
	else
	{
		s_buffer[9] = 0xbb;
		Vy.ivalue = -Vy.ivalue;
	}

	if(z>=0)
	{
		s_buffer[13] = 0xaa;
		Ang_v.ivalue = Ang_v.ivalue;
	}
	else
	{
		s_buffer[13] = 0xbb;
		Ang_v.ivalue = -Ang_v.ivalue;
	}

	//数据打包
	s_buffer[0] = 0xff;
	s_buffer[1] = 0xff;
	//Vx
	s_buffer[2] = Vx.cvalue[0];
	s_buffer[3] = Vx.cvalue[1];
	s_buffer[4] = Vx.cvalue[2];
	//s_buffer[5] = Vx.cvalue[3];
	//Vy
	s_buffer[6] = Vy.cvalue[0];
	s_buffer[7] = Vy.cvalue[1];
	s_buffer[8] = Vy.cvalue[2];
	//s_buffer[9] = Vy.cvalue[3];
	//Ang_v
	s_buffer[10] = Ang_v.cvalue[0];
	s_buffer[11] = Ang_v.cvalue[1];
	s_buffer[12] = Ang_v.cvalue[2];
	//s_buffer[13] = Ang_v.cvalue[3];
	//CRC
	s_buffer[14] = s_buffer[2]^s_buffer[3]^s_buffer[4]^s_buffer[5]^s_buffer[6]^s_buffer[7]^
					s_buffer[8]^s_buffer[9]^s_buffer[10]^s_buffer[11]^s_buffer[12]^s_buffer[13];
	/*
	for(i=0;i<15;i++){
		ROS_INFO("0x%02x",s_buffer[i]);
	}
	*/
	ser.write(s_buffer,sBUFFERSIZE);
	
}
//接收数据分析与校验
unsigned char data_analysis(unsigned char *buffer)
{
	unsigned char ret=0,csum;
	int i;
	if((buffer[0]==0xaa) && (buffer[1]==0xaa))
	{
		csum = buffer[2]^buffer[3]^buffer[4]^buffer[5]^buffer[6]^buffer[7]^
				buffer[8]^buffer[9]^buffer[10]^buffer[11]^buffer[12]^buffer[13];
		ROS_INFO("check sum:0x%02x",csum);
		if(csum == buffer[14])
		{
			ret = 1;//校验通过，数据包正确
		}
		else 
		  ret =0;//校验失败，丢弃数据包
	}
	
	//for(i=0;i<rBUFFERSIZE;i++)
	  //ROS_INFO("0x%02x",buffer[i]);
	
	return ret;

}

//订阅turtle1/cmd_vel话题的回调函数，用于显示速度以及角速度
void cmd_vel_callback(const geometry_msgs::Twist& cmd_vel){
	ROS_INFO("I heard linear velocity: x-[%f],y-[%f],",cmd_vel.linear.x,cmd_vel.linear.y);
	ROS_INFO("I heard angular velocity: [%f]",cmd_vel.angular.z);
	std::cout << "Twist Received" << std::endl;	
	data_pack(cmd_vel);
}
int main (int argc, char** argv){
    ros::init(argc, argv, "my_serial_node");
    ros::NodeHandle nh;

	//订阅/turtle1/cmd_vel话题用于测试 $ rosrun turtlesim turtle_teleop_key
	ros::Subscriber write_sub = nh.subscribe("cmd_vel",1000,cmd_vel_callback);
	//发布里程计话题 odom
	ros::Publisher read_pub = nh.advertise<nav_msgs::Odometry>("odom",1000);

    try
    {
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(9600);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
        return -1;
    }
	//定义tf 对象
	static tf::TransformBroadcaster odom_broadcaster;
	//定义tf发布时需要的类型消息
	geometry_msgs::TransformStamped odom_trans;
	//定义里程计消息对象
	nav_msgs::Odometry odom;
	//定义四元数变量
	geometry_msgs::Quaternion odom_quat;
	//位置 速度 角速度
	//float_union posx,posy,vx,vy,va,yaw;
	int_union vx,vy,va;
	//定义时间
	ros::Time current_time, last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();
	      double x = 0.0;
		  double y = 0.0;
	      double th = 0.0;
		  double m = 0.0;
		  double n = 0.0;
		  double q = 0.0;
		  double M = 0.0;
		  double N = 0.0;
		  double Q = 0.0;

    //10hz频率执行
    ros::Rate loop_rate(50);
    while(ros::ok()){

        ros::spinOnce();
		
        if(ser.available()){
			current_time = ros::Time::now();
            //ROS_INFO_STREAM("Reading from serial port");
			ser.read(r_buffer,rBUFFERSIZE);
			//int i;
			//for(i=0;i<rBUFFERSIZE;i++)
			//ROS_INFO("[0x%02x]",r_buffer[i]);
			//ROS_INFO_STREAM("End reading from serial port");
			
			if(data_analysis(r_buffer) != 0){
				int i;
				for(i=0;i<3;i++){
					//posx.cvalue[i] = r_buffer[2+i];//x 坐标
					//posy.cvalue[i] = r_buffer[6+i];//y 坐标
					vx.cvalue[i] = r_buffer[2+i];//车轮1方向速度
					vx.cvalue[3] = 0x00;
					vy.cvalue[i] = r_buffer[6+i];//车轮2方向速度
					vy.cvalue[3] = 0x00;
					va.cvalue[i] = r_buffer[10+i];//车轮3方向速度
					va.cvalue[3] = 0x00;
					//yaw.cvalue[i] = r_buffer[14+i];	//yaw 偏航角
				}	
          //由车的速度求出车的位置
		if(r_buffer[5]==0x1A)
		  vx.ivalue = vx.ivalue;
		else
		  vx.ivalue = -vx.ivalue;
		if(r_buffer[9]==0x2A)
		  vy.ivalue = vy.ivalue;
		else
		  vy.ivalue = -vy.ivalue;
		if(r_buffer[13]==0x3A)
		  va.ivalue = va.ivalue;
		else
		  va.ivalue = -va.ivalue;
	//ROS_INFO_STREAM("Reading from serial port");
	//ROS_INFO("I heard 1.2 velocity: x-[%d],y-[%d],",vx.ivalue,vy.ivalue);
	//ROS_INFO("I heard 3 velocity: [%d]",va.ivalue);
	//ROS_INFO_STREAM("End reading from serial port");
		  double m = vx.ivalue*0.010000;
		  double n = vy.ivalue*0.010000;
		  double q = va.ivalue*0.010000;
    //ROS_INFO_STREAM("Reading from serial port");
	//ROS_INFO("I heard 1.2 velocity: mx-[%f],n-[%f],",m,n);
	//ROS_INFO("I heard 3 velocity: [%f]",q);
	//ROS_INFO_STREAM("End reading from serial port");
		double M = 0.577350*(m-n);
		double N = (m+n-2*q)*0.333333;
		double Q = (m+n+q)*1.666700;
	 ROS_INFO_STREAM("Reading from serial port");
	ROS_INFO("I heard 1.2 velocity: M-[%f],N-[%f],",M,N);
	ROS_INFO("I heard 3 velocity: [%f]",Q);
	ROS_INFO_STREAM("End reading from serial port");
        double dt = (current_time - last_time).toSec();
        double delta_x = (M * cos(th) - N * sin(th)) * dt;
        double delta_y = (M * sin(th) + N * cos(th)) * dt;
        double delta_th = Q * dt;

        x += delta_x;
        y += delta_y;
        th += delta_th;
		//ROS_INFO_STREAM("Reading from serial port");
		//ROS_INFO("I heard linear velocity: x-[%f],y-[%f],",x,y);
		//ROS_INFO("I heard angular velocity: [%f]",th);
		//ROS_INFO_STREAM("End reading from serial port");
		std::cout << "Twist Received" << std::endl;	
				//将偏航角转换成四元数才能发布
				odom_quat = tf::createQuaternionMsgFromYaw(th);
				
				//发布坐标变换父子坐标系
				odom_trans.header.frame_id = "odom";
				odom_trans.child_frame_id = "base_link";
				//填充获取的数据
				odom_trans.transform.translation.x = x;//posx.fvalue;//x坐标
				odom_trans.transform.translation.y = y;//posy.fvalue;//y坐标
				odom_trans.transform.translation.z = 0;//z坐标				
				odom_trans.transform.rotation = odom_quat;//偏航角
				//发布tf坐标变换
				odom_broadcaster.sendTransform(odom_trans);
				//获取当前时间
				current_time = ros::Time::now();
				//载入里程计时间戳
				odom.header.stamp = current_time;
				//里程计父子坐标系
				odom.header.frame_id = "odom";
				odom.child_frame_id = "base_link";
				//里程计位置数据
				odom.pose.pose.position.x = x;//posx.fvalue;
				odom.pose.pose.position.y = y;//posy.fvalue;
				odom.pose.pose.position.z = 0;
				odom.pose.pose.orientation = odom_quat;
				//载入线速度和角速度
				odom.twist.twist.linear.x = m;//vx.fvalue;
				odom.twist.twist.linear.y = n;//vy.fvalue;
				odom.twist.twist.angular.z =q;// va.fvalue;
				//发布里程计消息
				read_pub.publish(odom);
				ROS_INFO("publish odometry");
				last_time = current_time;				
			}
			memset(r_buffer,0,rBUFFERSIZE);
        }
        loop_rate.sleep();

    }
}

