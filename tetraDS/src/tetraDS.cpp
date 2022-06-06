#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>

//Custom Service//
#include <tetraDS/parameter_read.h>
#include <tetraDS/parameter_write.h>
//Move Mode service//
#include <tetraDS/set_move_mode.h>
#include <tetraDS/linear_position_move.h>
#include <tetraDS/angular_position_move.h>

extern "C"
{
	#include "drive_module.h"
	#include "dssp_rs232_drive_module.h"
}

#include <pthread.h>
#include <time.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
using namespace std;

#define WHEEL_RADIUS 0.1015 //m
#define WHEEL_DISTANCE 0.438 //m
#define TREAD_WIDTH 0.04 //m

//serial
int com_port = 0;
char port[16] = {0,};
//cmd_vel
double input_linear=0.0, input_angular = 0.0;
double control_linear=0.0, control_angular = 0.0;
double linear=0.0, angular = 0.0, bt_linear = 0.0, bt_angular = 0.0;
int m_old_accel_data = 50;
//Pose
double m_dX_pos = 0.0;
double m_dY_pos = 0.0;
double m_dTheta = 0.0;
//bumper & emg
int m_bumper_data = 0;
int m_emg_state = 0;
//Position move
int m_iPOS_Y = 0;
int m_iPOS_Theta = 0;
bool bPosition_mode_flag = false;
//emg one time check flag
bool m_bCheck_emg = true;
//tf_prefix add
std::string tf_prefix_;
bool has_prefix;
//Joystick value
int joy_linear = 1.0;
int joy_angular = 1.0;
//Parameter data
int m_iParam = 0;
//tetra parameter read & write service
ros::ServiceServer parameter_read_service;
tetraDS::parameter_read param_read_cmd;
ros::ServiceServer parameter_write_service;
tetraDS::parameter_write param_write_cmd;
//tetra Position Move service
ros::ServiceServer mode_change_service;
tetraDS::set_move_mode mode_change_cmd;
ros::ServiceServer linear_position_move_service;
tetraDS::linear_position_move linear_move_cmd;
ros::ServiceServer angular_position_move_service;
tetraDS::angular_position_move angular_move_cmd;

//ekf_localization
bool m_bEKF_option = false;


class TETRA
{
    private:

    public:
    bool first;
    double prev_coordinates[3];
    double coordinates[3]; //x,y,theta

    ros::NodeHandle pnh;
    ros::Publisher odom_publisher;
    tf::TransformBroadcaster odom_broadcaster;
    ros::Time current_time, last_time;

    TETRA()
	{
        first=true;
        odom_publisher = pnh.advertise<nav_msgs::Odometry>("odom", 50);
        printf("Create TETRA Class\n");

        current_time=ros::Time::now();
        last_time=ros::Time::now();
        for(int i=0;i<3;i++)
		{
            prev_coordinates[i] = 0;
            coordinates[i] = 0;
        }

    }
    ~TETRA()
	{
		printf("Distroy TETRA Class\n");
	}

		void read(){

				if(first) 
				{
					current_time=ros::Time::now();
					last_time=current_time;
					first=false;
				}
				else 
				{
					current_time=ros::Time::now();
					pub();
				}

		}

    static void velCallback(const geometry_msgs::Twist::ConstPtr& vel)
	{
		linear = vel->linear.x;
		angular = vel->angular.z;

    }

    void pub()
	{
        double dt=(current_time-last_time).toSec();
        double velocity[3];

        for(int i=0;i<3;i++)
		{
            velocity[i]=(coordinates[i]-prev_coordinates[i])/dt;
            prev_coordinates[i]=coordinates[i];
        }

		
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(coordinates[2]);
		
        geometry_msgs::TransformStamped odom_trans;
		if(!m_bEKF_option) //add...ekf_localization option _ wbjin
		{
			odom_trans.header.stamp = current_time;
			if(has_prefix)
			{
				odom_trans.header.frame_id = tf_prefix_ + "/odom"; //tf_prefix add
				odom_trans.child_frame_id = tf_prefix_ + "/base_footprint"; //tf_prefix add
			} 
			else 
			{
				odom_trans.header.frame_id = "odom";
				odom_trans.child_frame_id = "base_footprint";
			}
			odom_trans.transform.translation.x = coordinates[0];
			odom_trans.transform.translation.y = coordinates[1];
			odom_trans.transform.translation.z = 0.0;
			odom_trans.transform.rotation = odom_quat;
			odom_broadcaster.sendTransform(odom_trans);
		}
		
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
		if(has_prefix)
		{
			odom.header.frame_id = tf_prefix_ + "/odom"; //tf_prefix add
			odom.child_frame_id = tf_prefix_ + "/base_footprint"; //tf_prefix add
		} 
		else 
		{
			odom.header.frame_id = "odom";
			odom.child_frame_id = "base_footprint";
		}
        //pose
        odom.pose.pose.position.x= coordinates[0];
        odom.pose.pose.position.y= coordinates[1];
        odom.pose.pose.position.z= 0.0;
        odom.pose.pose.orientation= odom_quat;
        odom.twist.twist.linear.x = velocity[0];
        odom.twist.twist.linear.y = velocity[1];
        odom.twist.twist.linear.z = 0.0;
	odom.twist.twist.angular.z = velocity[2];
        odom_publisher.publish(odom);

        last_time=current_time;
    }
};

static void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	//Axis stick
	////Two Hand Joystick
	if(joy->axes[4])
	{
		bt_angular += joy_angular * joy->axes[4];
	}
	if(joy->axes[5])
	{
		bt_linear += joy_linear * joy->axes[5];
	}

	//Buttons
	if(joy->buttons[0])
	{
		joy_angular -= 1.0;
		if(joy_angular < 0) joy_angular = 0;
	}
	if(joy->buttons[1])
	{
		joy_linear -= 1.0;
		if(joy_linear < 0) joy_linear = 0;
	}
	if(joy->buttons[2])
	{
		joy_angular += 1.0;
	}
	if(joy->buttons[3])
	{
		joy_linear += 1.0;
	}
	if(joy->buttons[5])
	{
			bt_linear = 0;
			bt_angular = 0;
			joy_linear = 1.0;
			joy_angular = 1.0;
	}
	
	//Velocity Command//
	linear = (joy_linear * (double)joy->axes[1] + bt_linear) / 3.0;
	////Two Hand Joystick
	angular = ((double)joy_angular * (joy->axes[1] >= 0 ? joy->axes[2] : (joy->axes[2] * -1) ) + bt_angular) / 3.0;

}

void accelCallback(const std_msgs::Int32::ConstPtr& msg)
{
	int m_iAccel_data = msg->data;
	//Acceleration command
	if(m_old_accel_data != m_iAccel_data)
	{
		dssp_rs232_drv_module_set_parameter(6, m_iAccel_data);
		//update
		m_old_accel_data = m_iAccel_data;
	}

}

void vjoyCallback(const geometry_msgs::Twist::ConstPtr& vel)
{
	//Virtual joystick command//
	linear = vel->linear.x;
	angular = vel->angular.z;
}

int Reset = 0;
void PoseResetCallback(const std_msgs::Int32::ConstPtr& msg)
{
	TETRA tetra;
	Reset = msg->data;
	if(Reset == 1)
	{
		dssp_rs232_drv_module_reset_odometry();
		Reset = 0;
	}
}

void ServoONCallback(const std_msgs::Int32::ConstPtr& msg)
{
	int m_idata = msg->data;
	if(m_idata == 1)
	{
		bPosition_mode_flag = false;
		//Velocity Mode
		dssp_rs232_drv_module_set_velocitymode();
		usleep(10000);
		dssp_rs232_drv_module_set_servo(1);
		m_idata = 0;
	}
	else if(m_idata == 2)
	{
		dssp_rs232_drv_module_set_servo(0);
		m_idata = 0;
	}

	else if(m_idata == 3)
	{
		bPosition_mode_flag = true;
		//Velocity Mode
		dssp_rs232_drv_module_set_positionmode();
		usleep(10000);
		dssp_rs232_drv_module_set_servo(1);
		m_idata = 0;
	}
}

void Power_statusCallback(const std_msgs::Int32::ConstPtr& msg)
{
	int m_idata = msg->data;
	if(m_idata == -10)
	{
		dssp_rs232_drv_module_set_servo(0); //Servo Off
		printf("[Error]: Power Board Error !!! \n");
	}

}

bool Parameter_Read_Command(tetraDS::parameter_read::Request  &req, 
							tetraDS::parameter_read::Response &res)
{
	bool bResult = false;
    
	dssp_rs232_drv_module_read_parameter(req.num, &m_iParam);
    
	res.data = m_iParam;
	/*
	int32 num
	---
	int32 data
	bool command_Result
    */
    bResult = true;
	res.command_Result = bResult;
	return true;
}

bool Parameter_Write_Command(tetraDS::parameter_write::Request  &req, 
							 tetraDS::parameter_write::Response &res)
{
	bool bResult = false;
    
	dssp_rs232_drv_module_set_parameter(req.num, req.data);
	/*
	int32 num
	int32 data
	---
	bool command_Result
    */
    bResult = true;
	res.command_Result = bResult;
	return true;
}

bool Movemode_Change_Command(tetraDS::set_move_mode::Request  &req, 
							 tetraDS::set_move_mode::Response &res)
{
	bool bResult = false;
    
	if(req.mode == 1) //Position Mode
	{
		dssp_rs232_drv_module_set_servo(0);
		usleep(10000);
		dssp_rs232_drv_module_set_positionmode();
		usleep(10000);
		dssp_rs232_drv_module_set_servo(1);
		usleep(10000);
	}
	else //Velocity Mode
	{
		dssp_rs232_drv_module_set_servo(0);
		usleep(10000);
		dssp_rs232_drv_module_set_velocitymode();
		usleep(10000);
		dssp_rs232_drv_module_set_servo(1);
		usleep(10000);
	}
	/*
	int32 mode
	---
	bool command_Result
    */

    bResult = true;
	res.command_Result = bResult;
	return true;
}

bool Linear_Move_Command(tetraDS::linear_position_move::Request  &req, 
						 tetraDS::linear_position_move::Response &res)
{
	bool bResult = false;
    
	dssp_rs232_drv_module_set_position(0, 0, req.linear_position);
	/*
	int32 linear_position
	---
	bool command_Result
    */

    bResult = true;
	res.command_Result = bResult;
	return true;
}

bool Angular_Move_Command(tetraDS::angular_position_move::Request  &req, 
						  tetraDS::angular_position_move::Response &res)
{
	bool bResult = false;
    
	dssp_rs232_drv_module_set_position(2, 0, req.angular_degree);
	/*
	int32 angular_degree
	---
	bool command_Result
    */

    bResult = true;
	res.command_Result = bResult;
	return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double distance_to_wheelrotation(double wheel_diameter, double distance)
{
	return (distance / (M_PI * wheel_diameter));
}

double direction_to_diwheelrotation_diff(double wheel_distance, double wheel_diameter, double rad)
{
	return (wheel_distance / (wheel_diameter * M_PI)) * rad;
}

void speed_to_diwheel_rpm(double meter_per_sec, double rad_per_sec, double * left_rpm, double * right_rpm)
{
	double linearRotate = distance_to_wheelrotation(2.0*WHEEL_RADIUS, meter_per_sec);
	double angularRorate = direction_to_diwheelrotation_diff(WHEEL_DISTANCE/2.0, 2.0*WHEEL_RADIUS, rad_per_sec);

	 *left_rpm = (linearRotate - angularRorate) * 60.0;
	 *right_rpm = (linearRotate + angularRorate) * 60.0;

}

double RPM_to_ms(double d_rpm)
{
	return WHEEL_RADIUS*2.0*M_PI*(d_rpm/60.0);
}

void SetMoveCommand(double fLinear_vel, double fAngular_vel)
{
	double Left_Wheel_vel = 0.0;
	double Right_Wheel_vel = 0.0;
	speed_to_diwheel_rpm(fLinear_vel, fAngular_vel, &Left_Wheel_vel, &Right_Wheel_vel);

	//printf("Left_Wheel_vel: %f, Right_Wheel_vel: %f \n", Left_Wheel_vel, Right_Wheel_vel);

	int iData1 = 1000.0 * RPM_to_ms(Left_Wheel_vel);
	int iData2 = 1000.0 * RPM_to_ms(Right_Wheel_vel);
	dssp_rs232_drv_module_set_velocity(iData1, iData2);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Main Loop//
int main(int argc, char * argv[])
{
    ros::init(argc, argv, "tetraDS");
    ros::NodeHandle n;
	ros::NodeHandle njoy;
	ros::NodeHandle vjoy;
	ros::NodeHandle nReset;
	ros::NodeHandle nbumper;
	ros::NodeHandle nemg;
	ros::NodeHandle param;
    ros::Publisher tetra_battery_publisher;
	has_prefix=ros::param::get("tf_prefix", tf_prefix_); //tf_prefix add - 210701

	//Read Conveyor Option Param Read//
    n.getParam("ekf_option", m_bEKF_option);
    printf("##ekf_option: %d \n", m_bEKF_option);

    TETRA tetra;
    //cmd_velocity_velue//
	ros::Subscriber vel_sub = n.subscribe("cmd_vel",100,tetra.velCallback);
	//acceleration_velue//
	ros::Subscriber acc_sub = n.subscribe("accel_vel",10,accelCallback);
	//Joystick//
    ros::Subscriber joy_sub = njoy.subscribe<sensor_msgs::Joy>("joy", 10, joyCallback);
	ros::Subscriber vjoy_sub = vjoy.subscribe<geometry_msgs::Twist>("virtual_joystick/cmd_vel", 10, vjoyCallback);
	//Pose Reset//_test
	ros::Subscriber PoseReset = nReset.subscribe<std_msgs::Int32>("PoseRest",10, PoseResetCallback);
	//Servo ON && OFF//
	ros::Subscriber Servo_ON = nReset.subscribe<std_msgs::Int32>("Servo_ON",10, ServoONCallback);
	//Power Board Error Check//
	ros::Subscriber Power_status = nReset.subscribe<std_msgs::Int32>("power_status",1, Power_statusCallback);
	//Bumper 
	ros::Publisher bumper_publisher;
	bumper_publisher = nbumper.advertise<std_msgs::Int32>("bumper_data", 10);
	std_msgs::Int32 bumper_data;
	//EMG Switch
	ros::Publisher emg_publisher;
	emg_publisher = nemg.advertise<std_msgs::Int32>("emg_state", 10);
	std_msgs::Int32 emg_state;

	//tetraDS_service
	parameter_read_service  = param.advertiseService("param_read_cmd", Parameter_Read_Command);
	parameter_write_service = param.advertiseService("param_write_cmd", Parameter_Write_Command);
	mode_change_service = param.advertiseService("mode_change_cmd", Movemode_Change_Command);
	linear_position_move_service = param.advertiseService("linear_move_cmd", Linear_Move_Command);
	angular_position_move_service = param.advertiseService("angular_move_cmd", Angular_Move_Command);

    ros::Rate loop_rate(30.0); //default: 30HZ

	sprintf(port, "/dev/ttyS0");
	//RS232 Connect
	if(dssp_rs232_drv_module_create(port, 200) == 0)
	{
		printf("TETRA_DS4_rs232 Port Open Success\n");
	}
	else
	{
		printf("TETRA_DS4_rs232 Port Open Error!\n");
		return -1;
	}

	// Error Reset//
	dssp_rs232_drv_module_set_drive_err_reset();
	usleep(10000);
	//set acc slope time
	dssp_rs232_drv_module_set_parameter(6, 50); //accelation slop time 50mecs
	usleep(10000);
	//Velocity Mode
	dssp_rs232_drv_module_set_velocitymode();
	usleep(10000);
	//Servo On//
	dssp_rs232_drv_module_set_servo(1);
	usleep(10000);
	//Reset odometry
	dssp_rs232_drv_module_reset_odometry();
	usleep(10000);
	dssp_rs232_drv_module_set_charge(0);
	//emg flag//
	bool m_bflag_emg = false;
	//
	printf("□■■■■■■■□■■■■■■□□■■■■■■■□■■■■■■□□□□□□■□□□□\n");
	printf("□□□□■□□□□■□□□□□□□□□□■□□□□■□□□□□■□□□□□■□□□□\n");
	printf("□□□□■□□□□■□□□□□□□□□□■□□□□■□□□□□■□□□□■□■□□□\n");
	printf("□□□□■□□□□■■■■■■□□□□□■□□□□■■■■■■□□□□■□□□■□□\n");
	printf("□□□□■□□□□■□□□□□□□□□□■□□□□■□□□□□■□□□■■■■■□□\n");
	printf("□□□□■□□□□■□□□□□□□□□□■□□□□■□□□□□■□□■□□□□□■□\n");
	printf("□□□□■□□□□■■■■■■□□□□□■□□□□■□□□□□■□□■□□□□□■□\n");

    while(ros::ok())
	{
        ros::spinOnce();
		
		input_linear  = linear;
		input_angular = angular;

		//smoother velocity Loop//////////////////////////////////////////////////
		//linear_velocity
		if(input_linear > control_linear)
		{
			control_linear = min(input_linear, control_linear + 0.008);  //8mm++
		}
		else if(input_linear < control_linear)
		{
			control_linear = max(input_linear, control_linear - 0.025);  //25mm --
		}
		else
		{
			control_linear = input_linear;
		}
		// //angular_velocity
		// if(input_angular > control_angular)
		// {
		// 	control_angular = min(input_angular, control_angular + 0.05);
		// }
		// else if(input_angular < control_angular)
		// {
		// 	control_angular = max(input_angular, control_angular - 0.05); 
		// }
		// else
		// {
		// 	control_angular = input_angular;
		// }
		//////////////////////////////////////////////////////////////////////////
		
		//control_linear = input_linear;
		control_angular = input_angular;

		//EMG Check Loop
		emg_state.data = m_emg_state;
		emg_publisher.publish(emg_state);

		//Bumper Check Loop
		bumper_data.data = m_bumper_data;
		bumper_publisher.publish(bumper_data);

		//odometry calback//
		dssp_rs232_drv_module_read_odometry(&m_dX_pos, &m_dY_pos, &m_dTheta);
		tetra.coordinates[0] = (m_dX_pos /1000.0);
		tetra.coordinates[1] = (m_dY_pos /1000.0);
		tetra.coordinates[2] = m_dTheta * (M_PI/1800.0);

		if(!bPosition_mode_flag) //Velocity mode only
		{
			SetMoveCommand(control_linear, control_angular);
			dssp_rs232_drv_module_read_bumper_emg(&m_bumper_data, &m_emg_state);
		}

		if(m_emg_state)
		{
			if(m_bflag_emg)
			{
				dssp_rs232_drv_module_set_servo(0); //Servo Off
				usleep(1000);
				dssp_rs232_drv_module_set_drive_err_reset();
				usleep(1000);
				m_bflag_emg = false;
			}
		}
		else
		{
			if(!m_bflag_emg)
			{
				dssp_rs232_drv_module_set_servo(1); //Servo On
				m_bflag_emg = true;
			}
		}

		tetra.read();
		loop_rate.sleep();
    }

	//Servo Off
	dssp_rs232_drv_module_set_servo(0);
	printf("Servo Off \n");
	usleep(10000);
	//RS232 Disconnect
	dssp_rs232_drv_module_destroy();
	printf("RS232 Disconnect \n");

    return 0;
}
