#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Range.h> //Ultrasonic//
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "tetraDS_interface/ledcontrol.h" //SRV
#include "tetraDS_interface/ledtogglecontrol.h" //SRV
#include "tetraDS_interface/toggleon.h" //SRV
#include "tetraDS_interface/Integrallog.h" //SRV
#include "tetraDS_interface/power_set_outport.h" //SRV 
#include "tetraDS_interface/GPIO.h" //MSG
#include "tetraDS_interface/loadcell_callibration.h" //SRV
#include "tetraDS_interface/conveyor_auto_movement.h" //SRV
#include "tetraDS_interface/conveyor_manual_movement.h" //SRV
#include "tetraDS_interface/power_get_io_status.h" //SRV

#include <tetraDS_interface/power_set_enable.h> //SRV
#include <tetraDS_interface/power_set_single_outport.h> //SRV
#include <tetraDS_interface/power_set_single_enable.h> //SRV
#include <tetraDS_interface/power_wheel_enable.h> //SRV
#include <tetraDS_interface/power_parameter_read.h> // SRV
#include <tetraDS_interface/conveyor_parameter_read.h> // SRV
#include <tetraDS_interface/power_parameter_write.h> // SRV
#include <tetraDS_interface/conveyor_parameter_write.h> // SRV
#include <tetraDS_interface/conveyor_data_read.h> // SRV
#include <tetraDS_interface/power_data_read.h> // SRV
#include <tetraDS_interface/power_version_read.h> // SRV
#include <tetraDS_interface/power_sonar_read.h> // SRV
#include <tetraDS_interface/power_adc_read.h> // SRV
#include <tetraDS_interface/power_sonar_cmd.h> //SRV



extern "C"
{
	#include "power_module.h"
	#include "dssp_rs232_power_module.h"
}

#include <thread>
#include <time.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
using namespace std;

#define Ultrasonic_MIN_range	0.04
#define Ultrasonic_MAX_range	0.5
#define BUF_LEN 4096

//Power parameter data
int m_iParam0 = 0;
int m_iParam1 = 0;
int m_iParam2 = 0;
int m_iParam3 = 0;
int m_iParam4 = 0;
int m_iParam5 = 0;
int m_iParam6 = 0;
int m_iParam7 = 0;
int m_iParam8 = 0;
int m_iParam9 = 0;
int m_iParam10 = 0;
int m_iParam11 = 0;
int m_iParam12 = 0;
int m_iParam13 = 0;
int m_iParam14 = 0;
int m_iParam15 = 0;
int m_iParam16 = 0;
int m_iParam17 = 0;
int m_iParam18 = 0;
int m_iParam19 = 0;
int m_iParam20 = 0;
int m_iParam21 = 0;
int m_iParam22 = 0;
int m_iParam23 = 0;
int m_iParam24 = 0;
int m_iParam25 = 0;
int m_iParam26 = 0;
int m_iParam27 = 0;
int m_iParam28 = 0;
int m_iParam29 = 0;
int m_iParam30 = 0;
int m_iParam31 = 0;

int m_iFlag_PowerCheck_cnt = 0;
int com_port = 0;
char port[16] = {0,};
//battery data
int m_ibattery_Level = 0;
double m_dbattery = 0.0;
double m_dVoltage = 0.0;
double m_dCurrent = 0.0;
int m_imode_status = 0;
int m_iPowerCheck = 0;
int m_iPowerCheckCount = 0;
//Ultrasonic data//
double m_dUltrasonic[8] = {0.0, };  //Max Ultrasonic: 8ea (TETRA-DS5 used 4ea)
sensor_msgs::Range range_msg1; //Ultrasonic_1
sensor_msgs::Range range_msg2; //Ultrasonic_2
sensor_msgs::Range range_msg3; //Ultrasonic_3
sensor_msgs::Range range_msg4; //Ultrasonic_4
double time_offset_in_seconds;
//GPIO data//
int m_iOutput[8] = {0,};
int m_iInput[8] = {0,};
//Conveyor loadcell & sensor status//
double m_dLoadcell_weight = 0.0;
int m_dConveyor_sensor = 0; 
int m_iConveyor_movement = 0;
bool m_bConveyor_option = true;

//ROS tetraDS_interface custom service
ros::ServiceServer chargeport_service_on, chargeport_service_off;
ros::ServiceServer led_service;
tetraDS_interface::ledcontrol led_cmd;
ros::ServiceServer ledtoggle_service;
tetraDS_interface::ledtogglecontrol ledtoggle_cmd;
ros::ServiceServer turnon_service;
tetraDS_interface::toggleon turnon_cmd;
ros::ServiceServer log_service;
tetraDS_interface::Integrallog log_cmd;
//GPIO_service
ros::ServiceServer power_outport_service;
tetraDS_interface::power_set_outport Power_outport_cmd;

ros::ServiceServer power_single_outport_service;
tetraDS_interface::power_set_single_outport Power_single_outport_cmd;

ros::ServiceServer power_get_io_service;
tetraDS_interface::power_get_io_status Power_io_status_cmd;

//GPIO msg
tetraDS_interface::GPIO gpio_msg;
ros::Publisher GPIO_pub;

ros::Publisher points_1;
ros::Publisher points_2;
ros::Publisher points_3;
ros::Publisher points_4;

//Conveyor Loadcell CAL
ros::ServiceServer loadcell_callibration_service;
tetraDS_interface::loadcell_callibration CAL_cmd;
//Conveyor Auto Movement
ros::ServiceServer conveyor_auto_movement_service;
tetraDS_interface::conveyor_auto_movement Auto_Move_cmd;
//Conveyor Manual Movement
ros::ServiceServer conveyor_manual_movement_service;
tetraDS_interface::conveyor_manual_movement Manual_Move_cmd;

//POWER Enalbe service
ros::ServiceServer power_enable_service;
tetraDS_interface::power_set_enable Power_enable_cmd; 
ros::ServiceServer power_single_enable_service;
tetraDS_interface::power_set_single_enable Power_single_enable_cmd;
ros::ServiceServer power_wheel_enable_service;
tetraDS_interface::power_wheel_enable Power_wheel_enable_cmd;
//PowerSensor SONAR START
ros::ServiceServer power_sonar_cmd_service;
tetraDS_interface::power_sonar_cmd Power_sonar_start_cmd;

//tetra parameter read & write service
ros::ServiceServer power_parameter_read_service;
tetraDS_interface::power_parameter_read Power_parameter_read_cmd;
ros::ServiceServer conveyor_parameter_read_service;
tetraDS_interface::conveyor_parameter_read Conveyor_parameter_read_cmd;
ros::ServiceServer power_parameter_write_service;
tetraDS_interface::power_parameter_write Power_parameter_write_cmd;
ros::ServiceServer conveyor_parameter_write_service;
tetraDS_interface::conveyor_parameter_write Conveyor_parameter_write_cmd;

//tetra data read service
ros::ServiceServer conveyor_data_read_service;
tetraDS_interface::conveyor_data_read Conveyor_data_read_cmd;
ros::ServiceServer power_data_read_service;
tetraDS_interface::power_data_read Power_data_read_cmd;
ros::ServiceServer power_version_read_service;
tetraDS_interface::power_version_read Power_version_read_cmd;
ros::ServiceServer power_sonar_read_service;
tetraDS_interface::power_sonar_read Power_sonar_read_cmd;
ros::ServiceServer power_adc_read_service;
tetraDS_interface::power_adc_read Power_adc_read_cmd;


//File read & write
FILE *fp;
int status;
char Textbuffer[BUF_LEN];
char strPath[BUF_LEN];

//tf_prefix add
std::string tf_prefix_;
bool has_prefix;

bool Log_Command(tetraDS_interface::Integrallog::Request  &req, 
		tetraDS_interface::Integrallog::Response &res)
{
	bool bResult = false;

	time_t timer; 
	struct tm* t; 
	timer = time(NULL); 
	t = localtime(&timer); 

	int m_iVlotage_data[500] = {0, };
	int m_iCurrent_data[500] = {0, };
	string m_strFilePathName;
	string m_colon = ":";
	string m_time_stemp = to_string(t->tm_hour) + m_colon + to_string(t->tm_min) + m_colon + to_string(t->tm_sec);
	string m_temp1 = m_time_stemp + "_V_log";
	string m_temp2 = m_time_stemp + "_I_log";
    
	if(req.value_index == 1)
	{
		m_strFilePathName = "/home/tetra/LOG/" + m_temp1 + ".txt";  
		dssp_rs232_power_module_read_Voltage(m_iVlotage_data); 

		fp = fopen(m_strFilePathName.c_str(), "w");
		if(fp == NULL) 
			ROS_INFO("file is null");

		for(int i=0; i<500; i++)
		{
			fprintf(fp, "%.2f \n", (double)m_iVlotage_data[i] / 10.0);
		}
		fclose(fp);
	}
	else if(req.value_index == 2)
	{
		m_strFilePathName = "/home/tetra/LOG/" + m_temp2 + ".txt";  
		dssp_rs232_power_module_read_Current(m_iCurrent_data);

		fp = fopen(m_strFilePathName.c_str(), "w");
		if(fp == NULL) 
			ROS_INFO("file is null");

		for(int j=0; j<500; j++)
		{
			fprintf(fp, "%.2f \n", (double)m_iCurrent_data[j] / 10.0);
		}
		fclose(fp);
	}
	
    /*
	int32 value_index
	---
	bool command_Result
    */
    bResult = true;
	res.command_Result = bResult;
	return true;
}

void Error_Log_write(string log_data)
{
	time_t timer; 
	struct tm* t; 
	timer = time(NULL); 
	t = localtime(&timer); 

	string m_strFilePathName;
	string m_colon = ":";
	string m_time_stemp = to_string(t->tm_hour) + m_colon + to_string(t->tm_min) + m_colon + to_string(t->tm_sec);
	string m_temp1 = m_time_stemp + "_error";		

	m_strFilePathName = "/home/tetra/LOG/" + m_temp1 + ".txt";  
	fp = fopen(m_strFilePathName.c_str(), "w");
	if(fp == NULL) 
		ROS_INFO("file is null");


	fprintf(fp, "[Error]: %s \n", log_data.c_str());
	fclose(fp);
}

bool LEDcontrol_Command(tetraDS_interface::ledcontrol::Request  &req, 
			tetraDS_interface::ledcontrol::Response &res)
{
	bool bResult = false;
    
	dssp_rs232_power_module_set_light(req.ID, req.led_brightness);
    /*
    int32 ID
	int32 led_brightness
    ---
    bool command_Result
    */
    bResult = true;
	res.command_Result = bResult;
	return true;
}

bool LEDtoggle_Command(tetraDS_interface::ledtogglecontrol::Request  &req, 
			tetraDS_interface::ledtogglecontrol::Response &res)
{
	bool bResult = false;

	dssp_rs232_power_module_set_light_toggle(req.de_index, req.light_accel, 
						req.led_High_brightness, req.light_decel, req.led_Low_brightness);
													 
	/*
	int32 de_index
    int32 light_accel
    int32 led_High_brightness
    int32 light_decel
    int32 led_Low_brightness
    ---
    bool command_Result
    */
	bResult = true;
	res.command_Result = bResult;
	return true;
}

bool TurnOn_Command(tetraDS_interface::toggleon::Request  &req, 
			tetraDS_interface::toggleon::Response &res)
{
	bool bResult = false;
    
	dssp_rs232_power_module_toggle_on(req.ID);
    /*
    int32 ID
    ---
    bool command_Result
    */
    	bResult = true;
	res.command_Result = bResult;
	return true;
}

bool ChargingPortOn(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res )
{
	dssp_rs232_power_module_set_charging_ready(1);
	return true;
}

bool ChargingPortOff(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res )
{
	dssp_rs232_power_module_set_charging_ready(0);
	return true;
}

bool OutportOnOff(tetraDS_interface::power_set_outport::Request  &req, 
			tetraDS_interface::power_set_outport::Response &res)
{
	bool bResult = false;
    
	dssp_rs232_power_module_set_OutputPort(req.Output0,req.Output1,req.Output2,req.Output3,req.Output4,req.Output5,req.Output6,req.Output7);
    /*
    int8 Output0
	int8 Output1
	int8 Output2
	int8 Output3
	int8 Output4
	int8 Output5
	int8 Output6
	int8 Output7
	---
	bool command_Result
    */
    bResult = true;
	res.command_Result = bResult;
	return true;
}

bool SingleOutportOnOff(tetraDS_interface::power_set_single_outport::Request  &req, 
			tetraDS_interface::power_set_single_outport::Response &res)
{
	bool bResult = false;
    
	dssp_rs232_power_module_set_Single_OutputPort(req.ID,req.VALUE);
   
    bResult = true;
	res.command_Result = bResult;
	return true;
}

bool Power_get_IO_Status(tetraDS_interface::power_get_io_status::Request  &req, 
			 tetraDS_interface::power_get_io_status::Response &res)
{
	bool bResult = false;
    
	//Input data
	res.Input0 = gpio_msg.Input0;
	res.Input1 = gpio_msg.Input1;
	res.Input2 = gpio_msg.Input2;
	res.Input3 = gpio_msg.Input3;
	res.Input4 = gpio_msg.Input4;
	res.Input5 = gpio_msg.Input5;
	res.Input6 = gpio_msg.Input6;
	res.Input7 = gpio_msg.Input7;
	//Output data
	res.Output0 = gpio_msg.Output0;
	res.Output1 = gpio_msg.Output1;
	res.Output2 = gpio_msg.Output2;
	res.Output3 = gpio_msg.Output3;
	res.Output4 = gpio_msg.Output4;
	res.Output5 = gpio_msg.Output5;
	res.Output6 = gpio_msg.Output6;
	res.Output7 = gpio_msg.Output7;
	
    /*
	uint8  Input0
	uint8  Input1
	uint8  Input2
	uint8  Input3
	uint8  Input4
	uint8  Input5
	uint8  Input6
	uint8  Input7
	uint8  Output0
	uint8  Output1
	uint8  Output2
	uint8  Output3
	uint8  Output4
	uint8  Output5
	uint8  Output6
	uint8  Output7
	bool   command_Result
    */

	bResult = true;
	res.command_Result = bResult;
	return true;
}

bool Loadcell_Callibration_Command(tetraDS_interface::loadcell_callibration::Request  &req, 
					tetraDS_interface::loadcell_callibration::Response &res)
{
	bool bResult = false;

	dssp_rs232_power_module_loadcell_callibration();
													 
	/*
    ---
    bool command_Result
    */
	bResult = true;
	res.command_Result = bResult;
	return true;
}

bool Conveyor_Auto_Move_Command(tetraDS_interface::conveyor_auto_movement::Request  &req, 
					tetraDS_interface::conveyor_auto_movement::Response &res)
{
	bool bResult = false;
    
	dssp_rs232_power_module_conveyor_movement(req.start);
    /*
    int32 start
    ---
    bool command_Result
    */
	bResult = true;
	res.command_Result = bResult;
	return true;
}

bool Conveyor_Manual_Move_Command(tetraDS_interface::conveyor_manual_movement::Request  &req, 
					tetraDS_interface::conveyor_manual_movement::Response &res)
{
	bool bResult = false;
    
	dssp_rs232_power_module_conveyor_manual_movement(req.mode);
    /*
    int32 mode
    ---
    bool command_Result
    */
	bResult = true;
	res.command_Result = bResult;
	return true;
}

void RangeToCloud_D_L(const sensor_msgs::Range::ConstPtr& range_msg)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	cloud->header.stamp = pcl_conversions::toPCL(range_msg->header).stamp;
	if(has_prefix)
	{
		cloud->header.frame_id = tf_prefix_ + "/sonar_DL";
	}
	else
	{
		cloud->header.frame_id = "sonar_DL";
	}

	cloud->height = 1;

	if (range_msg->range < std::numeric_limits<float>::infinity())
	{
		tf::Point pt(range_msg->range, 0.0, 0.0);
		pcl::PointXYZ pcl_point;
		pcl_point.x = pt.m_floats[0];
		pcl_point.y = pt.m_floats[1];
		pcl_point.z = pt.m_floats[2];
		cloud->points.push_back(pcl_point);
		++cloud->width;
		points_1.publish(cloud);
	}
}

void RangeToCloud_R_L(const sensor_msgs::Range::ConstPtr& range_msg)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	cloud->header.stamp = pcl_conversions::toPCL(range_msg->header).stamp;
	if(has_prefix)
	{
		cloud->header.frame_id = tf_prefix_ + "/sonar_RL";
	}
	else
	{
		cloud->header.frame_id = "sonar_RL";
	}
	
	cloud->height = 1;

	if (range_msg->range < std::numeric_limits<float>::infinity())
	{
		tf::Point pt(range_msg->range, 0.0, 0.0);
		pcl::PointXYZ pcl_point;
		pcl_point.x = pt.m_floats[0];
		pcl_point.y = pt.m_floats[1];
		pcl_point.z = pt.m_floats[2];
		cloud->points.push_back(pcl_point);
		++cloud->width;
		points_2.publish(cloud);
	}
}

void RangeToCloud_R_R(const sensor_msgs::Range::ConstPtr& range_msg)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	cloud->header.stamp = pcl_conversions::toPCL(range_msg->header).stamp;
	if(has_prefix)
	{
		cloud->header.frame_id = tf_prefix_ + "/sonar_RR";
	}
	else
	{
		cloud->header.frame_id = "sonar_RR";
	}
	cloud->height = 1;

	if (range_msg->range < std::numeric_limits<float>::infinity())
	{
		tf::Point pt(range_msg->range, 0.0, 0.0);
		pcl::PointXYZ pcl_point;
		pcl_point.x = pt.m_floats[0];
		pcl_point.y = pt.m_floats[1];
		pcl_point.z = pt.m_floats[2];
		cloud->points.push_back(pcl_point);
		++cloud->width;
		points_3.publish(cloud);
	}
}

void RangeToCloud_D_R(const sensor_msgs::Range::ConstPtr& range_msg)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	cloud->header.stamp = pcl_conversions::toPCL(range_msg->header).stamp;
	if(has_prefix)
	{
		cloud->header.frame_id = tf_prefix_ + "/sonar_DR";
	}
	else
	{
		cloud->header.frame_id = "sonar_DR";
	}
	cloud->height = 1;

	if (range_msg->range < std::numeric_limits<float>::infinity())
	{
		tf::Point pt(range_msg->range, 0.0, 0.0);
		pcl::PointXYZ pcl_point;
		pcl_point.x = pt.m_floats[0];
		pcl_point.y = pt.m_floats[1];
		pcl_point.z = pt.m_floats[2];
		cloud->points.push_back(pcl_point);
		++cloud->width;
		points_4.publish(cloud);
	}
}

bool PowerEnableOnOff(tetraDS_interface::power_set_enable::Request  &req, 
			tetraDS_interface::power_set_enable::Response &res)
{
	bool bResult = false;
    
	dssp_rs232_power_module_set_Enable(req.POWER0,req.POWER1,req.POWER2,req.POWER3,req.POWER4,req.POWER5,req.POWER6,req.POWER7);
	
    /*
    int8 POWER0
	int8 POWER1
	int8 POWER2
	int8 POWER3
	int8 POWER4
	int8 POWER5
	int8 POWER6
	int8 POWER7
	---
	bool command_Result
    */
	bResult = true;
	res.command_Result = bResult;
	return true;
}

bool PowerSingleEnableOnOff(tetraDS_interface::power_set_single_enable::Request  &req, 
				 tetraDS_interface::power_set_single_enable::Response &res)
{
	bool bResult = false;
    
	dssp_rs232_power_module_set_Single_Enable(req.ID,req.VALUE);
	   
    bResult = true;
	res.command_Result = bResult;
	return true;
}

bool PowerWheelEnableOnOff(tetraDS_interface::power_wheel_enable::Request  &req, 
				tetraDS_interface::power_wheel_enable::Response &res)
{
	bool bResult = false;
    
	dssp_rs232_power_module_wheel_enable(req.on);
    /*
    int32 start
    ---
    bool command_Result
    */
	bResult = true;
	res.command_Result = bResult;
	return true;
}

bool Power_parameter_Read_Command(tetraDS_interface::power_parameter_read::Request  &req, 
					tetraDS_interface::power_parameter_read::Response &res)
{
	bool bResult = false;
    
	dssp_rs232_power_module_parameter_read(&m_iParam0,&m_iParam1,&m_iParam2,&m_iParam3,&m_iParam4,&m_iParam5,&m_iParam6,&m_iParam7,&m_iParam8,&m_iParam9,&m_iParam10,&m_iParam11,&m_iParam12,&m_iParam13,&m_iParam14,&m_iParam15,&m_iParam16,&m_iParam17,&m_iParam18,&m_iParam19,&m_iParam20,&m_iParam21,&m_iParam22,&m_iParam23,&m_iParam24,&m_iParam25,&m_iParam26,&m_iParam27,&m_iParam28,&m_iParam29,&m_iParam30,&m_iParam31);
	res.P0_UART1_BAUDRATE = m_iParam0;
	res.P1_UART2_BAUDRATE = m_iParam1;
	res.P2_SONAR_SELECT = m_iParam2;
	res.P3_SONAR_DISTANCE_OFFSET = m_iParam3;
	res.P4_SONAR_QUANTITY = m_iParam4;
	res.P5_SONAR_MAX_DISTANCE = m_iParam5;
	res.P6_NC = m_iParam6;
	res.P7_SONAR_NONE_DETECT_MODE = m_iParam7;
	res.P8_BATTERY_CHARGING_OFFSET = m_iParam8;
	res.P9_BATTERY_CHARGING_GAIN = m_iParam9;
	res.P10_STATUS_LED_MIN = m_iParam10;
	res.P11_STATUS_LED_MAX = m_iParam11;
	res.P12_BATTERY_LED_MIN = m_iParam12;
	res.P13_BATTERY_LED_MAX = m_iParam13;
	res.P14_LED_CH1_OFFSET = m_iParam14;
	res.P15_LED_CH2_OFFSET = m_iParam15;
	res.P16_LED_CH3_OFFSET = m_iParam16;
	res.P17_LED_CH4_OFFSET = m_iParam17;
	res.P18_LED_CH5_OFFSET = m_iParam18;
	res.P19_LED_CH6_OFFSET = m_iParam19;
	res.P20_LED_CH7_OFFSET = m_iParam20;
	res.P21_LED_CH8_OFFSET = m_iParam21;
	res.P22_POWER_ENABLE = m_iParam22;
	res.P23_OUTPORT = m_iParam23;
	res.P24_BATTERY_RECHARGE_VOLTAGE = m_iParam24;
	res.P25_BATTERY_RECHARGE_OFFSET = m_iParam25;
	res.P26_BATTERY_MIN = m_iParam26;
	res.P27_BATTERY_MAX = m_iParam27;
	res.P28_BATTERY_SAMPLING_TIME = m_iParam28;
	res.P29_CONVEYOR_MODE = m_iParam29;
	res.P30_CONVEYOR_TERMINAL_BASE = m_iParam30;
	res.P31_CONVEYOR_TERMINAL_OFFSET = m_iParam31;
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

bool Conveyor_parameter_Read_Command(tetraDS_interface::conveyor_parameter_read::Request  &req, 
					tetraDS_interface::conveyor_parameter_read::Response &res)
{
	bool bResult = false;
    
	dssp_rs232_conveyor_module_parameter_read(&m_iParam0,&m_iParam1,&m_iParam2,&m_iParam3,&m_iParam4,&m_iParam5,&m_iParam6,&m_iParam7,&m_iParam8,&m_iParam9,&m_iParam10,&m_iParam11,&m_iParam12,&m_iParam13,&m_iParam14,&m_iParam15,&m_iParam16,&m_iParam17,&m_iParam18,&m_iParam19,&m_iParam20);
	res.P0_UART1_BAUDRATE = m_iParam0;
	res.P1_UART2_BAUDRATE = m_iParam1;
	res.P2_MODE_SELECT = m_iParam2;
	res.P3_OUTPORT = m_iParam3;
	res.P4_LOADCELL_ZERO = m_iParam4;
	res.P5_LOADCELL_MIN = m_iParam5;
	res.P6_LOADCELL_GAIN = m_iParam6;
	res.P7_CONVEYOR_DIR = m_iParam7;
	res.P8_CONVEYOR_TIMEOUT = m_iParam8;
	res.P9_FIX_LOADING_START_DELAY = m_iParam9;
	res.P10_FIX_LOADING_END_DELAY = m_iParam10;
	res.P11_FIX_UNLOADING_START_DELAY = m_iParam11;
	res.P12_FIX_UNLOADING_MOTOR_DELAY = m_iParam12;
	res.P13_FIX_UNLOADING_END_DELAY = m_iParam13;
	res.P14_LOADING_START_DELAY = m_iParam14;
	res.P15_LOADING_END_DELAY = m_iParam15;
	res.P16_UNLOADING_START_DELAY = m_iParam16;
	res.P17_UNLOADING_END_DELAY = m_iParam17;
	res.P18_EXPRESS_QUANTITY = m_iParam18;
	res.P19_TERMINAL_LOADING_BASE = m_iParam19;
	res.P20_TERMINAL_UNLOADING_BASE = m_iParam20;
	
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

bool Conveyor_data_Read_Command(tetraDS_interface::conveyor_data_read::Request  &req, 
				tetraDS_interface::conveyor_data_read::Response &res)
{
	bool bResult = false;
    
	dssp_rs232_conveyor_module_data_read(&m_iParam0,&m_iParam1,&m_iParam2,&m_iParam3,&m_iParam4,&m_iParam5,&m_iParam6,&m_iParam7,&m_iParam8,&m_iParam9,&m_iParam10,&m_iParam11,&m_iParam12,&m_iParam13);
	res.D0_DOCKING_SIGNAL0 = m_iParam0;
	res.D1_DOCKING_SIGNAL1 = m_iParam1;
	res.D2_CHARGE_CURRENT = m_iParam2;
	res.D3_CHARGE_SIGNAL = m_iParam3;
	res.D4_CHARGE_VOLTAGE = m_iParam4;
	res.D5_LOADCELL_LOW = m_iParam5;
	res.D6_LOADCELL_SEN = m_iParam6;
	res.D7_AD_IN1 = m_iParam7;
	res.D8_AD_IN2 = m_iParam8;
	res.D9_AD_IN3 = m_iParam9;
	res.D10_IN_STATUS = m_iParam10;
	res.D11_OUT_STATUS = m_iParam11;
	res.D12_PHOTO_STATUS = m_iParam12;
	res.D13_CONVEYOR_STATUS = m_iParam13;
		
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

bool Power_data_Read_Command(tetraDS_interface::power_data_read::Request  &req, 
				tetraDS_interface::power_data_read::Response &res)
{
	bool bResult = false;
    
	dssp_rs232_power_module_data_read(&m_iParam0,&m_iParam1,&m_iParam2,&m_iParam3,&m_iParam4,&m_iParam5,&m_iParam6,&m_iParam7,&m_iParam8,&m_iParam9,&m_iParam10,&m_iParam11);
	res.D0_BATTERY_VOLTAGE = m_iParam0;
	res.D1_SYSTEM_CURRENT = m_iParam1;
	res.D2_CHARGE_CURRENT = m_iParam2;
	res.D3_CHARGE_SIGNAL = m_iParam3;
	res.D4_INPORT_STATUS = m_iParam4;
	res.D5_OUTPORT_STATUS = m_iParam5;
	res.D6_POWER_STATUS = m_iParam6;
	res.D7_CHARGER_TERMINAL_STATUS = m_iParam7;
	res.D8_TEMPERATURE0 = m_iParam8;
	res.D9_TEMPERATURE1 = m_iParam9;
	res.D10_MOBD_INPORT_STATUS = m_iParam10;
	res.D11_MOBD_OUTPORT_STATUS = m_iParam11;		
	
    bResult = true;
	res.command_Result = bResult;
	return true;
}

bool Power_version_Read_Command(tetraDS_interface::power_version_read::Request  &req, 
				tetraDS_interface::power_version_read::Response &res)
{
	bool bResult = false;
    
	dssp_rs232_power_module_version_read(&m_iParam0,&m_iParam1,&m_iParam2,&m_iParam3,&m_iParam4,&m_iParam5,&m_iParam6,&m_iParam7);
	res.D0_BOARD_TYPE = m_iParam0;
	res.D1_YEAR2 = m_iParam1;
	res.D2_YEAR1 = m_iParam2;
	res.D3_MONTH2 = m_iParam3;
	res.D4_MONTH1 = m_iParam4;
	res.D5_DAY2 = m_iParam5;
	res.D6_DAY1 = m_iParam6;
	res.D7_VER = m_iParam7;

   	bResult = true;
	res.command_Result = bResult;
	return true;
}

bool Power_sonar_Read_Command(tetraDS_interface::power_sonar_read::Request  &req, 
				tetraDS_interface::power_sonar_read::Response &res)
{
	bool bResult = false;
    
	dssp_rs232_power_module_sonar_read(&m_iParam0,&m_iParam1,&m_iParam2,&m_iParam3,&m_iParam4,&m_iParam5);
	res.D0_SONAR0 = m_iParam0;
	res.D1_SONAR1 = m_iParam1;
	res.D2_SONAR2 = m_iParam2;
	res.D3_SONAR3 = m_iParam3;
	res.D4_SONAR4 = m_iParam4;
	res.D5_SONAR5 = m_iParam5;
	
   	bResult = true;
	res.command_Result = bResult;
	return true;
}

bool Power_adc_Read_Command(tetraDS_interface::power_adc_read::Request  &req, 
				tetraDS_interface::power_adc_read::Response &res)
{
	bool bResult = false;
    
	dssp_rs232_power_module_adc_read(&m_iParam0,&m_iParam1,&m_iParam2,&m_iParam3,&m_iParam4,&m_iParam5,&m_iParam6,&m_iParam7);
	res.D0_ADC0 = m_iParam0;
	res.D1_ADC1 = m_iParam1;
	res.D2_ADC2 = m_iParam2;
	res.D3_ADC3 = m_iParam3;
	res.D4_ADC4 = m_iParam4;
	res.D5_ADC5 = m_iParam5;
	res.D6_ADC6 = m_iParam6;
	res.D7_ADC7 = m_iParam7;
				
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

bool Power_parameter_Write_Command(tetraDS_interface::power_parameter_write::Request  &req, 
					tetraDS_interface::power_parameter_write::Response &res)
{
	bool bResult = false;
    
	dssp_rs232_power_module_parameter_write(req.num, req.data);
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

bool Conveyor_parameter_Write_Command(tetraDS_interface::conveyor_parameter_write::Request  &req, 
					tetraDS_interface::conveyor_parameter_write::Response &res)
{
	bool bResult = false;
    
	dssp_rs232_conveyor_module_parameter_write(req.num, req.data);
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



bool Power_sonar_start_Command(tetraDS_interface::power_sonar_cmd::Request  &req, 
				tetraDS_interface::power_sonar_cmd::Response &res)
{
	bool bResult = false;
	dssp_rs232_power_module_set_Ultrasonic(req.start);
	
    /*
    int32 start
    ---
    bool command_Result
    */
	bResult = true;
	res.command_Result = bResult;
	return true;
}


int limit_time = 0;
int main(int argc, char * argv[])
{
    ros::init(argc, argv, "tetraDS_interface");
    ros::NodeHandle n;
    ros::Publisher tetra_battery_publisher;
	ros::Publisher docking_status_publisher;
	ros::Publisher battery_voltage_publisher;
	ros::Publisher battery_current_publisher;
	ros::Publisher conveyor_loadcell_publisher; //conveyor loadcell
	ros::Publisher conveyor_sensor_publisher; //conveyor sensor *2ea
	ros::Publisher conveyor_movement_publisher; //conveyor movement
	ros::Publisher power_error_publisher;
	ros::Publisher servo_pub;
	std_msgs::Int32 servo;

	ros::NodeHandle private_node_handle("~");
    private_node_handle.param<double>("time_offset_in_seconds", time_offset_in_seconds, 0.0);
	has_prefix=ros::param::get("tf_prefix", tf_prefix_); //tf_prefix add

	//I/O Control Service
    ros::NodeHandle IOS_h;
    led_service = IOS_h.advertiseService("led_cmd", LEDcontrol_Command);
    ledtoggle_service = IOS_h.advertiseService("ledtoggle_cmd", LEDtoggle_Command);
	turnon_service = IOS_h.advertiseService("turnon_cmd", TurnOn_Command);
	// Charging Port Control Services
    chargeport_service_on  = IOS_h.advertiseService("charging_port_on", ChargingPortOn);
    chargeport_service_off = IOS_h.advertiseService("charging_port_off", ChargingPortOff);
	//GPIO_Output service
	power_outport_service = IOS_h.advertiseService("Power_outport_cmd", OutportOnOff);
	power_single_outport_service = IOS_h.advertiseService("Power_single_outport_cmd", SingleOutportOnOff);
	//GPIO Check service
	power_get_io_service = IOS_h.advertiseService("Power_io_status_cmd", Power_get_IO_Status);

	//Loadcell service
	ros::NodeHandle CONVEYOR_h;
	loadcell_callibration_service = CONVEYOR_h.advertiseService("CAL_cmd", Loadcell_Callibration_Command);
	//Conveyor Auto Movement Service
	conveyor_auto_movement_service = CONVEYOR_h.advertiseService("Auto_Move_cmd", Conveyor_Auto_Move_Command);
	//Conveyor Manual Movement Service
	conveyor_manual_movement_service = CONVEYOR_h.advertiseService("Manual_Move_cmd", Conveyor_Manual_Move_Command);

	//Integral Log Service
	ros::NodeHandle log_h;
	log_service = log_h.advertiseService("log_cmd", Log_Command);

	//Ultrasonic//
	ros::Publisher Ultrasonic1_pub = n.advertise<sensor_msgs::Range>("Ultrasonic_D_L", 10);
	ros::Publisher Ultrasonic2_pub = n.advertise<sensor_msgs::Range>("Ultrasonic_R_L", 10);
	ros::Publisher Ultrasonic3_pub = n.advertise<sensor_msgs::Range>("Ultrasonic_R_R", 10);
	ros::Publisher Ultrasonic4_pub = n.advertise<sensor_msgs::Range>("Ultrasonic_D_R", 10);
	
	// PowerSensor parameter read Services
	ros::NodeHandle param;
    power_parameter_read_service  = param.advertiseService("Power_parameter_read_cmd", Power_parameter_Read_Command);
	conveyor_parameter_read_service  = param.advertiseService("Conveyor_parameter_read_cmd", Conveyor_parameter_Read_Command);
	power_parameter_write_service  = param.advertiseService("Power_parameter_write_cmd", Power_parameter_Write_Command);
	conveyor_parameter_write_service  = param.advertiseService("Conveyor_parameter_write_cmd", Conveyor_parameter_Write_Command);
	//system data read
	ros::NodeHandle data;
    conveyor_data_read_service  = data.advertiseService("Conveyor_data_read_cmd", Conveyor_data_Read_Command);
	power_data_read_service  = data.advertiseService("Power_data_read_cmd", Power_data_Read_Command);
	power_adc_read_service  = data.advertiseService("Power_adc_read_cmd", Power_adc_Read_Command);
	power_version_read_service  = data.advertiseService("Power_version_read_cmd", Power_version_Read_Command);
	power_sonar_read_service  = data.advertiseService("Power_sonar_read_cmd", Power_sonar_Read_Command);

	//battery & status
	tetra_battery_publisher = n.advertise<std_msgs::Int32>("tetra_battery", 1);
	std_msgs::Int32 battery_level;
	docking_status_publisher = n.advertise<std_msgs::Int32>("docking_status", 1);
	std_msgs::Int32 docking_status;

	//PowerBoard Check
	power_error_publisher = n.advertise<std_msgs::Int32>("power_status", 1);
	std_msgs::Int32 power_status;

	//battery Voltage & Current
	battery_voltage_publisher = n.advertise<std_msgs::Float64>("battery_voltage", 1);
	std_msgs::Float64 battery_voltage;
	battery_current_publisher = n.advertise<std_msgs::Float64>("battery_current", 1);
	std_msgs::Float64 battery_current;
	//GPIO status
	GPIO_pub = n.advertise<tetraDS_interface::GPIO>("gpio_status", 10);

	//Conveyor Status//
	conveyor_loadcell_publisher = n.advertise<std_msgs::Float64>("conveyor_loadcell", 1);
	std_msgs::Float64 conveyor_loadcell;
	conveyor_sensor_publisher = n.advertise<std_msgs::Int32>("conveyor_sensor", 1);
	std_msgs::Int32 conveyor_sensor;
	conveyor_movement_publisher = n.advertise<std_msgs::Int32>("conveyor_movement", 1);
	std_msgs::Int32 conveyor_movement;

	//Servo On/Off publish
    	servo_pub = n.advertise<std_msgs::Int32>("Servo_ON",10);

	//POWER Enable service
	power_enable_service = IOS_h.advertiseService("Power_enable_cmd", PowerEnableOnOff);
	power_single_enable_service = IOS_h.advertiseService("Power_single_enable_cmd", PowerSingleEnableOnOff);
	power_wheel_enable_service = IOS_h.advertiseService("Power_wheel_enable_cmd", PowerWheelEnableOnOff);
	power_sonar_cmd_service = IOS_h.advertiseService("Power_sonar_start_cmd", Power_sonar_start_Command);

	//Read Conveyor Option Param Read//
	n.getParam("conveyor_option", m_bConveyor_option);
	printf("##conveyor_option: %d \n", m_bConveyor_option);

	//Ultrasonic Paramter Setting//////////////////////////////////
	char frameid1[] = "/Ultrasonic_Down_Left";
	range_msg1.header.frame_id = tf_prefix_ + frameid1;
	range_msg1.radiation_type = 0; //Ultrasonic
	range_msg1.field_of_view = (60.0/180.0) * M_PI; //
	range_msg1.min_range = Ultrasonic_MIN_range; 
	range_msg1.max_range = Ultrasonic_MAX_range; 

	char frameid2[] = "/Ultrasonic_Rear_Left";
	range_msg2.header.frame_id = tf_prefix_ + frameid2;
	range_msg2.radiation_type = 0; //Ultrasonic
	range_msg2.field_of_view = (60.0/180.0) * M_PI; //
	range_msg2.min_range = Ultrasonic_MIN_range;
	range_msg2.max_range = Ultrasonic_MAX_range;

	char frameid3[] = "/Ultrasonic_Rear_Right";
	range_msg3.header.frame_id = tf_prefix_ + frameid3;
	range_msg3.radiation_type = 0; //Ultrasonic
	range_msg3.field_of_view = (60.0/180.0) * M_PI; //
	range_msg3.min_range = Ultrasonic_MIN_range;
	range_msg3.max_range = Ultrasonic_MAX_range;

	char frameid4[] = "/Ultrasonic_Down_Right";
	range_msg4.header.frame_id = tf_prefix_ + frameid4;
	range_msg4.radiation_type = 0; //Ultrasonic
	range_msg4.field_of_view = (60.0/180.0) * M_PI; //
	range_msg4.min_range = Ultrasonic_MIN_range;
	range_msg4.max_range = Ultrasonic_MAX_range;

	////////////////////////////////////////////////////////////////////////////////////////////
	////sonar range to pointcloud//
	points_1 = n.advertise<sensor_msgs::PointCloud2>("range_points_DL", 10);
	points_2 = n.advertise<sensor_msgs::PointCloud2>("range_points_RL", 10);
	points_3 = n.advertise<sensor_msgs::PointCloud2>("range_points_RR", 10);
	points_4 = n.advertise<sensor_msgs::PointCloud2>("range_points_DR", 10);

	ros::Subscriber sonar0_sub = n.subscribe<sensor_msgs::Range>("Ultrasonic_D_L",10,RangeToCloud_D_L);
	ros::Subscriber sonar1_sub = n.subscribe<sensor_msgs::Range>("Ultrasonic_R_L",10,RangeToCloud_R_L);
	ros::Subscriber sonar2_sub = n.subscribe<sensor_msgs::Range>("Ultrasonic_R_R",10,RangeToCloud_R_R);
	ros::Subscriber sonar3_sub = n.subscribe<sensor_msgs::Range>("Ultrasonic_D_R",10,RangeToCloud_D_R);

    	ros::Rate loop_rate(30.0); //30Hz Loop
	sprintf(port, "/dev/ttyS1");
	
	//RS232 Connect
	if(dssp_rs232_power_module_create(port, 200) == 0)
	{
		printf("TETRA_Power_rs232 Port Open Success\n");
	}
	else
	{
		printf("TETRA_Power_rs232 Port Open Error!\n");
		return -1;
	}

	//Charge port Enable//
	dssp_rs232_power_module_set_charging_ready(1);
	//Ultrasonic On//
	// dssp_rs232_power_module_set_Ultrasonic(1);

    	while(ros::ok())
	{
        	ros::spinOnce();
		// calculate measurement time
		ros::Time measurement_time = ros::Time(0) + ros::Duration(time_offset_in_seconds);
	//	m_iPowerCheck = dssp_rs232_power_module_read_battery(&m_dbattery, &m_dVoltage, &m_dCurrent, &m_imode_status, m_iInput, m_iOutput);
		m_iPowerCheck = dssp_rs232_power_module_read_tetra(&m_dbattery, &m_dVoltage, &m_dCurrent, &m_imode_status, m_iInput, m_iOutput, m_dUltrasonic);

		if(m_iPowerCheck < 0)
		{
			printf("!!!! Power Board data read Error(m_iPowerCheck: %d) !!!! \n", m_iPowerCheck);
			if(m_iFlag_PowerCheck_cnt > 10)
			{
				//Error 
				printf("[Error]: Power Board Disconnect !!! \n");
				Error_Log_write("Power Board Disconnect !!!");
				//servo Off
				servo.data = 2;
				servo_pub.publish(servo);
				
			}
			else
			{
				m_iFlag_PowerCheck_cnt++;
			}
			loop_rate.sleep();
			continue;
		}
		else
		{
			m_iFlag_PowerCheck_cnt = 0;
		}
		//add...Power Board Check
		power_status.data = m_iPowerCheck;
		power_error_publisher.publish(power_status);
		m_ibattery_Level = m_dbattery;
		battery_level.data = m_ibattery_Level;
		tetra_battery_publisher.publish(battery_level);
		//battery Voltage
		battery_voltage.data = m_dVoltage;
		battery_voltage_publisher.publish(battery_voltage);
		//battery Current
		battery_current.data = m_dCurrent;
		battery_current_publisher.publish(battery_current);
		//docking_status_publisher
		docking_status.data = m_imode_status;
		docking_status_publisher.publish(docking_status);
				
		//GPIO_status////////////////////////////////////////////
		//Input data
		gpio_msg.Input0 = m_iInput[0];
		gpio_msg.Input1 = m_iInput[1];
		gpio_msg.Input2 = m_iInput[2];
		gpio_msg.Input3 = m_iInput[3];
		gpio_msg.Input4 = m_iInput[4];
		gpio_msg.Input5 = m_iInput[5];
		gpio_msg.Input6 = m_iInput[6];
		gpio_msg.Input7 = m_iInput[7];
		//Output data
		gpio_msg.Output0 = m_iOutput[0];
		gpio_msg.Output1 = m_iOutput[1];
		gpio_msg.Output2 = m_iOutput[2];
		gpio_msg.Output3 = m_iOutput[3];
		gpio_msg.Output4 = m_iOutput[4];
		gpio_msg.Output5 = m_iOutput[5];
		gpio_msg.Output6 = m_iOutput[6];
		gpio_msg.Output7 = m_iOutput[7];
		GPIO_pub.publish(gpio_msg);

		//m_dUltrasonic * 4ea
	//	dssp_rs232_power_module_read_Ultrasonic(m_dUltrasonic);

		if(m_dUltrasonic[0] == 0.0)
			range_msg1.range = Ultrasonic_MAX_range;
		else
			range_msg1.range = m_dUltrasonic[0];

		range_msg1.header.stamp = measurement_time;
		if(m_dUltrasonic[1] == 0.0)
			range_msg2.range = Ultrasonic_MAX_range;
		else
			range_msg2.range = m_dUltrasonic[1];

		range_msg2.header.stamp = measurement_time;
		if(m_dUltrasonic[2] == 0.0)
			range_msg3.range = Ultrasonic_MAX_range;
		else
			range_msg3.range = m_dUltrasonic[2];

		range_msg3.header.stamp = measurement_time;
		if(m_dUltrasonic[3] == 0.0)
			range_msg4.range = Ultrasonic_MAX_range;
		else
			range_msg4.range = m_dUltrasonic[3];
			
		range_msg4.header.stamp = measurement_time;
		//Ultrasonic Publish
		Ultrasonic1_pub.publish(range_msg1);
		Ultrasonic2_pub.publish(range_msg2);
		Ultrasonic3_pub.publish(range_msg3);
		Ultrasonic4_pub.publish(range_msg4);

		if(m_bConveyor_option)
		{
			//Conveyor Loadcell weight
			dssp_rs232_power_module_read_loadcell(&m_dLoadcell_weight);

			conveyor_loadcell.data = m_dLoadcell_weight;
			conveyor_loadcell_publisher.publish(conveyor_loadcell);

			//Conveyor Sensor status
			dssp_rs232_power_module_read_conveyor_sensor(&m_dConveyor_sensor);

			conveyor_sensor.data = m_dConveyor_sensor;
			conveyor_sensor_publisher.publish(conveyor_sensor);

			//Conveyor Movement status
			dssp_rs232_power_module_read_conveyor_movement(&m_iConveyor_movement);

			conveyor_movement.data = m_iConveyor_movement;
			conveyor_movement_publisher.publish(conveyor_movement);
		}

		loop_rate.sleep();
    	}

	//Ultrasonic Off//
	dssp_rs232_power_module_set_Ultrasonic(0);
	//RS232 Disconnect
	dssp_rs232_power_module_destroy();

    return 0;
}
