#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseArray.h> //teb_poses...
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h> //teb markers..
#include <visualization_msgs/MarkerArray.h>
#include <message_filters/subscriber.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/AccelWithCovarianceStamped.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Header.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Joy.h> 
#include <sensor_msgs/Range.h>
#include <teb_local_planner/TrajectoryMsg.h>
#include <teb_local_planner/TrajectoryPointMsg.h>
#include <teb_local_planner/FeedbackMsg.h>
#include <actionlib_msgs/GoalID.h>

#include <math.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <vector>
#include <iostream>
#include <thread>
#include <dirent.h>
#include <signal.h>

//Service_srv file
#include "tetraDS_TCP/gotolocation.h" //SRV
#include "tetraDS_TCP/gotolocation2.h" //SRV
#include "tetraDS_TCP/gotocancel.h" //SRV
#include "tetraDS_TCP/getlocation.h" //SRV
#include "tetraDS_TCP/setlocation.h" //SRV
#include "tetraDS_TCP/setsavemap.h" //SRV
#include "tetraDS_TCP/getlocationlist.h" //SRV
#include "tetraDS_TCP/deletelocation.h" //SRV
#include "tetraDS_TCP/runmapping.h" //SRV
#include "tetraDS_TCP/runnavigation.h" //SRV
#include "tetraDS_TCP/rosnodekill.h" //SRV
#include "tetraDS_TCP/getmaplist.h" //SRV
#include "tetraDS_TCP/setmaxspeed.h" //SRV
#include "tetraDS_TCP/dockingcontrol.h" //SRV
#include "tetraDS_TCP/power_set_outport.h" //SRV
#include "tetraDS_TCP/power_get_io_status.h" //SRV
//add...230405_wbjin
#include "tetraDS_TCP/power_set_single_outport.h" //SRV

#define BUF_LEN 4096
using namespace std;

char buffer[BUF_LEN];
char Send_buffer[BUF_LEN];
struct sockaddr_in server_addr, client_addr;
char temp[20];
int server_fd, client_fd;
int len, msg_size;
pthread_t p_auto_thread;
pthread_t p_auto_thread2;
bool m_bflag_thread = false;

///*Socket Data define *///
char cSTX[] = "DS";
char cETX[] = "XX";
char* m_cSTX;
char* m_cLEN;
char* m_cMOD;
char* m_cCMD;
char* m_cPARAM[16];
char* m_cDATA;
char* m_cETX;
char m_cCOMMA[] = ",";
int m_iLEN = 0;
int m_iMOD = 0;

//data///
int m_iTotal_Map_cnt = 0;
int m_iTotal_Location_cnt = 0;
string m_strMap[100] = {"", }; //Max Map data
string m_strLocation[255] = {"", }; //Max Location data


//*Custom Service Client*//
ros::ServiceClient goto_cmd_client;
tetraDS_TCP::gotolocation goto_cmd_service;
ros::ServiceClient goto_cmd_client2;
tetraDS_TCP::gotolocation2 goto_cmd_service2;
ros::ServiceClient gotocancel_cmd_client;
tetraDS_TCP::gotocancel gotocancel_cmd_service;
ros::ServiceClient setlocation_cmd_client;
tetraDS_TCP::setlocation setlocation_cmd_service;
ros::ServiceClient getlocation_cmd_client;
tetraDS_TCP::getlocation getlocation_cmd_service;
ros::ServiceClient locationlist_cmd_client;
tetraDS_TCP::getlocationlist locationlist_cmd_service;
ros::ServiceClient maplist_cmd_client;
tetraDS_TCP::getmaplist maplist_cmd_service;
ros::ServiceClient setspeed_cmd_client;
tetraDS_TCP::setmaxspeed setspeed_cmd_service;
ros::ServiceClient navigation_cmd_client;
tetraDS_TCP::runnavigation navigation_cmd_service;
ros::ServiceClient mapping_cmd_client;
tetraDS_TCP::runmapping mapping_cmd_service;
ros::ServiceClient mapsave_cmd_client;
tetraDS_TCP::setsavemap mapsave_cmd_service;
ros::ServiceClient nodekill_cmd_client;
tetraDS_TCP::rosnodekill nodekill_cmd_service;
ros::ServiceClient deletelocation_cmd_client;
tetraDS_TCP::deletelocation deletelocation_cmd_service;
ros::ServiceClient dockingcontrol_cmd_client;
tetraDS_TCP::dockingcontrol dockingcontrol_cmd_service;
ros::ServiceClient output_cmd_client;
tetraDS_TCP::power_set_outport output_cmd_service;
ros::ServiceClient gpio_status_cmd_client;
tetraDS_TCP::power_get_io_status gpio_status_cmd_service;
//add_230405
ros::ServiceClient single_output_cmd_client;
tetraDS_TCP::power_set_single_outport single_output_cmd_service;


//***************************************************************************************************************************************/
//Struct define//
typedef struct ODOMETRY
{
    double dOdom_position_x = 0.0;
    double dOdom_position_y = 0.0;
    double dOdom_position_z = 0.0;
    double dOdom_quaternion_x = 0.0;
    double dOdom_quaternion_y = 0.0;
    double dOdom_quaternion_z = 0.0;
    double dOdom_quaternion_w = 0.0;
    double dTwist_linear = 0.0;
    double dTwist_angular = 0.0;

}ODOMETRY;
ODOMETRY _pOdometry;

typedef struct AMCL_POSE
{
    double dPoseAMCLx = 0.0;
    double dPoseAMCLy = 0.0;
    double dPoseAMCLz = 0.0;
    double dPoseAMCLqx = 0.0;
    double dPoseAMCLqy = 0.0;
    double dPoseAMCLqz = 0.0;
    double dPoseAMCLqw = 0.0;

}AMCL_POSE;
AMCL_POSE _pAMCL_pose;

typedef struct GOAL_POSE
{
    double dGoal_positionX = 0.0;
    double dGoal_positionY = 0.0;
    double dGoal_positionZ = 0.0;
    double dGoal_quarterX = 0.0;
    double dGoal_quarterY = 0.0;
    double dGoal_quarterZ = 0.0;
    double dGoal_quarterW = 1.0;

}GOAL_POSE;
GOAL_POSE _pGoal_pose;

typedef struct ROBOT_STATUS
{
    int iCallback_Battery = 0;
    int iCallback_ErrorCode = 0;
    int iCallback_EMG = 0;
    int iCallback_Bumper = 0;
    int iCallback_Charging_status = 0;
    //Bumper Collision Behavior//
    int iBumperCollisionBehavor_cnt = 0;
    //auto test
    int iMovebase_Result = 0;
    //Conveyor Info..(Option)
    double dLoadcell_weight = 0.0;
    int iConveyor_Sensor_info = 0;
    int HOME_ID = 0; //Docking ID Param Read//
    int CONVEYOR_ID = 0;
    int CONVEYOR_MOVEMENT = 0; // 0: nomal , 1: Loading , 2: Unloading

}ROBOT_STATUS;
ROBOT_STATUS _pRobot_Status;

//***************************************************************************************************************************************/
//Callback Function///
void OdometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    _pOdometry.dOdom_position_x = msg->pose.pose.position.x;
    _pOdometry.dOdom_position_y = msg->pose.pose.position.y;
    _pOdometry.dOdom_position_z = msg->pose.pose.position.z;
    _pOdometry.dOdom_quaternion_x = msg->pose.pose.orientation.x;
    _pOdometry.dOdom_quaternion_y = msg->pose.pose.orientation.y;
    _pOdometry.dOdom_quaternion_z = msg->pose.pose.orientation.z;
    _pOdometry.dOdom_quaternion_w = msg->pose.pose.orientation.w;

    _pOdometry.dTwist_linear = msg->twist.twist.linear.x;
    _pOdometry.dTwist_angular = msg->twist.twist.angular.z;

}

void resultCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr& msgResult)
{
    uint8_t PENDING    = 0;   // The goal has yet to be processed by the action server
    uint8_t ACTIVE     = 1;   // The goal is currently being processed by the action server
    uint8_t PREEMPTED  = 2;   // The goal received a cancel request after it started executing
                                //   and has since completed its execution (Terminal State)
    uint8_t SUCCEEDED  = 3;   // The goal was achieved successfully by the action server (Terminal State)
    uint8_t ABORTED    = 4;   // The goal was aborted during execution by the action server due
                                //    to some failure (Terminal State)
    uint8_t REJECTED   = 5;   // The goal was rejected by the action server without being processed,
                                //    because the goal was unattainable or invalid (Terminal State)
    uint8_t PREEMPTING = 6;   // The goal received a cancel request after it started executing
                                //    and has not yet completed execution
    uint8_t RECALLING  = 7;   // The goal received a cancel request before it started executing,
                                //    but the action server has not yet confirmed that the goal is canceled
    uint8_t RECALLED   = 8;   // The goal received a cancel request before it started executing
                                //    and was successfully cancelled (Terminal State)
    uint8_t LOST       = 9;   // An action client can determine that a goal is LOST. This should not be
                                //    sent over the wire by an action server
    // time_t curr_time;
    // struct tm *curr_tm;
    // curr_time = time(NULL);
    // curr_tm = localtime(&curr_time);
    _pRobot_Status.iMovebase_Result = msgResult->status.status;
    ROS_INFO("[SUCCEEDED]resultCallback: %d ",msgResult->status.status);
   
}

void BatteryCallback(const std_msgs::Int32::ConstPtr& msg)
{
    _pRobot_Status.iCallback_Battery = msg->data;
    //printf("_pRobot_Status.iCallback_Battery: %d \n",_pRobot_Status.iCallback_Battery);
}

void ChargingCallback(const std_msgs::Int32::ConstPtr& msg)
{
    _pRobot_Status.iCallback_Charging_status = msg->data;
    //printf("_pRobot_Status.iCallback_Charging_status: %d \n",_pRobot_Status.iCallback_Charging_status);
}

void EMGCallback(const std_msgs::Int32::ConstPtr& msg)
{
    _pRobot_Status.iCallback_EMG = msg->data;
}

void BumperCallback(const std_msgs::Int32::ConstPtr& msg)
{
    _pRobot_Status.iCallback_Bumper = msg->data;
}

//***************************************************************************************************************************************/
//Sevice Function///
bool GotoLocation(string strLocation_name)
{
    bool bResult = false;
    goto_cmd_service.request.Location = strLocation_name;
    goto_cmd_client.call(goto_cmd_service);

    _pGoal_pose.dGoal_positionX = goto_cmd_service.response.goal_positionX;
    _pGoal_pose.dGoal_positionY = goto_cmd_service.response.goal_positionY;
    _pGoal_pose.dGoal_quarterX = goto_cmd_service.response.goal_quarterX;
    _pGoal_pose.dGoal_quarterY = goto_cmd_service.response.goal_quarterY;
    _pGoal_pose.dGoal_quarterZ = goto_cmd_service.response.goal_quarterZ;
    _pGoal_pose.dGoal_quarterW = goto_cmd_service.response.goal_quarterW;

    sprintf(Send_buffer,"DS,7,GO1,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,XX", 
                _pGoal_pose.dGoal_positionX,_pGoal_pose.dGoal_positionY,
                _pGoal_pose.dGoal_quarterX,_pGoal_pose.dGoal_quarterY,_pGoal_pose.dGoal_quarterZ,_pGoal_pose.dGoal_quarterW);

    bResult = true;
    return bResult;
}

bool GotoLocation2(double goal_positionX, double goal_positionY, double goal_quarterX, double goal_quarterY, double goal_quarterZ, double goal_quarterW)
{
    bool bResult = false;

    goto_cmd_service2.request.goal_positionX = goal_positionX;
    goto_cmd_service2.request.goal_positionY = goal_positionY;
    goto_cmd_service2.request.goal_quarterX = goal_quarterX;
    goto_cmd_service2.request.goal_quarterY = goal_quarterY;
    goto_cmd_service2.request.goal_quarterZ = goal_quarterZ;
    goto_cmd_service2.request.goal_quarterW = goal_quarterW;

    goto_cmd_client2.call(goto_cmd_service2);

    // sprintf(Send_buffer,"DS,7,GO2,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,XX", 
    //             _pGoal_pose.dGoal_positionX,_pGoal_pose.dGoal_positionY,
    //             _pGoal_pose.dGoal_quarterX,_pGoal_pose.dGoal_quarterY,_pGoal_pose.dGoal_quarterZ,_pGoal_pose.dGoal_quarterW);

    bResult = true;
    return bResult;
}

bool GotoCancel()
{
    bool bResult = false;

    gotocancel_cmd_service.request.Location_id = "";
    gotocancel_cmd_client.call(gotocancel_cmd_service);

    bResult = true;
    return bResult;
}

bool Getlocation()
{
    bool bResult = false;
    getlocation_cmd_client.call(getlocation_cmd_service);

    _pAMCL_pose.dPoseAMCLx = getlocation_cmd_service.response.poseAMCLx;
    _pAMCL_pose.dPoseAMCLy = getlocation_cmd_service.response.poseAMCLy;

    _pAMCL_pose.dPoseAMCLqx = getlocation_cmd_service.response.poseAMCLqx;
    _pAMCL_pose.dPoseAMCLqy = getlocation_cmd_service.response.poseAMCLqy;
    _pAMCL_pose.dPoseAMCLqz = getlocation_cmd_service.response.poseAMCLqz;
    _pAMCL_pose.dPoseAMCLqw = getlocation_cmd_service.response.poseAMCLqw;

    sprintf(Send_buffer, "DS,7,AMCL,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,XX", 
                _pAMCL_pose.dPoseAMCLx, _pAMCL_pose.dPoseAMCLy,
                _pAMCL_pose.dPoseAMCLqx,_pAMCL_pose.dPoseAMCLqy,_pAMCL_pose.dPoseAMCLqz,_pAMCL_pose.dPoseAMCLqw);

    bResult = true;
    return bResult;
}

bool GetDataAll()
{
    bool bResult = false;
    getlocation_cmd_client.call(getlocation_cmd_service);

    _pAMCL_pose.dPoseAMCLx = getlocation_cmd_service.response.poseAMCLx;
    _pAMCL_pose.dPoseAMCLy = getlocation_cmd_service.response.poseAMCLy;

    _pAMCL_pose.dPoseAMCLqx = getlocation_cmd_service.response.poseAMCLqx;
    _pAMCL_pose.dPoseAMCLqy = getlocation_cmd_service.response.poseAMCLqy;
    _pAMCL_pose.dPoseAMCLqz = getlocation_cmd_service.response.poseAMCLqz;
    _pAMCL_pose.dPoseAMCLqw = getlocation_cmd_service.response.poseAMCLqw;


    sprintf(Send_buffer,"DS,12,DATA,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%d,%d,%d,%d,XX", 
                _pAMCL_pose.dPoseAMCLx, _pAMCL_pose.dPoseAMCLy,
                _pAMCL_pose.dPoseAMCLqx,_pAMCL_pose.dPoseAMCLqy,_pAMCL_pose.dPoseAMCLqz,_pAMCL_pose.dPoseAMCLqw,
                _pRobot_Status.iCallback_Battery, _pRobot_Status.iCallback_EMG, _pRobot_Status.iCallback_Bumper, _pRobot_Status.iCallback_Charging_status,
                _pRobot_Status.iMovebase_Result);

    bResult = true;
    return bResult;
}

bool NavigationMode_ON(string strMap_name)
{
    bool bResult = false;

    navigation_cmd_service.request.map_name = strMap_name;
    navigation_cmd_client.call(navigation_cmd_service);


    bResult = true;
    return bResult;
}

bool GetLocation_List()
{
    bool bResult = false;
    string strTemp;

    locationlist_cmd_client.call(locationlist_cmd_service);
    m_iTotal_Location_cnt = locationlist_cmd_service.response.list_num;

    for(int i=0; i<m_iTotal_Location_cnt; i++)
    {
        m_strLocation[i] = locationlist_cmd_service.response.location_name[i];
        strTemp += (m_strLocation[i] + ",");
    }

    sprintf(Send_buffer,"DS,%d,LOCLIST,%sXX", 
                m_iTotal_Location_cnt+1,
                strTemp.c_str()
                );


    bResult = true;
    return bResult;
}

bool GetMap_List()
{
    bool bResult = false;
    string strTemp;

    maplist_cmd_client.call(maplist_cmd_service);
    m_iTotal_Map_cnt = maplist_cmd_service.response.list_num;

    for(int i=0; i<m_iTotal_Map_cnt; i++)
    {
        m_strMap[i] = maplist_cmd_service.response.map_name[i];
        strTemp += (m_strMap[i] + ",");
    }

    sprintf(Send_buffer,"DS,%d,MAPLIST,%sXX", 
                m_iTotal_Map_cnt+1,
                strTemp.c_str()
                );

    bResult = true;
    return bResult;
}

bool Set_Robot_MaxSpeed(double dSpeed)
{
    bool bResult = false;

    setspeed_cmd_service.request.speed = dSpeed;
    setspeed_cmd_client.call(setspeed_cmd_service);

    sprintf(Send_buffer,"DS,2,SPEED,%.3f,XX", setspeed_cmd_service.response.set_vel);

    bResult = true;
    return bResult;
}

bool Set_Location(string strLocationName)
{
    bool bResult = false;

    setlocation_cmd_service.request.Location = strLocationName;
    setlocation_cmd_client.call(setlocation_cmd_service);

    sprintf(Send_buffer,"DS,2,LSV,%s,XX", strLocationName.c_str());

    bResult = true;
    return bResult;
}

bool Set_Output(int Output0, int Output1, int Output2, int Output3, int Output4, int Output5, int Output6, int Output7)
{
    bool bResult = false;

    output_cmd_service.request.Output0 = Output0;
    output_cmd_service.request.Output1 = Output1;
    output_cmd_service.request.Output2 = Output2;
    output_cmd_service.request.Output3 = Output3;
    output_cmd_service.request.Output4 = Output4;
    output_cmd_service.request.Output5 = Output5;
    output_cmd_service.request.Output6 = Output6;
    output_cmd_service.request.Output7 = Output7;
    output_cmd_client.call(output_cmd_service);

    bResult = true;
    return bResult;

}

//add_230405_wbjin
bool Set_Single_Output(int Output_id, int iValue)
{
    bool bResult = false;
    
    single_output_cmd_service.request.ID = Output_id;
    single_output_cmd_service.request.VALUE = iValue;
    single_output_cmd_client.call(single_output_cmd_service);

    bResult = true;
    return bResult;

}

bool Get_GPIO_Status()
{
    bool bResult = false;

    gpio_status_cmd_client.call(gpio_status_cmd_service);

    sprintf(Send_buffer, "DS,17,GPIO,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,XX", 
            gpio_status_cmd_service.response.Input0,gpio_status_cmd_service.response.Input1,gpio_status_cmd_service.response.Input2,
            gpio_status_cmd_service.response.Input3,gpio_status_cmd_service.response.Input4,gpio_status_cmd_service.response.Input5,
            gpio_status_cmd_service.response.Input6,gpio_status_cmd_service.response.Input7,
            gpio_status_cmd_service.response.Output0,gpio_status_cmd_service.response.Output1,gpio_status_cmd_service.response.Output2,
            gpio_status_cmd_service.response.Output3,gpio_status_cmd_service.response.Output4,gpio_status_cmd_service.response.Output5,
            gpio_status_cmd_service.response.Output6,gpio_status_cmd_service.response.Output7);

    bResult = true;
    return bResult;

}

bool MappingMode_ON()
{
    bool bResult = false;

    mapping_cmd_client.call(mapping_cmd_service);

    bResult = true;
    return bResult;
}

bool NodeKill()
{
    bool bResult = false;

    nodekill_cmd_client.call(nodekill_cmd_service);

    bResult = true;
    return bResult;
}

bool Map_Save(string strMapName)
{
    bool bResult = false;

    mapsave_cmd_service.request.map_name = strMapName;
    mapsave_cmd_client.call(mapsave_cmd_service);

    bResult = true;
    return bResult;
}

bool Dcoking_Control(int iMarkerID, int iMode)
{
    bool bResult = false;

    //int32 id
    //int32 mode
    dockingcontrol_cmd_service.request.id = iMarkerID;
    dockingcontrol_cmd_service.request.mode = iMode;
    dockingcontrol_cmd_client.call(dockingcontrol_cmd_service);

    bResult = true;
    return bResult;
}


//***************************************************************************************************************************************/

void my_handler(sig_atomic_t s)
{
    printf("Caught signal %d\n",s);
    exit(1); 
}

constexpr unsigned int HashCode(const char* str)
{
    return str[0] ? static_cast<unsigned int>(str[0]) + 0xEDB8832Full * HashCode(str + 1) : 8603;
}

bool DoParsing(char* data) 
{
    bool bResult = false;

    char* token;
    char parsing[16][16];
    int iIndex = 0;
    int iCnt = 0;

    token = strtok(data, m_cCOMMA);
    while (token != NULL)
    {
        strcpy(parsing[iIndex++], token);
        token = strtok(NULL, m_cCOMMA);
    }

    //Packet Sorting.../////////////////////////////////////////////////////
    m_cSTX = parsing[0];
    m_cLEN = parsing[1];
    m_cMOD = parsing[2];
    m_cCMD = parsing[3];
    for(int i=0; i < (atoi(m_cLEN)-2); i++)
    {
        m_cPARAM[i] = parsing[4+i];
        iCnt++;
    }
    m_cETX = parsing[4+iCnt];
    
    ///ACK or NAK TEST////////////////////////////////////////
    if(!strcmp(cSTX, m_cSTX) && !strcmp(cETX, m_cETX))
    {
        switch(HashCode(m_cCMD))
        {
            case HashCode("ODOM"): //Odometry data
                if(m_cPARAM[0] == "1") //TETRA origin Odometry
                {
                    //printf("m_cPARAM 1 \n");
                    sprintf(Send_buffer, "DS,8,ODOM,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,XX", 
                    _pOdometry.dOdom_position_x, _pOdometry.dOdom_position_y, _pOdometry.dOdom_position_z,
                    _pOdometry.dOdom_quaternion_x,_pOdometry.dOdom_quaternion_y,_pOdometry.dOdom_quaternion_z,_pOdometry.dOdom_quaternion_w);
                }
                else // Twist_velocity
                {
                    //printf("m_cPARAM 2 \n");
                    sprintf(Send_buffer, "DS,3,ODOM,%.3f,%.3f,XX", 
                    _pOdometry.dTwist_linear, _pOdometry.dTwist_angular);
                }
                break;
            case HashCode("AMCL"): //Robot Pose data
                Getlocation();
                break;
            case HashCode("STATUS"): //TETRA Status data
                sprintf(Send_buffer, "DS,6,STATUS,%d,%d,%d,%d,%d,XX", 
                _pRobot_Status.iCallback_Battery, _pRobot_Status.iCallback_EMG, _pRobot_Status.iCallback_Bumper, _pRobot_Status.iCallback_Charging_status, _pRobot_Status.iMovebase_Result);
                break;
            case HashCode("MAPLIST"): //Save Map file List data
                GetMap_List();
                break;
            case HashCode("LOCLIST"): //Save WayPoint file List data
                GetLocation_List();
                break;
            case HashCode("NAV"): //Move_base(navigation) Mode Service Call
                NavigationMode_ON(m_cPARAM[0]);
                break;
            case HashCode("SLAM"): //Cartographer Mode Service Call
                MappingMode_ON();
                break;
            case HashCode("MSV"): // Map file Save Service Call
                Map_Save(m_cPARAM[0]);
                break;
            case HashCode("KILL"): //Mapping Mode or Navigation Mode Kill Service Call
                NodeKill();
                break;
            case HashCode("DOC"): // Docking command
                Dcoking_Control(atoi(m_cPARAM[0]), atoi(m_cPARAM[1]));
                break;
            case HashCode("GO1"): // Move to saved location 
                _pRobot_Status.iMovebase_Result = 0;
                GotoLocation(m_cPARAM[0]);
                break;
            case HashCode("GO2"): // Move to location coordinates
                _pRobot_Status.iMovebase_Result = 0;
                GotoLocation2(atof(m_cPARAM[0]), atof(m_cPARAM[1]), atof(m_cPARAM[2]), atof(m_cPARAM[3]), atof(m_cPARAM[4]), atof(m_cPARAM[5]));
                break;
            case HashCode("GOCXL"): // Move to saved location || Move to location coordinates Cancel
                GotoCancel();
                break;
            case HashCode("SPEED"): // TETRA Navigation Move Speed Set
                Set_Robot_MaxSpeed(atof(m_cPARAM[0]));
                break;
            case HashCode("LSV"): // Save to Location data
                Set_Location(m_cPARAM[0]);
                break;
            case HashCode("OUT"): // GPIO_Output command
                Set_Output(atoi(m_cPARAM[0]),atoi(m_cPARAM[1]),atoi(m_cPARAM[2]),atoi(m_cPARAM[3]),
                            atoi(m_cPARAM[4]),atoi(m_cPARAM[5]),atoi(m_cPARAM[6]),atoi(m_cPARAM[7]));
                break;
            case HashCode("SOUT"): // GPIO_Sigle Output command
                Set_Single_Output(atoi(m_cPARAM[0]),atoi(m_cPARAM[1]));
                break;
            case HashCode("GPIO"): // GPIO Status Check command
                Get_GPIO_Status();
                break;
            case HashCode("DATA"): // AMCL Pose & Robot Status Data all...
                GetDataAll();
                break;

        }



        bResult = true;
        //printf("ACK \n");
        // sprintf(Send_buffer, "ACK");
        write(client_fd, Send_buffer, strlen(Send_buffer));
    }
    else
    {
        bResult = false;
        printf("NAK \n");
        sprintf(Send_buffer, "NAK");
        write(client_fd, Send_buffer, strlen(Send_buffer));
    }    


    return bResult;
}

void *AutoThread_function(void *data)
{
    while(1)
    {
        if(m_bflag_thread)
        {
            msg_size = read(client_fd, buffer, BUF_LEN);
            if(msg_size < 1)
            {
                m_bflag_thread = false;
            }
            else
            {
                printf("[DATA] = %s [msg_size] = %d \n",buffer, msg_size);
                memset(&Send_buffer, 0x00, sizeof(Send_buffer)); //clear Send_buffer
                DoParsing(buffer);
                memset(&buffer, 0x00, sizeof(buffer)); //clear buffer
            }

        }

        usleep(100000); //10ms
    }
    pthread_cancel(p_auto_thread); //Thread kill
}

void *SocketCheck_Thread_function(void *data)
{
    while(1)
    {
        
        client_fd = accept(server_fd, (sockaddr *)&client_addr, (socklen_t*)&len);
        if(client_fd < 0)
        {
            printf("Server: accept failed.\n");
            m_bflag_thread = false;
            exit(0);
        }

        inet_ntop(AF_INET, &client_addr.sin_addr.s_addr, temp, sizeof(temp));
        printf("Server : %s client connected.\n", temp);
        m_bflag_thread = true;
        

        usleep(1000000); //100ms
    }
    pthread_cancel(p_auto_thread2); //Thread kill
}

//add _ GUI Button callback fuction...
void TESTCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    if(joy->buttons[5] == 1) //HOME goto...
    {
        _pRobot_Status.iMovebase_Result = 10;
    }

}

int main(int argc, char* argv[])
{
    signal(SIGINT,my_handler);

    ros::init(argc,argv, "tetraDS_TCP", ros::init_options::NoSigintHandler);
    ////Subscriber//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //ROS msg_subscriber//
    ros::NodeHandle nh;
    ros::Subscriber odom_sub = nh.subscribe("odom", 100, OdometryCallback);
    ros::Subscriber result_sub = nh.subscribe<move_base_msgs::MoveBaseActionResult>("move_base/result", 10, resultCallback);
    //Infomation_subscriber//
    ros::NodeHandle nInfo;
    ros::Subscriber tetra_battery = nInfo.subscribe<std_msgs::Int32>("tetra_battery", 1, BatteryCallback);
    ros::Subscriber emg_state = nInfo.subscribe<std_msgs::Int32>("emg_state", 1, EMGCallback);
    ros::Subscriber bumper_data = nInfo.subscribe<std_msgs::Int32>("bumper_data", 1, BumperCallback);
    ros::Subscriber docking_status = nInfo.subscribe<std_msgs::Int32>("docking_status", 1, ChargingCallback);

    //add GUI...
    ros::NodeHandle nTest;
    ros::Subscriber Test_sub = nTest.subscribe("/rviz_visual_tools_gui", 10, TESTCallback);

    ////tetraDS ServiceClient///////////////////////////////////////////////////////////////////////////////////
    ros::NodeHandle client_h;
    goto_cmd_client  = client_h.serviceClient<tetraDS_TCP::gotolocation>("goto_cmd");
    goto_cmd_client2 = client_h.serviceClient<tetraDS_TCP::gotolocation2>("goto_cmd2");
    gotocancel_cmd_client = client_h.serviceClient<tetraDS_TCP::gotocancel>("gotocancel_cmd");
    getlocation_cmd_client = client_h.serviceClient<tetraDS_TCP::getlocation>("getlocation_cmd");
    locationlist_cmd_client = client_h.serviceClient<tetraDS_TCP::getlocationlist>("locationlist_cmd");
    maplist_cmd_client = client_h.serviceClient<tetraDS_TCP::getmaplist>("maplist_cmd");
    setspeed_cmd_client = client_h.serviceClient<tetraDS_TCP::setmaxspeed>("setspeed_cmd");
    setlocation_cmd_client = client_h.serviceClient<tetraDS_TCP::setlocation>("setlocation_cmd");
    navigation_cmd_client = client_h.serviceClient<tetraDS_TCP::runnavigation>("navigation_cmd");
    mapping_cmd_client = client_h.serviceClient<tetraDS_TCP::runmapping>("mapping_cmd");
    mapsave_cmd_client = client_h.serviceClient<tetraDS_TCP::setsavemap>("savemap_cmd");
    nodekill_cmd_client = client_h.serviceClient<tetraDS_TCP::rosnodekill>("nodekill_cmd");
    deletelocation_cmd_client = client_h.serviceClient<tetraDS_TCP::deletelocation>("delete_location_cmd");
    dockingcontrol_cmd_client = client_h.serviceClient<tetraDS_TCP::dockingcontrol>("docking_cmd");
    output_cmd_client = client_h.serviceClient<tetraDS_TCP::power_set_outport>("Power_outport_cmd");
    gpio_status_cmd_client = client_h.serviceClient<tetraDS_TCP::power_get_io_status>("Power_io_status_cmd");
    //add
    single_output_cmd_client = client_h.serviceClient<tetraDS_TCP::power_set_single_outport>("Power_single_outport_cmd");

    //***************************************************************************************************************************************/
    //TCP/IP Socket Loop...///
    if(argc != 2)
    {
        printf("usage : %s [port]\n", argv[0]);
        exit(0);
    }
 
    if((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == -1)
    {// 소켓 생성
        printf("Server : Can't open stream socket\n");
        exit(0);
    }
    
    int option = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &option, sizeof(option));
    memset(&server_addr, 0x00, sizeof(server_addr));
    //server_Addr 을 NULL로 초기화
 
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr.sin_port = htons(atoi(argv[1]));
    //server_addr 셋팅
 
    if(bind(server_fd, (struct sockaddr *)&server_addr, sizeof(server_addr)) <0)
    {//bind() 호출
        printf("Server : Can't bind local address.\n");
        exit(0);
    }
 
    if(listen(server_fd, 5) < 0)
    {//소켓을 수동 대기모드로 설정
        printf("Server : Can't listening connect.\n");
        exit(0);
    }
 
    memset(buffer, 0x00, sizeof(buffer)); //Send_buffer
    printf("Server : wating connection request.\n");
    m_bflag_thread = false;
    len = sizeof(client_addr);

    //***************************************************************************************************************************************/


    /*Thread Create...*/
    int auto_thread_id;
    int a = 1;
    auto_thread_id = pthread_create(&p_auto_thread, NULL, AutoThread_function, (void *)&a);
    if (auto_thread_id < 0)
    {
        printf("auto thread create error !!");
        exit(0);
    }  

    /* SocketCheck Thread Create...*/
    int SocketCheck_thread_id;
    int a2 = 1;
    SocketCheck_thread_id = pthread_create(&p_auto_thread2, NULL, SocketCheck_Thread_function, (void *)&a2);
    if (SocketCheck_thread_id < 0)
    {
        printf("SocketCheck_thread create error !!");
        exit(0);
    } 

    ros::Rate loop_rate(30); // 30Hz

    while(ros::ok())
    { 
        ros::spinOnce(); 

        
        loop_rate.sleep();
    }

    pthread_cancel(p_auto_thread); //Thread kill
    pthread_cancel(p_auto_thread2); //Thread kill
    close(server_fd);
    
    printf("---server_fd Close--.\n");
    return 0;
}
