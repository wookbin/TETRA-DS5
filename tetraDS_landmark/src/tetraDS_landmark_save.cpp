
////TETRA_DS Landmark ROS Package_Ver 0.1
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseArray.h> 
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <message_filters/subscriber.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/AccelWithCovarianceStamped.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Header.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/Joy.h>
#include <sstream>
#include <vector>
#include <math.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string.h>
#include <time.h>
#include <thread>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <net/if.h>
#include <sys/types.h>
#include <dirent.h>
#include <error.h>
#include <cstdlib>
#include <algorithm>

//Service
#include "ar_track_alvar_msgs/AlvarMarkers.h" //MSG AR_TAG
//Save Mark ID
#include "tetraDS_landmark/savemark.h" //SRV

#define BUF_LEN 4096
using namespace std;
pthread_t p_thread;

FILE* fp;
int status;
char Textbuffer[BUF_LEN];
double m_baselink2cam_distance = 0.384; //384mm

typedef struct LANDMARK_POSE
{
    string header_frame_id;
    string ns;
    int mark_id;
    double pose_position_x;
    double pose_position_y;
    double pose_position_z;
    double pose_orientation_x;
    double pose_orientation_y;
    double pose_orientation_z;
    double pose_orientation_w;

}LANDMARK_POSE;

//AR_TAG Pose
int m_iAR_tag_id = -1;
float m_fAR_tag_pose_x = 0.0;
float m_fAR_tag_pose_y = 0.0;
float m_fAR_tag_pose_z = 0.0;
float m_fAR_tag_orientation_x = 0.0;
float m_fAR_tag_orientation_y = 0.0;
float m_fAR_tag_orientation_z = 0.0;
float m_fAR_tag_orientation_w = 0.0;
//Tracked Pose
float m_fTracked_pose_x = 0.0;
float m_fTracked_pose_y = 0.0;
float m_fTracked_pose_z = 0.0;
float m_fTracked_orientation_x = 0.0;
float m_fTracked_orientation_y = 0.0;
float m_fTracked_orientation_z = 0.0;
float m_fTracked_orientation_w = 0.0;
//calc pose
float m_fTracked_calc_pose_x = 0.0;
float m_fTracked_calc_pose_y = 0.0;
//Landmark add..
ros::Publisher landmark_pub;
visualization_msgs::Marker node;
//LandMark Pose//
LANDMARK_POSE _pLandMark;
bool m_bSave_Enable = false;
//**Command srv _ Service Server************************/
tetraDS_landmark::savemark savemark_cmd;
ros::ServiceServer savemark_service;

bool SaveMark_Command(tetraDS_landmark::savemark::Request &req, 
					  tetraDS_landmark::savemark::Response &res)
{
	bool bResult = false;
    
    m_bSave_Enable = true;

    /*
    ---
    bool command_Result
    */

	res.command_Result = m_bSave_Enable;
	return true;
}

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    if(joy->buttons[5]) //RB button
	{	
        //Save Landmark Data Enable 
        m_bSave_Enable = true;
	}
}


bool SaveLandMark(LANDMARK_POSE p)
{
    bool bResult = false;

    string m_strFilePathName;
    m_strFilePathName = "/home/tetra/LANDMARK/" + p.ns + ".txt";    
    fp = fopen(m_strFilePathName.c_str(), "w");
    if(fp == NULL)
    { 
        ROS_INFO("file is null");
        bResult = false;
    }
    else
    {
        if(p.mark_id == m_iAR_tag_id)
        {
            fprintf(fp, "0,%s,%s,%d,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",
                    p.header_frame_id.c_str(), 
                    p.ns.c_str(), 
                    p.mark_id,
                    p.pose_position_x,
                    p.pose_position_y,
                    p.pose_position_z,
                    p.pose_position_x + (m_fTracked_pose_x - p.pose_position_x), 
                    p.pose_position_y + (m_fTracked_pose_y - p.pose_position_y),
                    p.pose_position_z, 
                    p.pose_orientation_y - p.pose_orientation_x, 
                    p.pose_orientation_w + p.pose_orientation_z
                    );

                  
            fclose(fp);

            bResult = true;
        }
        else
            bResult = false;
    }

    return bResult;
}

//Map to AR_tag Transform
void Map2Mark_Callback(ar_track_alvar_msgs::AlvarMarkers req) 
{
    if (!req.markers.empty()) 
    {
        //Green circle LandMark//////////////////////////////////////////
        _pLandMark.header_frame_id = node.header.frame_id = req.markers[0].header.frame_id;
        node.header.stamp = ros::Time::now(); 
        node.type = visualization_msgs::Marker::SPHERE;
        _pLandMark.ns = node.ns = "marker_" + std::to_string(req.markers[0].id);
        _pLandMark.mark_id = node.id = req.markers[0].id;

        node.action = visualization_msgs::Marker::ADD; 
        _pLandMark.pose_position_x = node.pose.position.x = req.markers[0].pose.pose.position.x;
        _pLandMark.pose_position_y = node.pose.position.y = req.markers[0].pose.pose.position.y;
        _pLandMark.pose_position_z = node.pose.position.z = req.markers[0].pose.pose.position.z;
        _pLandMark.pose_orientation_x = node.pose.orientation.x = req.markers[0].pose.pose.orientation.x;
        _pLandMark.pose_orientation_y = node.pose.orientation.y = req.markers[0].pose.pose.orientation.y;
        _pLandMark.pose_orientation_z = node.pose.orientation.z = req.markers[0].pose.pose.orientation.z;
        _pLandMark.pose_orientation_w = node.pose.orientation.w = req.markers[0].pose.pose.orientation.w;

        // Points are green 
        node.color.a = 0.8; 
        node.color.r = 0.5;
        node.color.g = 1.0; 
        node.color.b = 0.0;  
        node.scale.x = 0.3;
        node.scale.y = 0.3;
        node.scale.z = 0.3;  

        if(m_bSave_Enable)
        {
            //Publish
            landmark_pub.publish(node);
            //Save Marker data//
            SaveLandMark(_pLandMark);
            m_bSave_Enable = false;
        }

    }
}

//AR_tagCallback
void AR_tagCallback(ar_track_alvar_msgs::AlvarMarkers req) 
{
    if (!req.markers.empty()) 
    {
        //AR_Tag data update...
        m_iAR_tag_id = req.markers[0].id;
        m_fAR_tag_pose_x = req.markers[0].pose.pose.position.x;
        m_fAR_tag_pose_y = req.markers[0].pose.pose.position.y;
        m_fAR_tag_pose_z = req.markers[0].pose.pose.position.z;
        m_fAR_tag_orientation_x = req.markers[0].pose.pose.orientation.x;
        m_fAR_tag_orientation_y = req.markers[0].pose.pose.orientation.y;
        m_fAR_tag_orientation_z = req.markers[0].pose.pose.orientation.z;
        m_fAR_tag_orientation_w = req.markers[0].pose.pose.orientation.w;

    }
    else
    {
        m_iAR_tag_id = -1;
    }
    
}

bool Tracked_pose_Callback(const geometry_msgs::PoseStamped::ConstPtr& msgPose)
{
    bool bResult = false;

    m_fTracked_pose_x = msgPose->pose.position.x;
    m_fTracked_pose_y = msgPose->pose.position.y;
    m_fTracked_pose_z = msgPose->pose.position.z;
    m_fTracked_orientation_x = msgPose->pose.orientation.x;
    m_fTracked_orientation_y = msgPose->pose.orientation.y;
    m_fTracked_orientation_z = msgPose->pose.orientation.z;
    m_fTracked_orientation_w = msgPose->pose.orientation.w;

    bResult = true;
    return bResult;
}


int main (int argc, char** argv)
{
    ros::init(argc, argv, "tetraDS_landmark_save");
    ros::NodeHandle nh;
    //AR_TAG_subscriber//
    ros::Subscriber map2marker_sub = nh.subscribe("map_to_marker_pose", 100, Map2Mark_Callback);
    //AR_TAG_subscriber//
    ros::Subscriber AR_sub = nh.subscribe("ar_pose_marker", 100, AR_tagCallback);
    //tracked_pose subscriber
    ros::Subscriber tracked_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("tracked_pose", 100, Tracked_pose_Callback);

    //landmark add..
    ros::NodeHandle mark_n;
    landmark_pub = mark_n.advertise<visualization_msgs::Marker>("marker/node", 100);

    //Joystick add...
    ros::NodeHandle njoy;
    ros::Subscriber joy_sub = njoy.subscribe<sensor_msgs::Joy>("joy", 10, joyCallback);
    ros::Rate loop_rate(30); //30hz

    //Command Service//
    ros::NodeHandle service_h;
	savemark_service = service_h.advertiseService("savemark_cmd", SaveMark_Command);

    while(ros::ok())
    {
        ros::spinOnce();
      
        loop_rate.sleep();
    }

    return 0;
}
