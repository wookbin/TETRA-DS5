#define BUF_LEN 4096

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


using namespace std;

char buffer[BUF_LEN];
char Send_buffer[BUF_LEN];
struct sockaddr_in server_addr, client_addr;
char temp[20];
int server_fd, client_fd;
int len, msg_size;
pthread_t p_auto_thread;
bool m_bflag_thread = false;

char cSTX[] = "DS";
char cETX[] = "XX";

char* m_cSTX;
char* m_cLEN;
char* m_cMOD;
char* m_cCMD;
char* m_cPARAM;
char* m_cETX;
char m_cCOMMA[] = ",";

void my_handler(sig_atomic_t s)
{
    printf("Caught signal %d\n",s);
    exit(1); 
}

bool DoParsing(char* data) 
{
    bool bResult = false;
    char* token;
    int index_cnt = 0;

    token = strtok(data, m_cCOMMA);
    while (token != NULL)
    {
        //printf("DATA: %s \n", token);
        switch(index_cnt)
        {
            case 0:
                m_cSTX = token;
                //printf("0 \n");
                break;
            case 1:
                m_cLEN = token;
                //printf("1 \n");
                break;
            case 2:
                m_cMOD = token;
                //printf("2 \n");
                break;
            case 3:
                m_cCMD = token;
                //printf("3 \n");
                break;
            case 4:
                m_cPARAM = token;
                //printf("4 \n");
                break;
            case 5:
                m_cETX = token;
                //printf("5 \n");
                break;
        }
        
        index_cnt++;

        token = strtok(NULL, m_cCOMMA);
    }

    //printf("Output: %s,%s,%s,%s,%s \n", m_cSTX, m_cLEN, m_cMOD, m_cPARAM, m_cETX);

    if(!strcmp(cSTX, m_cSTX) && !strcmp(cETX, m_cETX))
    {
        bResult = true;
        printf("ACK \n");
        sprintf(Send_buffer, "ACK");
        write(client_fd, Send_buffer, sizeof(Send_buffer));
    }
    else
    {
        bResult = false;
        printf("NAK \n");
        sprintf(Send_buffer, "NAK");
        write(client_fd, Send_buffer, sizeof(Send_buffer));
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
                printf("[DATA] = %s, [msg_size] = %d \n",buffer, msg_size);

                DoParsing(buffer);
                memset(&buffer, 0x00, sizeof(buffer)); //clear buffer
                memset(&Send_buffer, 0x00, sizeof(Send_buffer)); //clear Send_buffer
            }

        }

        usleep(100000); //10ms
    }
    pthread_cancel(p_auto_thread); //Thread kill
}

int main(int argc, char* argv[])
{
    signal(SIGINT,my_handler);

    ros::init(argc,argv, "tetraDS_TCP", ros::init_options::NoSigintHandler);

    ros::NodeHandle n;
  
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

    /*Thread Create...*/
    int auto_thread_id;
    int a = 1;
    auto_thread_id = pthread_create(&p_auto_thread, NULL, AutoThread_function, (void *)&a);
    if (auto_thread_id < 0)
    {
        printf("auto thread create error !!");
        exit(0);
    }  


    ros::Rate loop_rate(60); // 60Hz

    while(ros::ok())
    { 
        ros::spinOnce(); 

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
    
            
            loop_rate.sleep();
        }

    pthread_cancel(p_auto_thread); //Thread kill
    close(server_fd);
    
    printf("---server_fd Close--.\n");
    return 0;
}
