#include "ros/ros.h"
#include "rgmp/Robotcontrol.h"
#include "boost/algorithm/string.hpp"
#include "robot_control_cmd.h"
#include "robot_state.h"
#include <fstream>
#include <iostream>
#include <iomanip>
#include <string>
#include <cstdlib>
#include "boost/thread.hpp"

#define ROBOT_DOF 4
#define ROBOT_FILE_BUFFER_LENGTH 20

#define ROBOT_HEAD_SERVO1_NAME      "head_1_servo"
#define ROBOT_HEAD_SERVO2_NAME      "head_2_servo"
#define ROBOT_LEFT_CLAW_NAME        "left_claw"
#define ROBOT_RIGHT_CLAW_NAME       "right_claw"
#define ROBOT_LEFT_ARM_NAME         "left_joint"
#define ROBOT_RIGHT_ARM_NAME        "right_joint"

const int LIM = 20;

using namespace std;
using namespace boost;

rgmp::Robotcontrol robot_control_rpy;
rgmp::Robotcontrol robot_teach_msg;
ros::Publisher teach_pub;

bool robot_control_rpy_flag = false;
bool robot_run_thread_flag = false;

ofstream fout;
ifstream fin;

int robot_state;  //robot buttons states now


void robot_run_function();

void robot_run_function(){
	char buf[ROBOT_FILE_BUFFER_LENGTH];
	int sleeptime;
	int i;
//the read file is ID   NAME    TIMEDEC    LENGTH    DATA[0]   DATA[1]   DATA[2]
//DATA[3]  DATA[4]   DATA[5] ........ 
	while(ros::ok()){
	//	robot_run_thread_flag = true;
		if(robot_run_thread_flag){
			//step 1 read the ID
			fin >> buf;
			robot_teach_msg.header.seq = atoi(buf);
			robot_teach_msg.header.stamp = ros::Time().now();
			robot_teach_msg.cmd = RPY_ROBOT_READ_POINT;
			//read name
			fin >> buf;
			robot_teach_msg.name = buf;
			//	ROS_INFO("the name  is %s",buf);
			//read TIMEDEC
			fin >> buf;
			sleeptime = atoi(buf);
			//	ROS_INFO("the sleep time is %d",atoi(buf));
			//read data length
			fin >> buf;
			robot_teach_msg.length = atoi(buf);
			//	ROS_INFO("the robot length is %d",atoi(buf));
			robot_teach_msg.data.resize(robot_teach_msg.length);
			for(i=0; i<robot_teach_msg.length; i++){
				fin >> buf;
			//	cout << atof(buf) << endl;
			    robot_teach_msg.data[i]	= atof(buf);
			}
			if(fin.eof()){
				ROS_INFO("read the file over!");
				fin.close();
				robot_run_thread_flag = false;
				continue;
			}
			teach_pub.publish(robot_teach_msg);
			if(sleeptime)
				sleep(sleeptime);
			//sync the speed between the net and the I/O fflush
			usleep(50000);
		}
	}
	return;
}

void teachCallback(const rgmp::Robotcontrol::ConstPtr& teachmsg){
		//	ROS_INFO("Receive the unused msg");
	int time_del;
	switch(teachmsg->cmd)
	{
		case CMD_ROBOT_TEACH_BEGIN:
			if(robot_state != ROBOT_FREE_STATE ){
				ROS_INFO("robot don't in the free mode can't teach!");
				break;
			}
		    ROS_INFO("Enter the teach mode");
			robot_control_rpy.cmd = RPY_ROBOT_TEACH_BEGIN;
			robot_state = ROBOT_TEACH_STATE;
			robot_control_rpy_flag = true;
			//TODO: create the teach file and open the file
			fout.open(teachmsg->name.c_str(), ios::out | ios::app);
			if(!fout.is_open())
			{
				cerr << "Can't open" << teachmsg->name << "file for write.\n";
				exit(EXIT_FAILURE);
			}
			//move the fout to the file end
			fout.seekp(0, ios_base::end);
		    ROS_INFO("create the file is ok!");
			break;
		case CMD_ROBOT_TEACH_END:
			if(robot_state != ROBOT_TEACH_STATE ){
				ROS_INFO("robot don't in the teach mode can't end teach!");
				break;
			}
		    ROS_INFO("End the teach mode");
			robot_control_rpy.cmd = RPY_ROBOT_TEACH_END;
			robot_state = ROBOT_FREE_STATE;
			robot_control_rpy_flag = true;
			//TODO: close the teach file and open the file
			if(fout.is_open()){
				fout.close();	
			}
			break;
		case CMD_ROBOT_ADD_POINT:
			if(robot_state != ROBOT_TEACH_STATE){
				ROS_INFO("robot don't in the teach mode can't add point!");
				break;
			}
			robot_state = ROBOT_WRBUSY_STATE;
			//write the point data
			ROS_INFO("recevie the message %s",teachmsg->name.c_str());
			int i;
			//for(i=0; i<6;i++)
			//	ROS_INFO("the joint data is [%f]",teachmsg->data[i]);
		   // ROS_INFO("the id is %d",teachmsg->length);
			ROS_INFO("add the one point");
			robot_control_rpy.cmd = RPY_ROBOT_ADD_POINT;
			robot_state = ROBOT_TEACH_STATE;
			robot_control_rpy_flag = true;
			//write the point data into the state
			//set the timeout between the each position
			if(teachmsg->name == ROBOT_RIGHT_CLAW_NAME){
				time_del = 4;
			}else{
				time_del = 0;	
			}
			fout << teachmsg->header.seq  << setw(20) << teachmsg->name << setw(5) << time_del << setw(5) << (unsigned int)teachmsg->length;	
			for(i=0; i<teachmsg->length; i++)
				fout << setprecision(6) << setw(15) <<teachmsg->data[i];
			fout << endl;
			flush(fout);
			break;
		case CMD_ROBOT_RUN_BEGIN:
			if(robot_state != ROBOT_FREE_STATE){
				ROS_INFO("robot don't in the free mode can't run!");
				break;
			}
		    ROS_INFO("Enter the run mode");
			robot_control_rpy.cmd = RPY_ROBOT_RUN_BEGIN;
			robot_state = ROBOT_RUN_STATE;
			//step1 open the txt file to run 
			fin.open(teachmsg->name.c_str(), ios::in | ios::app);
			if(!fin.is_open())
			{
				cerr << "Can't open" << teachmsg->name << "file for read.\n";
				exit(EXIT_FAILURE);
			}
			//step2 move the finger go to the begin from from the file
			fin.seekg(0, ios_base::beg);
		    ROS_INFO("open the file is ok!");
			//step2 begin the thread
			robot_run_thread_flag = true;
			robot_control_rpy_flag = true;
			break;
		case CMD_ROBOT_RUN_END:
			if(robot_state != ROBOT_RUN_STATE){
				ROS_INFO("robot don't in the run mode can't free!");
				break;
			}
		    ROS_INFO("Enter the free mode");
			robot_control_rpy.cmd = RPY_ROBOT_RUN_END;
			robot_state = ROBOT_FREE_STATE;
			robot_control_rpy_flag = true;
			//stop  the run thread about the robot
			robot_run_thread_flag = false;
			//close the file 
			if(fin.is_open()){
				fin.close();	
			}
			break;
		default: 
			ROS_INFO("Receive the unused msg");
	}
		   	
				
}

int main(int argc, char **argv){

	ros::init(argc, argv, "arm_teach");

	boost::thread robot_run_mode(boost::bind(&robot_run_function));

	ros::NodeHandle n;
	teach_pub = n.advertise<rgmp::Robotcontrol>("robot_teach_rpy", 1000);
	ros::Subscriber teach_sub = n.subscribe("robot_control_msg", 1000, teachCallback);

	//init the state
	robot_state = ROBOT_FREE_STATE;

	while(ros::ok())
	{
		if(robot_control_rpy_flag){
			robot_control_rpy.header.stamp = ros::Time().now();	
			teach_pub.publish(robot_control_rpy);
			robot_control_rpy.header.seq++;
			robot_control_rpy_flag = false;
		}
		ros::spinOnce();
	}

	return 0;
}
