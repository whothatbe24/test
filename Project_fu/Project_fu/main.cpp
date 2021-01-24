/***************************************************************************************************************************
 * writing.cpp
 *
 * Author: jiao
 *
 * Update Time: 2020.8.11
 *
 * ˵��: UAVд�� д������
 *      1.
 *      2.
 *      3.
***************************************************************************************************************************/

#include <ros/ros.h>
#include <fstream>
#include <math.h>
#include <string>
#include <time.h>
#include <queue>
#include <vector>
#include <cstdlib>
#include <stdlib.h>
#include <iostream>
#include <stdio.h>
#include <std_msgs/Bool.h>
#include <px4_command/command.h>


using namespace std;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>ȫ �� �� ��<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
enum Command
{
	Idle,
	Takeoff,
	Move_ENU,
	Move_Body,
	Hold,
	Land,
	Disarm,
	Failsafe_land,
};
px4_command::command Command_now;
//---------------------------------------���в���---------------------------------------------
float height;                //���и߶�
float height_d;                            //�߶Ȳ�
float sleep_time;
int main(int argc, char **argv)
{
	ros::init(argc, argv, "writing");
	ros::NodeHandle nh("~");

	// Ƶ�� [10hz]
	ros::Rate rate(10.0);
	ros::Rate rate_second(1.0);

	// �����������͸�position_control.cpp������
	ros::Publisher move_pub = nh.advertise<px4_command::command>("/px4/command", 10);

	//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>������ȡ<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
	nh.param<float>("height", height, 1.0);
	nh.param<float>("height_d", height_d, 0.5);
	nh.param<float>("sleep_time", sleep_time, 3.0);



	int check_flag;
	// ��һ����Ϊ�˳�������ǰ���һ�²����Ƿ���ȷ
	// ����1,�������������˳�����
	cout << "height: " << height << "[m]" << endl;
	cout << "height_d: " << height_d << "[m]" << endl;
	cout << "Please check the parameter and setting��1 for go on�� else for quit: " << endl;
	cin >> check_flag;

	if (check_flag != 1)
	{
		return -1;
	}


	int i = 0;
	int comid = 0;
	int stroke_id = 0;       // �ʻ���ID
	int point_id = 0;       // Ŀ����ID
	int point_step = 20;   // ÿ���ʻ���������

	int sum_stroke = 29;   // ����Ҫ���й켣������ ���鳤��-1
	float Fu[30][3] = { {-0.3, 0.7, height}, {-0.3, 0.5, height}, {-0.5, 0.4, height}, {-0.1, 0.4, height}, {-0.5, 0.0, height},
					   {-0.3, 0.2, height}, {-0.3, -0.6, height}, {-0.3, 0.2, height}, {0.0, -0.1, height}, {0.0, -0.1, height + height_d},
					   {0.1, 0.7, height}, {0.5, 0.7, height}, {0.5, 0.7, height + height_d}, {0.1, 0.5, height}, {0.1, 0.1, height},
					   {0.5, 0.1, height}, {0.5, 0.5, height}, {0.1, 0.5, height}, {0.1, 0.5, height + height_d}, {0.0, 0.0, height},
					   {0.0, -0.6, height}, {0.6, -0.6, height}, {0.6, 0.0, height}, {0.0, 0.0, height}, {0.0, -0.3, height},
					   {0.6, -0.3, height}, {0.6, -0.3, height + height_d}, {0.3, 0.0, height}, {0.3, -0.6, height}, {0.3, -0.6, height + height_d} };    // ���ָ��ʻ�������յ�

	//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>������<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
	//takeoff
	while (i < sleep_time)
	{

		Command_now.command = Move_ENU;       //Moveģʽ
		Command_now.sub_mode = 0;             //��ģʽ��λ�ÿ���ģʽ
		Command_now.pos_sp[0] = 0;
		Command_now.pos_sp[1] = 0;
		Command_now.pos_sp[2] = height;
		Command_now.yaw_sp = 0;
		Command_now.comid = comid;
		comid++;

		move_pub.publish(Command_now);

		rate_second.sleep();

		i++;

	}

	// �ɵ�д�ֵ����
	i = 0;
	while (i < sleep_time)
	{

		Command_now.command = Move_ENU;       //Moveģʽ
		Command_now.sub_mode = 0;             //��ģʽ��λ�ÿ���ģʽ
		Command_now.pos_sp[0] = Fu[0][0];
		Command_now.pos_sp[1] = Fu[0][1];
		Command_now.pos_sp[2] = Fu[0][2];
		Command_now.yaw_sp = 0;
		Command_now.comid = comid;
		comid++;

		move_pub.publish(Command_now);

		rate_second.sleep();
		i++;

	}

	//���η��ͺ���Ŀ����position_control.cpp
	while (stroke_id < sum_stroke)
	{
		stroke_id++;
		while (point_id <= point_step)
		{

			Command_now.command = Move_ENU;  //Moveģʽ
			Command_now.sub_mode = 0;             //��ģʽ��λ�ÿ���ģʽ
			Command_now.pos_sp[0] = Fu[stroke_id - 1][0] + point_id * (Fu[stroke_id][0] - Fu[stroke_id - 1][0]) / point_step;
			Command_now.pos_sp[1] = Fu[stroke_id - 1][1] + point_id * (Fu[stroke_id][1] - Fu[stroke_id - 1][1]) / point_step;
			Command_now.pos_sp[2] = Fu[stroke_id - 1][2] + point_id * (Fu[stroke_id][2] - Fu[stroke_id - 1][2]) / point_step;
			Command_now.yaw_sp = 0;
			Command_now.comid = comid;
			comid++;

			move_pub.publish(Command_now);

			rate.sleep();

			cout << "stroke" << stroke_id << "  Point " << point_id << endl;
			point_id++;
		}
		point_id = 0;
	}


	//����

	Command_now.command = Land;
	move_pub.publish(Command_now);

	rate_second.sleep();

	cout << "Land" << endl;





	return 0;
}