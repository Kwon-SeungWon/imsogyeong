/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Kwon SeungWon (Darby) */

#ifndef TURTLEBOT3_DRIVE_H_
#define TURTLEBOT3_DRIVE_H_

#include<iostream>
#include<cmath>
#include<math.h>

#include <ros/ros.h>

#include <turtlesim/Pose.h>
#include <stdlib.h>

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Pose2D.h>

#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

class Turtlebot3Drive
{
 public:
  Turtlebot3Drive();
  ~Turtlebot3Drive();
  bool init();
  bool controlLoop();
  bool ErrorCalculate();
  bool Stop();
 private:
  // ROS NodeHandle
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;

  // ROS Topic Publishers
  ros::Publisher cmd_vel_pub_;

  // ROS Topic Subscribers
  ros::Subscriber real_odom_sub_;
  ros::Subscriber pose_sub_;

  // Variables
  double err_angle;
  double err_dist;

	const double pi = 3.141592;		//파이값
	double x1, x2, y1, y2, x3, y3;	//중점을 찾기위한 좌표
	double m1, m2;					//중점 좌표 저장
	double real_x, real_y;					//차량의 좌표 저장
	double tx, ty;					//접점의 좌표 저장
	double angle1, angle2, angle3;	//전, 지금의 기울기/ 지금, 후의 기울기 /경로 예상 기울기
	double real_angle;				//차량의 조향각
	double theory_angle;			//접점에서의 조향각
	double distance;				//차랑과 접점과의 거리

  // Function prototypes
  void updatecommandVelocity(double linear, double angular);

  void realodomMsgCallBack(const std_msgs::Float32MultiArray::ConstPtr &msg);
  void poseMsgCallBack(const geometry_msgs::Pose2D::ConstPtr &msg);
};
#endif // TURTLEBOT3_DRIVE_H_
