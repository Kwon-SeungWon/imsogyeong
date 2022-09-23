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

#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

#define CENTER 0
#define LEFT   1
#define RIGHT  2

#define LINEAR_VELOCITY  0.3
#define ANGULAR_VELOCITY 1.5

#define GET_TB3_DIRECTION 0
#define TB3_DRIVE_FORWARD 1
#define TB3_RIGHT_TURN    2
#define TB3_LEFT_TURN     3

class Turtlebot3Drive
{
 public:
  Turtlebot3Drive();
  ~Turtlebot3Drive();
  bool init();
  bool controlLoop();
  bool StanleyMethod();
  bool Stop();
 private:
  // ROS NodeHandle
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;

  // ROS Parameters

  // ROS Time

  // ROS Topic Publishers
  ros::Publisher cmd_vel_pub_;

  // ROS Topic Subscribers
  ros::Subscriber laser_scan_sub_;
  ros::Subscriber odom_sub_;
//  ros::Subscriber pose_sub_;

  // Variables
  char q;
  double escape_range_;
  double check_forward_dist_;
  double check_side_dist_;

  double scan_data_[3] = {0.0, 0.0, 0.0};

  double tb3_pose_;
  double prev_tb3_pose_;
  double err_angle;
  double err_dist;

	const double pi = 3.141592;		//파이값
	double x1, x2, y1, y2, x3, y3;	//중점을 찾기위한 좌표
	double m1, m2;					//중점 좌표 저장
	double x, y;					//차량의 좌표 저장
	double tx, ty;					//접점의 좌표 저장
	double angle1, angle2, angle3;	//전, 지금의 기울기/ 지금, 후의 기울기 /경로 예상 기울기
	double real_angle;				//차량의 조향각
	double theory_angle;			//접점에서의 조향각
	double distance;				//차랑과 접점과의 거리

  // Function prototypes
  void updatecommandVelocity(double linear, double angular);
  void laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg);
  void odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg);
//  void Stop(int N);
};
#endif // TURTLEBOT3_DRIVE_H_
