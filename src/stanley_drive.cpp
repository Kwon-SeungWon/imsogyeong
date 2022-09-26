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

/* Authors: Kwon SeungWon */

#include "imsogyeong_action/stanley_drive.h"

Turtlebot3Drive::Turtlebot3Drive()
  : nh_priv_("~")
{
  //Init gazebo ros turtlebot3 node
  ROS_INFO("TurtleBot3 Simulation Node Init");
  auto ret = init();
  //ROS_ASSERT(ret);
}

Turtlebot3Drive::~Turtlebot3Drive()
{
  updatecommandVelocity(0.0, 0.0);
  ros::shutdown();
}

/*******************************************************************************
* Init function
*******************************************************************************/
bool Turtlebot3Drive::init()
{
  // initialize ROS parameter
  std::string cmd_vel_topic_name = nh_.param<std::string>("cmd_vel_topic_name", "");

  // initialize variables

  tb3_pose_ = 0.0;
  prev_tb3_pose_ = 0.0;
  err_dist = 0.0;
  err_angle = 0.0;


  // initialize publishers
  cmd_vel_pub_   = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_name, 10);

  // initialize subscribers
//  odom_sub_ = nh_.subscribe("odom", 10, &Turtlebot3Drive::odomMsgCallBack, this);
  real_odom_sub_ = nh_.subscribe("/path_coord", 10, &Turtlebot3Drive::realodomMsgCallBack, this);
  pose_sub_ = nh_.subscribe("/odom2", 10, &Turtlebot3Drive::poseMsgCallBack, this);

  return true;
}
/*
void Turtlebot3Drive::odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg) // 반시계기준 0 ~ 180도 = 0 ~ +3.14 , 180도 ~ 360도 = -3.14 ~ 0
{
  double siny = 2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
  double cosy = 1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);  

  tb3_pose_ = atan2(siny, cosy);
  std::cout<< tb3_pose_ <<"\n";
}
*/
void Turtlebot3Drive::realodomMsgCallBack(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
  if(msg->data.size() > 5)
  {
    x1 = msg->data[0], y1 = msg->data[1];
    x2 = msg->data[2], y2 = msg->data[3];
    x3 = msg->data[4], y3 = msg->data[5];
  }
  if (x3 > 10 && y3 > 10) {
	updatecommandVelocity(0,0);
	std::cout << "도착!!" << "\n";
	ros::shutdown();
  }
}

void Turtlebot3Drive::poseMsgCallBack(const geometry_msgs::Pose2D::ConstPtr &msg)
{
	real_x = msg->x, real_y = msg->y;
	real_angle = msg->theta;
	
	std::cout << msg->x << " " <<  msg->y << " " << msg->theta << "\n";
	std::cout << atan2(1,0) << " " << atan2(1,1) << " " << atan2(-1,1) << " " << atan2(0,1) << "\n";
}

void Turtlebot3Drive::updatecommandVelocity(double linear, double angular)
{
  geometry_msgs::Twist cmd_vel;

  cmd_vel.linear.x  = linear;
  cmd_vel.angular.z = angular;

  cmd_vel_pub_.publish(cmd_vel);
}

bool Turtlebot3Drive::StanleyMethod()
{
	/*
	x1 = round(x1 * 100) / 100;
	y1 = round(y1 * 100) / 100;
	x2 = round(x2 * 100) / 100;
	y2 = round(y2 * 100) / 100;
	x3 = round(x3 * 100) / 100;
	y3 = round(y3 * 100) / 100;
	*/
	m1 = double(x1 + x3) / 2.0;
	m2 = double(y1 + y3) / 2.0;

	angle1 = (y2 - y1) / (x2 - x1);
	angle2 = (y3 - y2) / (x3 - x2);
	angle3 = (y3 - y1) / (x3 - x1);

	if ((((angle1 - angle2) < 0.01) && ((angle1 - angle2) > -0.01)) || ((angle2 > 1000) && (angle1 > 1000))) {
		if ((angle3 > -0.01) && (angle3 < 0.01)) {					//////////////////////////////////////////////가로 직선
			tx = x2;
			ty = y2;
			distance = y2 - real_y;
			theory_angle = 0;
			err_angle = atan2(0, 1) - real_angle;
        	err_dist = distance;
		}
		else if ((angle3 > 0.8) && (angle3 < 1.2)) {				//////////////////////////////////////////////세로 직선
			tx = (real_x + real_y - (y2 - x2)) / 2.0;
			ty = tx + (y2 - x2);
			distance = sqrt(pow(real_x - tx, 2) + pow(real_y - ty, 2));
			theory_angle = 1;
			err_angle = atan2(1, 1) - real_angle;
        	err_dist = distance;
		}
		else if ((angle3 < -0.8) && (angle3 > -1.2)) {			//////////////////////////////////////////////대각 직선(+)
			tx = (real_x - real_y + (y2 + x2)) / 2.0;
			ty = -tx + (y2 + x2);
			distance = sqrt(pow(real_x - tx, 2) + pow(real_y - ty, 2));
			theory_angle = -1;
			err_angle = atan2(-1, 1) - real_angle;
        	err_dist = distance;
		}
		else if (angle3 == INFINITY) {		//////////////////////////////////////////////대각 직선(-)
			tx = x2;
			ty = y2;
			distance = x2 - real_x;
			theory_angle = INFINITY;
			err_angle = atan2(1, 0) - real_angle;
        	err_dist = distance;
		}
		else {
			std::cout << "일치하는 경로가 없습니다." << "\n";
		}
        
		
        std::cout << "\n" << "이론상의 조향각 : " << atan(theory_angle) << " " << "실제 조향각 : " << real_angle << "\n";
		std::cout << "조향각 오차 : " << err_angle << "\n";			//라디안 단위임 '도'단위로 바꿀려면 (* 180 / pi)
		std::cout << "접점의 좌표 : " << tx << " " << ty << "\n";
		std::cout << "횡방향 오차 : " << distance << "\n";
	}

	else if (((angle3 > 0.9) && (angle3 < 1.1)) || ((angle3 < -0.9) && (angle3 > -1.1))) {
		theory_angle = -1 / ((real_y - m2) / (real_x - m1));

		if (((angle3 > 0.9) && (angle3 < 1.1)) && (y3 > y2))//////////////////////////////////////////////1번(+ -)
		{
			tx = sqrt(pow(0.1, 2) / (pow((real_y - m2) / (real_x - m1), 2) + 1)) + m1;
			ty = -sqrt(pow(0.1, 2) - (pow(0.1, 2) / (pow((real_y - m2) / (real_x - m1), 2) + 1))) + m2;
			distance = sqrt(pow(real_x - tx, 2) + pow(real_y - ty, 2));
		}
		else if (((angle3 > 0.9) && (angle3 < 1.1)) && (x3 > x2))/////////////////////////////////////////2번(- +)
		{
			tx = -sqrt(pow(0.1, 2) / (pow((real_y - m2) / (real_x - m1), 2) + 1)) + m1;
			ty = sqrt(pow(0.1, 2) - (pow(0.1, 2) / (pow((real_y - m2) / (real_x - m1), 2) + 1))) + m2;
			distance = sqrt(pow(real_x - tx, 2) + pow(real_y - ty, 2));
		}
		else if (((angle3 < -0.9) && (angle3 > -1.1)) && (x3 < x2))/////////////////////////////////////////3번(+ +)
		{
			tx = sqrt(pow(0.1, 2) / (pow((real_y - m2) / (real_x - m1), 2) + 1)) + m1;
			ty = sqrt(pow(0.1, 2) - (pow(0.1, 2) / (pow((real_y - m2) / (real_x - m1), 2) + 1))) + m2;
			distance = sqrt(pow(real_x - tx, 2) + pow(real_y - ty, 2));
		}
		else if (((angle3 < -0.9) && (angle3 > -1.1)) && (y3 > y2))/////////////////////////////////////////4번(- -)
		{
			tx = -sqrt(pow(0.1, 2) / (pow((real_y - m2) / (real_x - m1), 2) + 1)) + m1;
			ty = -sqrt(pow(0.1, 2) - (pow(0.1, 2) / (pow((real_y - m2) / (real_x - m1), 2) + 1))) + m2;
			distance = sqrt(pow(real_x - tx, 2) + pow(real_y - ty, 2));
		}
		else if (((angle3 > 0.9) && (angle3 < 1.1)) && (y3 < y2))/////////////////////////////////////////5번(- +)
		{
			tx = -sqrt(pow(0.1, 2) / (pow((real_y - m2) / (real_x - m1), 2) + 1)) + m1;
			ty = sqrt(pow(0.1, 2) - (pow(0.1, 2) / (pow((real_y - m2) / (real_x - m1), 2) + 1))) + m2;
			distance = sqrt(pow(real_x - tx, 2) + pow(real_y - ty, 2));
		}
		else if (((angle3 > 0.9) && (angle3 < 1.1)) && (x3 < x2))/////////////////////////////////////////6번(+ -)
		{
			tx = sqrt(pow(0.1, 2) / (pow((real_y - m2) / (real_x - m1), 2) + 1)) + m1;
			ty = -sqrt(pow(0.1, 2) - (pow(0.1, 2) / (pow((real_y - m2) / (real_x - m1), 2) + 1))) + m2;
			distance = sqrt(pow(real_x - tx, 2) + pow(real_y - ty, 2));
		}
		else if (((angle3 < -0.9) && (angle3 > -1.1)) && (x3 > x2))/////////////////////////////////////////7번(- -)
		{
			tx = -sqrt(pow(0.1, 2) / (pow((real_y - m2) / (real_x - m1), 2) + 1)) + m1;
			ty = -sqrt(pow(0.1, 2) - (pow(0.1, 2) / (pow((real_y - m2) / (real_x - m1), 2) + 1))) + m2;
			distance = sqrt(pow(real_x - tx, 2) + pow(real_y - ty, 2));
		}
		else if (((angle3 < -0.9) && (angle3 > -1.1)) && (y3 < y2))/////////////////////////////////////////8번(+ +)
		{
			tx = sqrt(pow(0.1, 2) / (pow((real_y - m2) / (real_x - m1), 2) + 1)) + m1;
			ty = sqrt(pow(0.1, 2) - (pow(0.1, 2) / (pow((real_y - m2) / (real_x - m1), 2) + 1))) + m2;
			distance = sqrt(pow(real_x - tx, 2) + pow(real_y - ty, 2));
		}
		else
		{
			std::cout << "일치하는 경로가 없습니다." << "\n";
		}

        err_angle = atan(theory_angle) - real_angle;
        err_dist = distance;
		
        std::cout << "\n" << "이론상의 조향각 : " << atan(theory_angle) << " " << "실제 조향각 : " << real_angle << "\n";
		std::cout << "조향각 오차 : " << err_angle << "\n";			//라디안 단위임 '도'단위로 바꿀려면 (* 180 / pi)
		std::cout << "접점의 좌표 : " << tx << " " << ty << "\n";
		std::cout << "횡방향 오차 : " << distance << "\n";
	}

	else {
	if (y1 == y2) {
		m1 = x1;
		m2 = -(x3 - x2) / (y3 - y2) * (x1 - x3) + y3;
	}
	else if (y2 == y3) {
		m1 = x3;
		m2 = -(x2 - x1) / (y2 - y1) * (x3 - x1) + y1;
	}
	else if (x1 == x2) {
		m1 = -(y3 - y2) / (x3 - x2) * (y1 - y3) + x3;
		m2 = y1;
	}
	else if (x2 == x3) {
		m1 = -(y2 - y1) / (x2 - x1) * (y3 - y1) + x1;
		m2 = y3;
	}
	std::cout << "중점 : " << m1 << " " << m2 << "\n";
	std::cout << "곡선 경로" << "\n\n";

	theory_angle = -1 / ((real_y - m2) / (real_x - m1));

	if (((angle3 > 0.4) && (angle3 < 0.6)) && (((y3 > y2) && (x1 < x2)) || ((x2 > x3) && (y1 > y2))))//////////////////////////////////////////////1번(+ -)
	{
		tx = sqrt(pow(0.6, 2) / (pow((real_y - m2) / (real_x - m1), 2) + 1)) + m1;
		ty = -sqrt(pow(0.6, 2) - (pow(0.6, 2) / (pow((real_y - m2) / (real_x - m1), 2) + 1))) + m2;
		distance = sqrt(pow(real_x - tx, 2) + pow(real_y - ty, 2));
    	//err_angle = atan(theory_angle) - real_angle;
	}
	else if (((angle3 > 1.9) && (angle3 < 2.1)) && (((x3 > x2) && (y1 < y2)) || ((y2 > y3) && (x1 > x2))))/////////////////////////////////////////2번(- +)
	{
		tx = -sqrt(pow(0.6, 2) / (pow((real_y - m2) / (real_x - m1), 2) + 1)) + m1;
		ty = sqrt(pow(0.6, 2) - (pow(0.6, 2) / (pow((real_y - m2) / (real_x - m1), 2) + 1))) + m2;
		distance = sqrt(pow(real_x - tx, 2) + pow(real_y - ty, 2));
    	//err_angle = atan2((-1)*(real_x - m1) , (real_y - m2)) - real_angle;
	}
	else if (((angle3 < -1.9) && (angle3 > -2.1)) && (((x3 < x2) && (y1 < y2)) || ((y2 > y3) && (x1 < x2))))/////////////////////////////////////////3번(+ +)
	{
		tx = sqrt(pow(0.6, 2) / (pow((real_y - m2) / (real_x - m1), 2) + 1)) + m1;
		ty = sqrt(pow(0.6, 2) - (pow(0.6, 2) / (pow((real_y - m2) / (real_x - m1), 2) + 1))) + m2;
		distance = sqrt(pow(real_x - tx, 2) + pow(real_y - ty, 2));
    	//err_angle = atan2((-1)*(real_x - m1) , (real_y - m2)) - real_angle;
	}
	else if (((angle3 < -0.4) && (angle3 > -0.6)) && (((y3 > y2) && (x1 > x2)) || ((x2 < x3) && (y1 > y2))))/////////////////////////////////////////4번(- -)
	{
		tx = -sqrt(pow(0.6, 2) / (pow((real_y - m2) / (real_x - m1), 2) + 1)) + m1;
		ty = -sqrt(pow(0.6, 2) - (pow(0.6, 2) / (pow((real_y - m2) / (real_x - m1), 2) + 1))) + m2;
		distance = sqrt(pow(real_x - tx, 2) + pow(real_y - ty, 2));
    	//err_angle = atan2((-1)*(real_x - m1) , (real_y - m2)) - real_angle;
	}
	else if (((angle3 > 0.4) && (angle3 < 0.6)) && (((y3 < y2) && (x1 > x2)) || ((x2 < x3) && (y1 < y2))))/////////////////////////////////////////5번(- +)
	{
		tx = -sqrt(pow(0.6, 2) / (pow((real_y - m2) / (real_x - m1), 2) + 1)) + m1;
		ty = sqrt(pow(0.6, 2) - (pow(0.6, 2) / (pow((real_y - m2) / (real_x - m1), 2) + 1))) + m2;
		distance = sqrt(pow(real_x - tx, 2) + pow(real_y - ty, 2));
    	//err_angle = atan2((-1)*(real_x - m1) , (real_y - m2)) - real_angle;
	}
	else if (((angle3 > 1.9) && (angle3 < 2.1)) && (((x3 < x2) && (y1 > y2)) || ((y2 < y3) && (x1 < x2))))/////////////////////////////////////////6번(+ -)
	{
		tx = sqrt(pow(0.6, 2) / (pow((real_y - m2) / (real_x - m1), 2) + 1)) + m1;
		ty = -sqrt(pow(0.6, 2) - (pow(0.6, 2) / (pow((real_y - m2) / (real_x - m1), 2) + 1))) + m2;
		distance = sqrt(pow(real_x - tx, 2) + pow(real_y - ty, 2));
    	//err_angle = atan2((-1)*(real_y - m2) , (real_x - m1)) - real_angle;
	}
	else if (((angle3 < -1.9) && (angle3 > -2.1)) && (((x3 > x2) && (y1 > y2)) || ((y2 < y3) && (x1 > x2))))/////////////////////////////////////////7번(- -)
	{
		tx = -sqrt(pow(0.6, 2) / (pow((real_y - m2) / (real_x - m1), 2) + 1)) + m1;
		ty = -sqrt(pow(0.6, 2) - (pow(0.6, 2) / (pow((real_y - m2) / (real_x - m1), 2) + 1))) + m2;
		distance = sqrt(pow(real_x - tx, 2) + pow(real_y - ty, 2));
    	//err_angle = atan2((-1)*(real_x - m1) , (real_y - m2)) - real_angle;
	}
	else if (((angle3 < -0.4) && (angle3 > -0.6)) && (((y3 < y2) && (x1 < x2)) || ((x2 > x3) && (y1 < y2))))/////////////////////////////////////////8번(+ +)
	{
		tx = sqrt(pow(0.6, 2) / (pow((real_y - m2) / (real_x - m1), 2) + 1)) + m1;
		ty = sqrt(pow(0.6, 2) - (pow(0.6, 2) / (pow((real_y - m2) / (real_x - m1), 2) + 1))) + m2;
		distance = sqrt(pow(real_x - tx, 2) + pow(real_y - ty, 2));
    	//err_angle = atan2((-1)*(real_x - m1) , (real_y - m2)) - real_angle;
	}
	else
	{
		std::cout << "일치하는 경로가 없습니다." << "\n";
	}
    err_angle = atan(theory_angle) - real_angle;
    //err_angle = atan2((-1)*(real_x - m1) , (real_y - m2)) - real_angle;
    err_dist = distance;
	
  std::cout << "\n" << "이론상의 조향각 : " << atan(theory_angle) << " " << "실제 조향각 : " << real_angle << "\n";
	std::cout << "조향각 오차 : " << err_angle << "\n";			//라디안 단위임 '도'단위로 바꿀려면 (* 180 / pi)
	std::cout << "접점의 좌표 : " << tx << " " << ty << "\n";
	std::cout << "횡방향 오차 : " << distance << "\n";
  }

	return true;
}
/*******************************************************************************
* Control Loop function
*******************************************************************************/

bool Turtlebot3Drive::controlLoop()
{
 // static uint8_t turtlebot3_state_num = 0;
 // if((err_dist > 0.0 && err_dist < 0.02) && (err_angle > 0.0 && err_angle < 0.02)) updatecommandVelocity(0.3, 0.0); 	
  /*
  vector <double> vec(2);
  vec = [cos( + pi/2), sin(real_angle + pi/2)];
  cte1 = ([dx,dy] * vec());
  cte2 = sqrt(pow(dx,2)+pow(dy,2));
  if real_y < y2{
	cte2 *= -1;
  }
  	
  
  */
  if(err_angle > 0.7 || err_angle < -0.7 )
  {
	updatecommandVelocity(0.08, -(0.4*err_angle));
  }
  else
  {
	updatecommandVelocity(0.08, -(0.35*err_angle));
  }
  return true;
}

bool Turtlebot3Drive::Stop()
{
  int system (const char * string);
  //int quit1 = system("rosnode kill turtlebot3_drive");
  //int quit2 = system("reset");
  updatecommandVelocity(0.0, 0.0);
  ros::shutdown();
  return 0;
}

/*******************************************************************************
* Main function
*******************************************************************************/
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "turtlebot3_drive");
  Turtlebot3Drive turtlebot3_drive;

  ros::Rate loop_rate(10);
  int cnt = 0;
  while (ros::ok())
  {
	cnt = cnt + 1;
	if(cnt > 50){
		bool check = false;
    	check = turtlebot3_drive.StanleyMethod();
    	check = turtlebot3_drive.controlLoop();
	}

	// if (x3 > 10 && y3 > 10) {
    //    turtlebot3_drive.Stop();
    // }
	ros::spinOnce();
    loop_rate.sleep();
	
  }

  return 0;
}
