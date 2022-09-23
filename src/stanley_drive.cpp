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
  ROS_ASSERT(ret);
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
  escape_range_       = 30.0 * DEG2RAD;
  check_forward_dist_ = 0.4;
  check_side_dist_    = 0.3;

  tb3_pose_ = 0.0;
  prev_tb3_pose_ = 0.0;

  // initialize publishers
  cmd_vel_pub_   = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_name, 10);

  // initialize subscribers
  laser_scan_sub_  = nh_.subscribe("scan", 10, &Turtlebot3Drive::laserScanMsgCallBack, this);
  odom_sub_ = nh_.subscribe("odom", 10, &Turtlebot3Drive::odomMsgCallBack, this);

  return true;
}

void Turtlebot3Drive::odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg) // 반시계기준 0 ~ 180도 = 0 ~ +3.14 , 180도 ~ 360도 = -3.14 ~ 0
{
  double siny = 2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
	double cosy = 1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);  

	tb3_pose_ = atan2(siny, cosy);
  std::cout<< tb3_pose_ <<"\n";
}

//void Turtlebot3Drive::poseMsgCallBack(const )

void Turtlebot3Drive::laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg)
{
  uint16_t scan_angle[3] = {0, 30, 330};

  for (int num = 0; num < 3; num++)
  {
    if (std::isinf(msg->ranges.at(scan_angle[num])))
    {
      scan_data_[num] = msg->range_max;
    }
    else
    {
      scan_data_[num] = msg->ranges.at(scan_angle[num]);
    }
  }
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
  std::cout << "전 좌표 입력 : ";
	std::cin >> x1 >> y1;
	std::cout << "지금 좌표 입력 : ";
	std::cin >> x2 >> y2;
	std::cout << "후 좌표 입력 : ";
	std::cin >> x3 >> y3;

	m1 = double(x1 + x3) / 2.0;
	m2 = double(y1 + y3) / 2.0;

	angle1 = (y2 - y1) / (x2 - x1);
	angle2 = (y3 - y2) / (x3 - x2);
	angle3 = (y3 - y1) / (x3 - x1);

	if (angle1 == angle2) {
		std::cout << "\n" << "직선 경로" << "\n\n";
		std::cout << "차량의 좌표 : ";
		std::cin >> x >> y;
		std::cout << "차량의 방향 : ";
		std::cin >> real_angle;

		if (angle3 == 0) {					//////////////////////////////////////////////가로 직선
			tx = x2;
			ty = y2;
			distance = y2 - y;
			theory_angle = 0;
		}
		else if (angle3 == 1) {				//////////////////////////////////////////////세로 직선
			tx = (x + y - (y2 - x2)) / 2.0;
			ty = tx + (y2 - x2);
			distance = sqrt(pow(x - tx, 2) + pow(y - ty, 2));
			theory_angle = 1;
		}
		else if (angle3 == -1) {			//////////////////////////////////////////////대각 직선(+)
			tx = (x - y + (y2 + x2)) / 2.0;
			ty = -tx + (y2 + x2);
			distance = sqrt(pow(x - tx, 2) + pow(y - ty, 2));
			theory_angle = -1;
		}
		else if (angle3 == INFINITY) {		//////////////////////////////////////////////대각 직선(-)
			tx = x2;
			ty = y2;
			distance = x2 - x;
			theory_angle = INFINITY;
		}
		else {
			std::cout << "일치하는 경로가 없습니다." << "\n";
		}
        
        err_angle = atan(theory_angle) - atan(real_angle);
        err_dist = distance;
		
        std::cout << "\n" << "이론상의 조향각 : " << atan(theory_angle) << " " << "실제 조향각 : " << atan(real_angle) << "\n";
		std::cout << "조향각 오차 : " << atan(theory_angle) - atan(real_angle) << "\n";			//라디안 단위임 '도'단위로 바꿀려면 (* 180 / pi)
		std::cout << "접점의 좌표 : " << tx << " " << ty << "\n";
		std::cout << "횡방향 오차 : " << distance << "\n";
	}

	else if (angle3 == 1 || angle3 == -1) {
		std::cout << "중점 : " << m1 << " " << m2 << "\n";
		std::cout << "곡선 경로" << "\n\n";

		std::cout << "차량의 좌표 : ";
		std::cin >> x >> y;
		std::cout << "차량의 방향 : ";
		std::cin >> real_angle;

		theory_angle = -1 / ((y - m2) / (x - m1));

		if (angle3 == 1 && (y3 > y2))//////////////////////////////////////////////1번(+ -)
		{
			tx = sqrt(pow(0.5, 2) / (pow((y - m2) / (x - m1), 2) + 1)) + m1;
			ty = -sqrt(pow(0.5, 2) - (pow(0.5, 2) / (pow((y - m2) / (x - m1), 2) + 1))) + m2;
			distance = sqrt(pow(x - tx, 2) + pow(y - ty, 2));
		}
		else if (angle3 == 1 && (x3 > x2))/////////////////////////////////////////2번(- +)
		{
			tx = -sqrt(pow(0.5, 2) / (pow((y - m2) / (x - m1), 2) + 1)) + m1;
			ty = sqrt(pow(0.5, 2) - (pow(0.5, 2) / (pow((y - m2) / (x - m1), 2) + 1))) + m2;
			distance = sqrt(pow(x - tx, 2) + pow(y - ty, 2));
		}
		else if (angle3 == -1 && (x3 < x2))/////////////////////////////////////////3번(+ +)
		{
			tx = sqrt(pow(0.5, 2) / (pow((y - m2) / (x - m1), 2) + 1)) + m1;
			ty = sqrt(pow(0.5, 2) - (pow(0.5, 2) / (pow((y - m2) / (x - m1), 2) + 1))) + m2;
			distance = sqrt(pow(x - tx, 2) + pow(y - ty, 2));
		}
		else if (angle3 == -1 && (y3 > y2))/////////////////////////////////////////4번(- -)
		{
			tx = -sqrt(pow(0.5, 2) / (pow((y - m2) / (x - m1), 2) + 1)) + m1;
			ty = -sqrt(pow(0.5, 2) - (pow(0.5, 2) / (pow((y - m2) / (x - m1), 2) + 1))) + m2;
			distance = sqrt(pow(x - tx, 2) + pow(y - ty, 2));
		}
		else if (angle3 == 1 && (y3 < y2))/////////////////////////////////////////5번(- +)
		{
			tx = -sqrt(pow(0.5, 2) / (pow((y - m2) / (x - m1), 2) + 1)) + m1;
			ty = sqrt(pow(0.5, 2) - (pow(0.5, 2) / (pow((y - m2) / (x - m1), 2) + 1))) + m2;
			distance = sqrt(pow(x - tx, 2) + pow(y - ty, 2));
		}
		else if (angle3 == 1 && (x3 < x2))/////////////////////////////////////////6번(+ -)
		{
			tx = sqrt(pow(0.5, 2) / (pow((y - m2) / (x - m1), 2) + 1)) + m1;
			ty = -sqrt(pow(0.5, 2) - (pow(0.5, 2) / (pow((y - m2) / (x - m1), 2) + 1))) + m2;
			distance = sqrt(pow(x - tx, 2) + pow(y - ty, 2));
		}
		else if (angle3 == -1 && (x3 > x2))/////////////////////////////////////////7번(- -)
		{
			tx = -sqrt(pow(0.5, 2) / (pow((y - m2) / (x - m1), 2) + 1)) + m1;
			ty = -sqrt(pow(0.5, 2) - (pow(0.5, 2) / (pow((y - m2) / (x - m1), 2) + 1))) + m2;
			distance = sqrt(pow(x - tx, 2) + pow(y - ty, 2));
		}
		else if (angle3 == -1 && (y3 < y2))/////////////////////////////////////////8번(+ +)
		{
			tx = sqrt(pow(0.5, 2) / (pow((y - m2) / (x - m1), 2) + 1)) + m1;
			ty = sqrt(pow(0.5, 2) - (pow(0.5, 2) / (pow((y - m2) / (x - m1), 2) + 1))) + m2;
			distance = sqrt(pow(x - tx, 2) + pow(y - ty, 2));
		}
		else
		{
			std::cout << "일치하는 경로가 없습니다." << "\n";
		}

        err_angle = atan(theory_angle) - atan(real_angle);
        err_dist = distance;
		
        std::cout << "\n" << "이론상의 조향각 : " << atan(theory_angle) << " " << "실제 조향각 : " << atan(real_angle) << "\n";
		std::cout << "조향각 오차 : " << atan(theory_angle) - atan(real_angle) << "\n";			//라디안 단위임 '도'단위로 바꿀려면 (* 180 / pi)
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

	std::cout << "차량의 좌표 : ";
	std::cin >> x >> y;
	std::cout << "차량의 방향 : ";
	std::cin >> real_angle;

	theory_angle = -1 / ((y - m2) / (x - m1));

	if (angle3 == 0.5 && (((y3 > y2) && (x1 < x2)) || ((x2 > x3) && (y1 > y2))))//////////////////////////////////////////////1번(+ -)
	{
		tx = sqrt(pow(3, 2) / (pow((y - m2) / (x - m1), 2) + 1)) + m1;
		ty = -sqrt(pow(3, 2) - (pow(3, 2) / (pow((y - m2) / (x - m1), 2) + 1))) + m2;
		distance = sqrt(pow(x - tx, 2) + pow(y - ty, 2));
	}
	else if (angle3 == 2 && (((x3 > x2) && (y1 < y2)) || ((y2 > y3) && (x1 > x2))))/////////////////////////////////////////2번(- +)
	{
		tx = -sqrt(pow(3, 2) / (pow((y - m2) / (x - m1), 2) + 1)) + m1;
		ty = sqrt(pow(3, 2) - (pow(3, 2) / (pow((y - m2) / (x - m1), 2) + 1))) + m2;
		distance = sqrt(pow(x - tx, 2) + pow(y - ty, 2));
	}
	else if (angle3 == -2 && (((x3 < x2) && (y1 < y2)) || ((y2 > y3) && (x1 < x2))))/////////////////////////////////////////3번(+ +)
	{
		tx = sqrt(pow(3, 2) / (pow((y - m2) / (x - m1), 2) + 1)) + m1;
		ty = sqrt(pow(3, 2) - (pow(3, 2) / (pow((y - m2) / (x - m1), 2) + 1))) + m2;
		distance = sqrt(pow(x - tx, 2) + pow(y - ty, 2));
	}
	else if (angle3 == -0.5 && (((y3 > y2) && (x1 > x2)) || ((x2 < x3) && (y1 > y2))))/////////////////////////////////////////4번(- -)
	{
		tx = -sqrt(pow(3, 2) / (pow((y - m2) / (x - m1), 2) + 1)) + m1;
		ty = -sqrt(pow(3, 2) - (pow(3, 2) / (pow((y - m2) / (x - m1), 2) + 1))) + m2;
		distance = sqrt(pow(x - tx, 2) + pow(y - ty, 2));
	}
	else if (angle3 == 0.5 && (((y3 < y2) && (x1 > x2)) || ((x2 < x3) && (y1 < y2))))/////////////////////////////////////////5번(- +)
	{
		tx = -sqrt(pow(3, 2) / (pow((y - m2) / (x - m1), 2) + 1)) + m1;
		ty = sqrt(pow(3, 2) - (pow(3, 2) / (pow((y - m2) / (x - m1), 2) + 1))) + m2;
		distance = sqrt(pow(x - tx, 2) + pow(y - ty, 2));
	}
	else if (angle3 == 2 && (((x3 < x2) && (y1 > y2)) || ((y2 < y3) && (x1 < x2))))/////////////////////////////////////////6번(+ -)
	{
		tx = sqrt(pow(3, 2) / (pow((y - m2) / (x - m1), 2) + 1)) + m1;
		ty = -sqrt(pow(3, 2) - (pow(3, 2) / (pow((y - m2) / (x - m1), 2) + 1))) + m2;
		distance = sqrt(pow(x - tx, 2) + pow(y - ty, 2));
	}
	else if (angle3 == -2 && (((x3 > x2) && (y1 > y2)) || ((y2 < y3) && (x1 > x2))))/////////////////////////////////////////7번(- -)
	{
		tx = -sqrt(pow(3, 2) / (pow((y - m2) / (x - m1), 2) + 1)) + m1;
		ty = -sqrt(pow(3, 2) - (pow(3, 2) / (pow((y - m2) / (x - m1), 2) + 1))) + m2;
		distance = sqrt(pow(x - tx, 2) + pow(y - ty, 2));
	}
	else if (angle3 == -0.5 && (((y3 < y2) && (x1 < x2)) || ((x2 > x3) && (y1 < y2))))/////////////////////////////////////////8번(+ +)
	{
		tx = sqrt(pow(3, 2) / (pow((y - m2) / (x - m1), 2) + 1)) + m1;
		ty = sqrt(pow(3, 2) - (pow(3, 2) / (pow((y - m2) / (x - m1), 2) + 1))) + m2;
		distance = sqrt(pow(x - tx, 2) + pow(y - ty, 2));
	}
	else
	{
		std::cout << "일치하는 경로가 없습니다." << "\n";
	}
    
    err_angle = atan(theory_angle) - atan(real_angle);
    err_dist = distance;
	
  std::cout << "\n" << "이론상의 조향각 : " << atan(theory_angle) << " " << "실제 조향각 : " << atan(real_angle) << "\n";
	std::cout << "조향각 오차 : " << atan(theory_angle) - atan(real_angle) << "\n";			//라디안 단위임 '도'단위로 바꿀려면 (* 180 / pi)
	std::cout << "접점의 좌표 : " << tx << " " << ty << "\n";
	std::cout << "횡방향 오차 : " << distance << "\n";
  }

	return true;
}
/*******************************************************************************
* Control Loop function
*******************************************************************************/
/* bool Turtlebot3Drive::controlLoop()
{
  static uint8_t turtlebot3_state_num = 0;

  switch(turtlebot3_state_num)
  {
    case GET_TB3_DIRECTION:
      if (scan_data_[CENTER] > check_forward_dist_)
      {
        if (scan_data_[LEFT] < check_side_dist_)
        {
          prev_tb3_pose_ = tb3_pose_;
          turtlebot3_state_num = TB3_RIGHT_TURN;
        }
        else if (scan_data_[RIGHT] < check_side_dist_)
        {
          prev_tb3_pose_ = tb3_pose_;
          turtlebot3_state_num = TB3_LEFT_TURN;
        }
        else
        {
          turtlebot3_state_num = TB3_DRIVE_FORWARD;
        }
      }

      if (scan_data_[CENTER] < check_forward_dist_)
      {
        prev_tb3_pose_ = tb3_pose_;
        turtlebot3_state_num = TB3_RIGHT_TURN;
      }
      break;

    case TB3_DRIVE_FORWARD:
      updatecommandVelocity(LINEAR_VELOCITY, 0.0);
      turtlebot3_state_num = GET_TB3_DIRECTION;
      break;

    case TB3_RIGHT_TURN:
      if (fabs(prev_tb3_pose_ - tb3_pose_) >= escape_range_)
        turtlebot3_state_num = GET_TB3_DIRECTION;
      else
        updatecommandVelocity(0.0 , -1.0 * ANGULAR_VELOCITY);
      break;

    case TB3_LEFT_TURN:
      if (fabs(prev_tb3_pose_ - tb3_pose_) >= escape_range_)
        turtlebot3_state_num = GET_TB3_DIRECTION;
      else
        updatecommandVelocity(0.0, 1.0 * ANGULAR_VELOCITY);
      break;

    default:
      turtlebot3_state_num = GET_TB3_DIRECTION;
      break;
  }

  return true;
}

*/

bool Turtlebot3Drive::controlLoop()
{
  static uint8_t turtlebot3_state_num = 0;
  updatecommandVelocity(err_dist, err_angle);
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

  while (ros::ok())
  {
    turtlebot3_drive.StanleyMethod();
    turtlebot3_drive.controlLoop();
    ros::spinOnce();
    loop_rate.sleep();
    if (getchar() == 'q') {
      turtlebot3_drive.Stop();
      break;
    }
  }

  return 0;
}
