#include "imsogyeong_action/stanley_drive.h"

Turtlebot3Drive::Turtlebot3Drive()
  : nh_priv_("~")
{
  //Init gazebo ros turtlebot3 node
  ROS_INFO("TurtleBot3 Simulation Node Init");
  auto ret = init();
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
  err_dist = 0.0;
  err_angle = 0.0;

  // initialize publishers
  cmd_vel_pub_   = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_name, 10);

  // initialize subscribers
  real_odom_sub_ = nh_.subscribe("/path_coord", 10, &Turtlebot3Drive::realodomMsgCallBack, this);
  pose_sub_ = nh_.subscribe("/odom2", 10, &Turtlebot3Drive::poseMsgCallBack, this);

  return true;
}

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
}

void Turtlebot3Drive::updatecommandVelocity(double linear, double angular)
{
  geometry_msgs::Twist cmd_vel;

  cmd_vel.linear.x  = linear;
  cmd_vel.angular.z = angular;

  cmd_vel_pub_.publish(cmd_vel);
}

bool Turtlebot3Drive::ErrorCalculate()
{
	///////////////////////////////받아온 과거, 현재, 미래 좌표를 통해 각도를 계산한다.
	angle1 = (y2 - y1) / (x2 - x1);		//과거와 현재 좌표에 대한 각도
	angle2 = (y3 - y2) / (x3 - x2);		//현재와 미래 좌표에 대한 각도
	angle3 = (y3 - y1) / (x3 - x1);		//과거와 미래 좌표에 대한 각도

	if ((((angle1 - angle2) < 0.01) && ((angle1 - angle2) > -0.01)) || ((angle2 > 1000) && (angle1 > 1000))) {		//직선 경로인 경우 
		if ((angle3 > -0.01) && (angle3 < 0.01)) {					//x축과 평행한 경로
			tx = x2;
			ty = y2;
			distance = y2 - real_y;
			theory_angle = 0;
			err_angle = atan2(0, 1) - real_angle;
			err_dist = distance;
		}
		else if ((angle3 > 0.8) && (angle3 < 1.2)) {				//x, y 기준 둘다 +로 뻗어나가는 대각선 경로(y = x)
			tx = (real_x + real_y - (y2 - x2)) / 2.0;
			ty = tx + (y2 - x2);
			distance = sqrt(pow(real_x - tx, 2) + pow(real_y - ty, 2));
			theory_angle = 1;
			err_angle = atan2(1, 1) - real_angle;
			err_dist = distance;
		}
		else if ((angle3 < -0.8) && (angle3 > -1.2)) {				//x, y 기준 x만 +로 뻗어나가는 대각선 경로(y = -x)
			tx = (real_x - real_y + (y2 + x2)) / 2.0;
			ty = -tx + (y2 + x2);
			distance = sqrt(pow(real_x - tx, 2) + pow(real_y - ty, 2));
			theory_angle = -1;
			err_angle = atan2(-1, 1) - real_angle;
			err_dist = distance;
		}
		else if (angle3 == INFINITY) {								//y축과 평행한 경로
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

		//로봇의 좌표와 방향을 직접적으로 받음으로써 이론적으로 있어야하는 위치와 보고있는 방향을 비교해서 오차를 구한다.
		std::cout << "\n" << "이론상의 조향각 : " << atan(theory_angle) << " " << "실제 조향각 : " << real_angle << "\n";
		std::cout << "조향각 오차 : " << err_angle << "\n";				
		std::cout << "접점의 좌표 : " << tx << " " << ty << "\n";
		std::cout << "횡방향 오차 : " << distance << "\n";
	}

	else if (((angle3 > 0.9) && (angle3 < 1.1)) || ((angle3 < -0.9) && (angle3 > -1.1))) {			//좌표가 ㄱ형태로 주어진 곡선 경로인 경우

		m1 = (x1 + x3) / 2.0;		//곡선경로를 만들기 위한 원의 방정식의 중점(x)
		m2 = (y1 + y3) / 2.0;		//곡선경로를 만들기 위한 원의 방정식의 중점(y)

		std::cout << "중점 : " << m1 << " " << m2 << "\n";
		std::cout << "곡선 경로" << "\n\n";

		theory_angle = -1 / ((real_y - m2) / (real_x - m1));		//차량의 좌표와 원의 중점을 이용하여 움직여야하는 곡선 경로에서의 조향각을 구한다.

		if (((angle3 > 0.9) && (angle3 < 1.1)) && (y3 > y2))		//////////////////////////////////////////////1번(+ -)
		{
			tx = sqrt(pow(0.1, 2) / (pow((real_y - m2) / (real_x - m1), 2) + 1)) + m1;							//생성되는 원의 방정식의 반지름은 지정한 구역에서 10cm, 즉 0.1m로 고정이다.
			ty = -sqrt(pow(0.1, 2) - (pow(0.1, 2) / (pow((real_y - m2) / (real_x - m1), 2) + 1))) + m2;			//위에서 구한 tx와 반지름를 이용하여 원의 방정식에 대입해 ty를 구한다.
			distance = sqrt(pow(real_x - tx, 2) + pow(real_y - ty, 2));
		}
		else if (((angle3 > 0.9) && (angle3 < 1.1)) && (x3 > x2))	/////////////////////////////////////////2번(- +)
		{
			tx = -sqrt(pow(0.1, 2) / (pow((real_y - m2) / (real_x - m1), 2) + 1)) + m1;
			ty = sqrt(pow(0.1, 2) - (pow(0.1, 2) / (pow((real_y - m2) / (real_x - m1), 2) + 1))) + m2;
			distance = sqrt(pow(real_x - tx, 2) + pow(real_y - ty, 2));
		}
		else if (((angle3 < -0.9) && (angle3 > -1.1)) && (x3 < x2))	/////////////////////////////////////////3번(+ +)
		{
			tx = sqrt(pow(0.1, 2) / (pow((real_y - m2) / (real_x - m1), 2) + 1)) + m1;
			ty = sqrt(pow(0.1, 2) - (pow(0.1, 2) / (pow((real_y - m2) / (real_x - m1), 2) + 1))) + m2;
			distance = sqrt(pow(real_x - tx, 2) + pow(real_y - ty, 2));
		}
		else if (((angle3 < -0.9) && (angle3 > -1.1)) && (y3 > y2))	/////////////////////////////////////////4번(- -)
		{
			tx = -sqrt(pow(0.1, 2) / (pow((real_y - m2) / (real_x - m1), 2) + 1)) + m1;
			ty = -sqrt(pow(0.1, 2) - (pow(0.1, 2) / (pow((real_y - m2) / (real_x - m1), 2) + 1))) + m2;
			distance = sqrt(pow(real_x - tx, 2) + pow(real_y - ty, 2));
		}
		else if (((angle3 > 0.9) && (angle3 < 1.1)) && (y3 < y2))	/////////////////////////////////////////5번(- +)
		{
			tx = -sqrt(pow(0.1, 2) / (pow((real_y - m2) / (real_x - m1), 2) + 1)) + m1;
			ty = sqrt(pow(0.1, 2) - (pow(0.1, 2) / (pow((real_y - m2) / (real_x - m1), 2) + 1))) + m2;
			distance = sqrt(pow(real_x - tx, 2) + pow(real_y - ty, 2));
		}
		else if (((angle3 > 0.9) && (angle3 < 1.1)) && (x3 < x2))	/////////////////////////////////////////6번(+ -)
		{
			tx = sqrt(pow(0.1, 2) / (pow((real_y - m2) / (real_x - m1), 2) + 1)) + m1;
			ty = -sqrt(pow(0.1, 2) - (pow(0.1, 2) / (pow((real_y - m2) / (real_x - m1), 2) + 1))) + m2;
			distance = sqrt(pow(real_x - tx, 2) + pow(real_y - ty, 2));
		}
		else if (((angle3 < -0.9) && (angle3 > -1.1)) && (x3 > x2))	/////////////////////////////////////////7번(- -)
		{
			tx = -sqrt(pow(0.1, 2) / (pow((real_y - m2) / (real_x - m1), 2) + 1)) + m1;
			ty = -sqrt(pow(0.1, 2) - (pow(0.1, 2) / (pow((real_y - m2) / (real_x - m1), 2) + 1))) + m2;
			distance = sqrt(pow(real_x - tx, 2) + pow(real_y - ty, 2));
		}
		else if (((angle3 < -0.9) && (angle3 > -1.1)) && (y3 < y2))	/////////////////////////////////////////8번(+ +)
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

		//로봇의 좌표와 방향을 직접적으로 받음으로써 이론적으로 있어야하는 위치와 보고있는 방향을 비교해서 오차를 구한다.
		std::cout << "\n" << "이론상의 조향각 : " << atan(theory_angle) << " " << "실제 조향각 : " << real_angle << "\n";
		std::cout << "조향각 오차 : " << err_angle << "\n";		
		std::cout << "접점의 좌표 : " << tx << " " << ty << "\n";
		std::cout << "횡방향 오차 : " << distance << "\n";
	}

	else {		//좌표가 ㄱ형태가 아닌 각도가 좀 더 벌어진 보다 완만한 곡선의 경우
		if (y1 == y2) {			//과거, 현재 좌표의 직선이 x축에 평행하고 현재, 미래 좌표의 직선이 y = x 또는 y = -x 형태일 때
			m1 = x1;
			m2 = -(x3 - x2) / (y3 - y2) * (x1 - x3) + y3;
		}
		else if (y2 == y3) {	//현재, 미래 좌표의 직선이 x축에 평행하고 과거, 현재 좌표의 직선이 y = x 또는 y = -x 형태일 때
			m1 = x3;
			m2 = -(x2 - x1) / (y2 - y1) * (x3 - x1) + y1;
		}
		else if (x1 == x2) {	//과거, 현재 좌표의 직선이 y축에 평행하고 현재, 미래 좌표의 직선이 y = x 또는 y = -x 형태일 때
			m1 = -(y3 - y2) / (x3 - x2) * (y1 - y3) + x3;
			m2 = y1;
		}
		else if (x2 == x3) {	//현재, 미래 좌표의 직선이 x축에 평행하고 과거, 현재 좌표의 직선이 y = x 또는 y = -x 형태일 때
			m1 = -(y2 - y1) / (x2 - x1) * (y3 - y1) + x1;
			m2 = y3;
		}
		std::cout << "중점 : " << m1 << " " << m2 << "\n";
		std::cout << "곡선 경로" << "\n\n";

		theory_angle = -1 / ((real_y - m2) / (real_x - m1));		//차량의 좌표와 원의 중점을 이용하여 움직여야하는 곡선 경로에서의 조향각을 구한다.

		if (((angle3 > 0.4) && (angle3 < 0.6)) && (((y3 > y2) && (x1 < x2)) || ((x2 > x3) && (y1 > y2))))			/////////////////////////////////////////1번(+ -)
		{
			tx = sqrt(pow(0.6, 2) / (pow((real_y - m2) / (real_x - m1), 2) + 1)) + m1;								//생성되는 원의 방정식의 반지름은 지정한 구역에서 60cm, 즉 0.6m로 고정이다.
			ty = -sqrt(pow(0.6, 2) - (pow(0.6, 2) / (pow((real_y - m2) / (real_x - m1), 2) + 1))) + m2;				//위에서 구한 tx와 반지름를 이용하여 원의 방정식에 대입해 ty를 구한다.
			distance = sqrt(pow(real_x - tx, 2) + pow(real_y - ty, 2));
		}
		else if (((angle3 > 1.9) && (angle3 < 2.1)) && (((x3 > x2) && (y1 < y2)) || ((y2 > y3) && (x1 > x2))))		/////////////////////////////////////////2번(- +)
		{
			tx = -sqrt(pow(0.6, 2) / (pow((real_y - m2) / (real_x - m1), 2) + 1)) + m1;
			ty = sqrt(pow(0.6, 2) - (pow(0.6, 2) / (pow((real_y - m2) / (real_x - m1), 2) + 1))) + m2;
			distance = sqrt(pow(real_x - tx, 2) + pow(real_y - ty, 2));
		}
		else if (((angle3 < -1.9) && (angle3 > -2.1)) && (((x3 < x2) && (y1 < y2)) || ((y2 > y3) && (x1 < x2))))	/////////////////////////////////////////3번(+ +)
		{
			tx = sqrt(pow(0.6, 2) / (pow((real_y - m2) / (real_x - m1), 2) + 1)) + m1;
			ty = sqrt(pow(0.6, 2) - (pow(0.6, 2) / (pow((real_y - m2) / (real_x - m1), 2) + 1))) + m2;
			distance = sqrt(pow(real_x - tx, 2) + pow(real_y - ty, 2));
		}
		else if (((angle3 < -0.4) && (angle3 > -0.6)) && (((y3 > y2) && (x1 > x2)) || ((x2 < x3) && (y1 > y2))))	/////////////////////////////////////////4번(- -)
		{
			tx = -sqrt(pow(0.6, 2) / (pow((real_y - m2) / (real_x - m1), 2) + 1)) + m1;
			ty = -sqrt(pow(0.6, 2) - (pow(0.6, 2) / (pow((real_y - m2) / (real_x - m1), 2) + 1))) + m2;
			distance = sqrt(pow(real_x - tx, 2) + pow(real_y - ty, 2));
		}
		else if (((angle3 > 0.4) && (angle3 < 0.6)) && (((y3 < y2) && (x1 > x2)) || ((x2 < x3) && (y1 < y2))))		/////////////////////////////////////////5번(- +)
		{
			tx = -sqrt(pow(0.6, 2) / (pow((real_y - m2) / (real_x - m1), 2) + 1)) + m1;
			ty = sqrt(pow(0.6, 2) - (pow(0.6, 2) / (pow((real_y - m2) / (real_x - m1), 2) + 1))) + m2;
			distance = sqrt(pow(real_x - tx, 2) + pow(real_y - ty, 2));
		}
		else if (((angle3 > 1.9) && (angle3 < 2.1)) && (((x3 < x2) && (y1 > y2)) || ((y2 < y3) && (x1 < x2))))		/////////////////////////////////////////6번(+ -)
		{
			tx = sqrt(pow(0.6, 2) / (pow((real_y - m2) / (real_x - m1), 2) + 1)) + m1;
			ty = -sqrt(pow(0.6, 2) - (pow(0.6, 2) / (pow((real_y - m2) / (real_x - m1), 2) + 1))) + m2;
			distance = sqrt(pow(real_x - tx, 2) + pow(real_y - ty, 2));
		}
		else if (((angle3 < -1.9) && (angle3 > -2.1)) && (((x3 > x2) && (y1 > y2)) || ((y2 < y3) && (x1 > x2))))	/////////////////////////////////////////7번(- -)
		{
			tx = -sqrt(pow(0.6, 2) / (pow((real_y - m2) / (real_x - m1), 2) + 1)) + m1;
			ty = -sqrt(pow(0.6, 2) - (pow(0.6, 2) / (pow((real_y - m2) / (real_x - m1), 2) + 1))) + m2;
			distance = sqrt(pow(real_x - tx, 2) + pow(real_y - ty, 2));
		}
		else if (((angle3 < -0.4) && (angle3 > -0.6)) && (((y3 < y2) && (x1 < x2)) || ((x2 > x3) && (y1 < y2))))	/////////////////////////////////////////8번(+ +)
		{
			tx = sqrt(pow(0.6, 2) / (pow((real_y - m2) / (real_x - m1), 2) + 1)) + m1;
			ty = sqrt(pow(0.6, 2) - (pow(0.6, 2) / (pow((real_y - m2) / (real_x - m1), 2) + 1))) + m2;
			distance = sqrt(pow(real_x - tx, 2) + pow(real_y - ty, 2));
		}
		else
		{
			std::cout << "일치하는 경로가 없습니다." << "\n";
		}

		err_angle = atan(theory_angle) - real_angle;
		err_dist = distance;

		//로봇의 좌표와 방향을 직접적으로 받음으로써 이론적으로 있어야하는 위치와 보고있는 방향을 비교해서 오차를 구한다.
		std::cout << "\n" << "이론상의 조향각 : " << atan(theory_angle) << " " << "실제 조향각 : " << real_angle << "\n";
		std::cout << "조향각 오차 : " << err_angle << "\n";		
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
    	check = turtlebot3_drive.ErrorCalculate();
    	check = turtlebot3_drive.controlLoop();
	}
	ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
