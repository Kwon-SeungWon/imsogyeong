#include<iostream>
#include<cmath>
#include<math.h>

using namespace std;

int main(void) {
	const double pi = 3.141592;		//파이값
	double x1, x2, y1, y2, x3, y3;	//중점을 찾기위한 좌표
	double m1, m2;					//중점 좌표 저장
	double x, y;					//차량의 좌표 저장
	double tx, ty;					//접점의 좌표 저장
	double angle1, angle2, angle3;	//전, 지금의 기울기/ 지금, 후의 기울기 /경로 예상 기울기
	double real_angle;				//차량의 조향각
	double theory_angle;			//접점에서의 조향각
	double distance;				//차랑과 접점과의 거리

	cout << "전 좌표 입력 : ";
	cin >> x1 >> y1;
	cout << "지금 좌표 입력 : ";
	cin >> x2 >> y2;
	cout << "후 좌표 입력 : ";
	cin >> x3 >> y3;

	m1 = double(x1 + x3) / 2.0;
	m2 = double(y1 + y3) / 2.0;

	angle1 = (y2 - y1) / (x2 - x1);
	angle2 = (y3 - y2) / (x3 - x2);
	angle3 = (y3 - y1) / (x3 - x1);

	if ((((angle1 - angle2) < 0.01) && ((angle1 - angle2) > -0.01)) || ((angle2 > 1000) && (angle1 > 1000))) {
		cout << "\n" << "직선 경로" << "\n\n";
		cout << "차량의 좌표 : ";
		cin >> x >> y;
		cout << "차량의 방향 : ";
		cin >> real_angle;

		if ((angle3 > -0.01) && (angle3 < 0.01)) {					//////////////////////////////////////////////가로 직선
			tx = x2;
			ty = y2;
			distance = y2 - y;
			theory_angle = 0;
		}
		else if ((angle3 > 0.9) && (angle3 < 1.1)) {				//////////////////////////////////////////////세로 직선
			tx = (x + y - (y2 - x2)) / 2.0;
			ty = tx + (y2 - x2);
			distance = sqrt(pow(x - tx, 2) + pow(y - ty, 2));
			theory_angle = 1;
		}
		else if ((angle3 < -0.9) && (angle3 > -1.1)) {			//////////////////////////////////////////////대각 직선(+)
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
			cout << "일치하는 경로가 없습니다." << "\n";
		}
		cout << "\n" << "이론상의 조향각 : " << atan(theory_angle) << " " << "실제 조향각 : " << atan(real_angle) << "\n";
		cout << "조향각 오차 : " << atan(theory_angle) - atan(real_angle) << "\n";			//라디안 단위임 '도'단위로 바꿀려면 (* 180 / pi)
		cout << "접점의 좌표 : " << tx << " " << ty << "\n";
		cout << "횡방향 오차 : " << distance << "\n";
	}

	else if (((angle3 > 0.9) && (angle3 < 1.1)) || ((angle3 < -0.9) && (angle3 > -1.1))) {
		cout << "중점 : " << m1 << " " << m2 << "\n";
		cout << "곡선 경로1" << "\n\n";

		cout << "차량의 좌표 : ";
		cin >> x >> y;
		cout << "차량의 방향 : ";
		cin >> real_angle;

		theory_angle = -1 / ((y - m2) / (x - m1));

		if (((angle3 > 0.9) && (angle3 < 1.1)) && (y3 > y2))//////////////////////////////////////////////1번(+ -)
		{
			tx = sqrt(pow(0.5, 2) / (pow((y - m2) / (x - m1), 2) + 1)) + m1;
			ty = -sqrt(pow(0.5, 2) - (pow(0.5, 2) / (pow((y - m2) / (x - m1), 2) + 1))) + m2;
			distance = sqrt(pow(x - tx, 2) + pow(y - ty, 2));
		}
		else if (((angle3 > 0.9) && (angle3 < 1.1)) && (x3 > x2))/////////////////////////////////////////2번(- +)
		{
			tx = -sqrt(pow(0.5, 2) / (pow((y - m2) / (x - m1), 2) + 1)) + m1;
			ty = sqrt(pow(0.5, 2) - (pow(0.5, 2) / (pow((y - m2) / (x - m1), 2) + 1))) + m2;
			distance = sqrt(pow(x - tx, 2) + pow(y - ty, 2));
		}
		else if (((angle3 < -0.9) && (angle3 > -1.1)) && (x3 < x2))/////////////////////////////////////////3번(+ +)
		{
			tx = sqrt(pow(0.5, 2) / (pow((y - m2) / (x - m1), 2) + 1)) + m1;
			ty = sqrt(pow(0.5, 2) - (pow(0.5, 2) / (pow((y - m2) / (x - m1), 2) + 1))) + m2;
			distance = sqrt(pow(x - tx, 2) + pow(y - ty, 2));
		}
		else if (((angle3 < -0.9) && (angle3 > -1.1)) && (y3 > y2))/////////////////////////////////////////4번(- -)
		{
			tx = -sqrt(pow(0.5, 2) / (pow((y - m2) / (x - m1), 2) + 1)) + m1;
			ty = -sqrt(pow(0.5, 2) - (pow(0.5, 2) / (pow((y - m2) / (x - m1), 2) + 1))) + m2;
			distance = sqrt(pow(x - tx, 2) + pow(y - ty, 2));
		}
		else if (((angle3 > 0.9) && (angle3 < 1.1)) && (y3 < y2))/////////////////////////////////////////5번(- +)
		{
			tx = -sqrt(pow(0.5, 2) / (pow((y - m2) / (x - m1), 2) + 1)) + m1;
			ty = sqrt(pow(0.5, 2) - (pow(0.5, 2) / (pow((y - m2) / (x - m1), 2) + 1))) + m2;
			distance = sqrt(pow(x - tx, 2) + pow(y - ty, 2));
		}
		else if (((angle3 > 0.9) && (angle3 < 1.1)) && (x3 < x2))/////////////////////////////////////////6번(+ -)
		{
			tx = sqrt(pow(0.5, 2) / (pow((y - m2) / (x - m1), 2) + 1)) + m1;
			ty = -sqrt(pow(0.5, 2) - (pow(0.5, 2) / (pow((y - m2) / (x - m1), 2) + 1))) + m2;
			distance = sqrt(pow(x - tx, 2) + pow(y - ty, 2));
		}
		else if (((angle3 < -0.9) && (angle3 > -1.1)) && (x3 > x2))/////////////////////////////////////////7번(- -)
		{
			tx = -sqrt(pow(0.5, 2) / (pow((y - m2) / (x - m1), 2) + 1)) + m1;
			ty = -sqrt(pow(0.5, 2) - (pow(0.5, 2) / (pow((y - m2) / (x - m1), 2) + 1))) + m2;
			distance = sqrt(pow(x - tx, 2) + pow(y - ty, 2));
		}
		else if (((angle3 < -0.9) && (angle3 > -1.1)) && (y3 < y2))/////////////////////////////////////////8번(+ +)
		{
			tx = sqrt(pow(0.5, 2) / (pow((y - m2) / (x - m1), 2) + 1)) + m1;
			ty = sqrt(pow(0.5, 2) - (pow(0.5, 2) / (pow((y - m2) / (x - m1), 2) + 1))) + m2;
			distance = sqrt(pow(x - tx, 2) + pow(y - ty, 2));
		}
		else
		{
			cout << "일치하는 경로가 없습니다." << "\n";
		}

		cout << "\n" << "이론상의 조향각 : " << atan(theory_angle) << " " << "실제 조향각 : " << atan(real_angle) << "\n";
		cout << "조향각 오차 : " << atan(theory_angle) - atan(real_angle) << "\n";			//라디안 단위임 '도'단위로 바꿀려면 (* 180 / pi)
		cout << "접점의 좌표 : " << tx << " " << ty << "\n";
		cout << "횡방향 오차 : " << distance << "\n";
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
	cout << "중점 : " << m1 << " " << m2 << "\n";
	cout << "곡선 경로2" << "\n\n";

	cout << "차량의 좌표 : ";
	cin >> x >> y;
	cout << "차량의 방향 : ";
	cin >> real_angle;

	theory_angle = -1 / ((y - m2) / (x - m1));

	if (((angle3 > 0.4) && (angle3 < 0.6)) && (((y3 > y2) && (x1 < x2)) || ((x2 > x3) && (y1 > y2))))//////////////////////////////////////////////1번(+ -)
	{
		tx = sqrt(pow(3, 2) / (pow((y - m2) / (x - m1), 2) + 1)) + m1;
		ty = -sqrt(pow(3, 2) - (pow(3, 2) / (pow((y - m2) / (x - m1), 2) + 1))) + m2;
		distance = sqrt(pow(x - tx, 2) + pow(y - ty, 2));
	}
	else if (((angle3 > 1.9) && (angle3 < 2.1)) && (((x3 > x2) && (y1 < y2)) || ((y2 > y3) && (x1 > x2))))/////////////////////////////////////////2번(- +)
	{
		tx = -sqrt(pow(3, 2) / (pow((y - m2) / (x - m1), 2) + 1)) + m1;
		ty = sqrt(pow(3, 2) - (pow(3, 2) / (pow((y - m2) / (x - m1), 2) + 1))) + m2;
		distance = sqrt(pow(x - tx, 2) + pow(y - ty, 2));
	}
	else if (((angle3 < -1.9) && (angle3 > -2.1)) && (((x3 < x2) && (y1 < y2)) || ((y2 > y3) && (x1 < x2))))/////////////////////////////////////////3번(+ +)
	{
		tx = sqrt(pow(3, 2) / (pow((y - m2) / (x - m1), 2) + 1)) + m1;
		ty = sqrt(pow(3, 2) - (pow(3, 2) / (pow((y - m2) / (x - m1), 2) + 1))) + m2;
		distance = sqrt(pow(x - tx, 2) + pow(y - ty, 2));
	}
	else if (((angle3 < -0.4) && (angle3 > -0.6)) && (((y3 > y2) && (x1 > x2)) || ((x2 < x3) && (y1 > y2))))/////////////////////////////////////////4번(- -)
	{
		tx = -sqrt(pow(3, 2) / (pow((y - m2) / (x - m1), 2) + 1)) + m1;
		ty = -sqrt(pow(3, 2) - (pow(3, 2) / (pow((y - m2) / (x - m1), 2) + 1))) + m2;
		distance = sqrt(pow(x - tx, 2) + pow(y - ty, 2));
	}
	else if (((angle3 > 0.4) && (angle3 < 0.6)) && (((y3 < y2) && (x1 > x2)) || ((x2 < x3) && (y1 < y2))))/////////////////////////////////////////5번(- +)
	{
		tx = -sqrt(pow(3, 2) / (pow((y - m2) / (x - m1), 2) + 1)) + m1;
		ty = sqrt(pow(3, 2) - (pow(3, 2) / (pow((y - m2) / (x - m1), 2) + 1))) + m2;
		distance = sqrt(pow(x - tx, 2) + pow(y - ty, 2));
	}
	else if (((angle3 > 1.9) && (angle3 < 2.1)) && (((x3 < x2) && (y1 > y2)) || ((y2 < y3) && (x1 < x2))))/////////////////////////////////////////6번(+ -)
	{
		tx = sqrt(pow(3, 2) / (pow((y - m2) / (x - m1), 2) + 1)) + m1;
		ty = -sqrt(pow(3, 2) - (pow(3, 2) / (pow((y - m2) / (x - m1), 2) + 1))) + m2;
		distance = sqrt(pow(x - tx, 2) + pow(y - ty, 2));
	}
	else if (((angle3 < -1.9) && (angle3 > -2.1)) && (((x3 > x2) && (y1 > y2)) || ((y2 < y3) && (x1 > x2))))/////////////////////////////////////////7번(- -)
	{
		tx = -sqrt(pow(3, 2) / (pow((y - m2) / (x - m1), 2) + 1)) + m1;
		ty = -sqrt(pow(3, 2) - (pow(3, 2) / (pow((y - m2) / (x - m1), 2) + 1))) + m2;
		distance = sqrt(pow(x - tx, 2) + pow(y - ty, 2));
	}
	else if (((angle3 < -0.4) && (angle3 > -0.6)) && (((y3 < y2) && (x1 < x2)) || ((x2 > x3) && (y1 < y2))))/////////////////////////////////////////8번(+ +)
	{
		tx = sqrt(pow(3, 2) / (pow((y - m2) / (x - m1), 2) + 1)) + m1;
		ty = sqrt(pow(3, 2) - (pow(3, 2) / (pow((y - m2) / (x - m1), 2) + 1))) + m2;
		distance = sqrt(pow(x - tx, 2) + pow(y - ty, 2));
	}
	else
	{
		cout << "일치하는 경로가 없습니다." << "\n";
	}

	cout << "\n" << "이론상의 조향각 : " << atan(theory_angle) << " " << "실제 조향각 : " << atan(real_angle) << "\n";
	cout << "조향각 오차 : " << atan(theory_angle) - atan(real_angle) << "\n";			//라디안 단위임 '도'단위로 바꿀려면 (* 180 / pi)
	cout << "접점의 좌표 : " << tx << " " << ty << "\n";
	cout << "횡방향 오차 : " << distance << "\n";
}

	return 0;
}