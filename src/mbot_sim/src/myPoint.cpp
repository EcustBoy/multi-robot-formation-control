#include "myPoint.h"

MyPoint::MyPoint(double xPos, double yPos)
{
	x = xPos;
	y = yPos;
}

MyPoint::~MyPoint()
{
}

double MyPoint::getAngle(MyPoint* target)  //计算两个向量角度
{
	double angle;
	angle = atan2((target->y - y),(target->x - x));
	return angle;
}

double MyPoint::getDistance(MyPoint* target)  //计算两点之间距离离
{
	double distance;
	distance = sqrt((pow(target->y - y,2.0)) + (pow(target->x - x,2.0)));
	return distance;
}

MyPoint MyPoint::operator+(const MyPoint &other) const  //计算向量的和
{
   MyPoint result = *(new MyPoint(x+other.x, y+other.y));
   return result;
} 

MyPoint MyPoint::operator-(const MyPoint &other) const//计算向量的差
{
   MyPoint result = *(new MyPoint(x-other.x, y-other.y));
   return result;
} 

MyPoint MyPoint::times(double r){    //向量乘以实数
	return *(new MyPoint(r*x, r*y));
}

double MyPoint::getAbs()   //向量的绝对值
{
	return sqrt((pow(y,2.0)) + (pow(x,2.0)));
}
