// ObstacleAvoidance.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include <stdio.h>
#include <math.h>
#include <utility>
#include <algorithm>
#include <vector>

using namespace std;

#define M_PI       3.14159265358979323846   // pi

struct Angle
{
	unsigned short m_angle;
	Angle(unsigned short angle = 0) :m_angle(angle)
	{}
	Angle(const Angle& a) : m_angle(a.m_angle)
	{}
	Angle(double radian)
	{
		m_angle = (unsigned short)(radian / M_PI * 180);
	}
	Angle &operator ++()
	{
		m_angle = (++m_angle) % 360;
		return *this;
	}
	Angle operator ++(int)
	{
		Angle tmp(*this);
		(*this)++;
		return tmp;
	}
	Angle &operator --()
	{
		m_angle = m_angle == 0 ? 359 : m_angle - 1;
		return *this;
	}
	Angle operator --(int)
	{
		Angle tmp(*this);
		(*this)--;
		return tmp;
	}
	Angle operator + (unsigned short a) const
	{
		Angle tmp(*this);
		tmp.m_angle = (tmp.m_angle + a) % 360;
		return tmp;
	}
	Angle operator + (const Angle& a) const
	{
		return Angle(*this) + a.m_angle;
	}
	Angle operator - (unsigned short a) const
	{
		Angle tmp(*this);
		if (a != 0)
		{
			tmp.m_angle = tmp.m_angle < a ? 360 + tmp.m_angle - a : tmp.m_angle - a;
		}
		return tmp;
	}
	Angle operator - (const Angle& a) const
	{
		return Angle(*this) - a.m_angle;
	}
	operator unsigned short() const
	{
		return m_angle;
	}
	//不包括尾端end的
	bool IsBelong(const Angle& begin, const Angle &end) const
	{
		return (*this + begin).m_angle < end.m_angle;
	}
};


struct Point
{
	unsigned short quality;
	Angle angle;
	unsigned short distance;
};

//飞行对象
struct Object
{
	//飞机尺寸
	unsigned short m_size;
	unsigned short m_targetAngle;
	Object(short size) :m_size(size), m_targetAngle(0)
	{}
	void SetTarget(unsigned short angle)
	{
		m_targetAngle = angle;
	}
};

//探测策略
//飞行方向障碍点检测
struct DetectStrategy
{
	//记录安全距离范围(从270-360读的最小安全距离)
	unsigned short m_safeScope[91];
	//对象飞行器
	const Object& m_obj;
	//安全距离
	unsigned short m_safeDistance;
	DetectStrategy(const Object& obj,unsigned short safeDistance = 0) :m_obj(obj)
	{
		Modify(safeDistance);
	}
	void Modify(short safeDistance)
	{
		m_safeDistance = m_safeDistance;
		//计算当前位置到最远安全距离的切线与圆心连线的夹角
		short angle = (short)((atan((double)m_obj.m_size / safeDistance) / M_PI) * 180.0);

		//计算圆心连线到切线的夹角的安全距离
		for (short i = 0; i < angle; ++i)
		{
			m_safeScope[i] = (short)ceil(
				safeDistance * cos(i / 180.0 * M_PI) +
				sqrt(m_obj.m_size * m_obj.m_size - pow(safeDistance * sin(i / 180.0 * M_PI), 2)));
		}

		//计算切线到90°直径的夹角的安全距离
		for (short i = angle; i <= 90; ++i)
		{
			m_safeScope[i] = (short)ceil(
				m_obj.m_size / cos((90 - i) / 180.0 * M_PI));
		}
	}
	bool Detect(const Point& point)
	{
		return Detect(point.angle - m_obj.m_targetAngle, point.distance);
	}
	//target-飞行方向
	//point-检测点
	//返回值-true：有障碍物；false：无障碍物
	bool Detect(const Angle &target, const Point& point)
	{
		//在检测范围之内
		return Detect(point.angle - target, point.distance);
	}
	bool Detect(const Angle &angle, short distance)
	{
		//在检测范围之内
		if (angle.IsBelong((unsigned short)90, (unsigned short)0))
		{
			return false;
		}
		else
		{
			if (distance >= m_safeScope[angle])
				return false;
			else
				return true;
		}
	}

	//监测点平移
	Angle Clockwise(const Angle &target, const Point& p)
	{
		Angle a = acos((double)m_obj.m_size/p.distance);
		return Angle((unsigned short)90) - a + p.angle - target;
	}
};

struct DecisionStrategy
{
	//探测策略
	const DetectStrategy& m_stt;

};

//凸多边形避障策略
struct CovexPolygonStrategy
{
	const Point Strategy(const vector<Point> &map,const Object &obj)
	{

	}
};

//凹多边形避障策略
struct ConcavePolygonStrategy
{

};


//两点之间的距离
unsigned short Distance(const Point& p1,const Point& p2)
{
	unsigned short angle = p1.angle - p2.angle;
	double cosa = cos(angle / 180 * M_PI);
	double sina = sin(angle / 180 * M_PI);
	return (unsigned short)sqrt(pow(sina * p1.distance, 2) + pow(p2.distance - p1.distance * cosa, 2));
}



const Point Decision(const DetectStrategy &stt,const Object &obj,const vector<Point> &map)
{
	if (map.size() != 360)
	{
		return Point();
	}
	Point destination{0,(unsigned short)0,stt.m_safeDistance};



	return destination;
}




int main()
{
	Object obj(100);
	DetectStrategy stt(obj, 100);
	Point p1{ 0,(unsigned short)45, 200 }, p2{ 0, (unsigned short)270, 0 };
	stt.Detect(p1);
	Angle d = stt.Clockwise((unsigned short)0,p1);
	return 0;
}

