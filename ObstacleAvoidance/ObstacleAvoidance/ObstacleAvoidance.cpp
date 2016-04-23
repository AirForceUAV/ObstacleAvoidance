// ObstacleAvoidance.cpp : �������̨Ӧ�ó������ڵ㡣
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
	//������β��end��
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

//���ж���
struct Object
{
	//�ɻ��ߴ�
	unsigned short m_size;
	unsigned short m_targetAngle;
	Object(short size) :m_size(size), m_targetAngle(0)
	{}
	void SetTarget(unsigned short angle)
	{
		m_targetAngle = angle;
	}
};

//̽�����
//���з����ϰ�����
struct DetectStrategy
{
	//��¼��ȫ���뷶Χ(��270-360������С��ȫ����)
	unsigned short m_safeScope[91];
	//���������
	const Object& m_obj;
	//��ȫ����
	unsigned short m_safeDistance;
	DetectStrategy(const Object& obj,unsigned short safeDistance = 0) :m_obj(obj)
	{
		Modify(safeDistance);
	}
	void Modify(short safeDistance)
	{
		m_safeDistance = m_safeDistance;
		//���㵱ǰλ�õ���Զ��ȫ�����������Բ�����ߵļн�
		short angle = (short)((atan((double)m_obj.m_size / safeDistance) / M_PI) * 180.0);

		//����Բ�����ߵ����ߵļнǵİ�ȫ����
		for (short i = 0; i < angle; ++i)
		{
			m_safeScope[i] = (short)ceil(
				safeDistance * cos(i / 180.0 * M_PI) +
				sqrt(m_obj.m_size * m_obj.m_size - pow(safeDistance * sin(i / 180.0 * M_PI), 2)));
		}

		//�������ߵ�90��ֱ���ļнǵİ�ȫ����
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
	//target-���з���
	//point-����
	//����ֵ-true�����ϰ��false�����ϰ���
	bool Detect(const Angle &target, const Point& point)
	{
		//�ڼ�ⷶΧ֮��
		return Detect(point.angle - target, point.distance);
	}
	bool Detect(const Angle &angle, short distance)
	{
		//�ڼ�ⷶΧ֮��
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

	//����ƽ��
	Angle Clockwise(const Angle &target, const Point& p)
	{
		Angle a = acos((double)m_obj.m_size/p.distance);
		return Angle((unsigned short)90) - a + p.angle - target;
	}
};

struct DecisionStrategy
{
	//̽�����
	const DetectStrategy& m_stt;

};

//͹����α��ϲ���
struct CovexPolygonStrategy
{
	const Point Strategy(const vector<Point> &map,const Object &obj)
	{

	}
};

//������α��ϲ���
struct ConcavePolygonStrategy
{

};


//����֮��ľ���
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

