// ObstacleAvoidance.cpp : 定义控制台应用程序的入口点。
//
#pragma once

#include <stdexcept>
#include <math.h>
#include <utility>
#include <algorithm>
#include <vector>
#include <iostream>
#include <stdexcept>

using namespace std;

#define M_PI       3.14159265358979323846   // pi


struct Angle
{
	unsigned short m_angle;
	enum DIRECTION
	{
		CLOCKWISE,
		ANTICLOCKWISE
	};
	Angle(unsigned short angle = 0) :m_angle(angle)
	{}
	Angle(const Angle& a) : m_angle(a.m_angle)
	{}
	Angle(double radian)
	{
		m_angle = (unsigned short)ceil(radian / M_PI * 180.0);
	}
	bool operator < (const Angle &a) const
	{
		return m_angle < a.m_angle;
	}
	bool operator > (const Angle &a) const
	{
		return m_angle > a.m_angle;
	}

	Angle &operator ++()
	{
		unsigned short angle = (++m_angle) % 360;
		m_angle = angle;
		return *this;
	}
	Angle operator ++(int)
	{
		Angle tmp(*this);
		++(*this);
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
		--(*this);
		return tmp;
	}
	//顺时针旋转a°
	Angle operator + (unsigned short a) const
	{//顺时针转向
		Angle tmp(*this);
		tmp.m_angle = (tmp.m_angle + a) % 360;
		return tmp;
	}
	Angle operator + (const Angle& a) const
	{
		return Angle(*this) + a.m_angle;
	}
	//逆时针旋转a°
	Angle operator - (unsigned short a) const
	{//逆时针转动
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
	//两角差值（小于180的差值）
	Angle absDiff(const Angle &a) const
	{
		unsigned short tmp = m_angle > a.m_angle ? m_angle - a.m_angle : a.m_angle - m_angle;
		if (tmp > 180)
		{
			tmp = 360 - tmp;
		}
		return Angle(tmp);
	}
	operator unsigned short() const
	{
		return m_angle;
	}
	//不包括尾端end的
	//顺时针判断
	bool IsBelong(const Angle& begin, const Angle &end) const
	{
		return (*this + begin).m_angle < end.m_angle;
	}

};


struct MyPoint
{
	unsigned short quality;
	Angle angle;
	unsigned short distance;
	MyPoint(unsigned short q = 0,unsigned short a = 0,unsigned short d = 0) : quality(q),angle((unsigned short)q),distance(d)
	{}
	bool Available() const
	{
		return quality != 0;
	}

};

//飞行对象
struct Object
{
	//飞机尺寸
	unsigned short m_size;
	//目标方向
	Angle m_targetAngle;
	//当前方向
	Angle m_currentAngle;
	Object(short size) :m_size(size), m_targetAngle((unsigned short)0), m_currentAngle((unsigned short)0)
	{}
	void SetTarget(unsigned short angle)
	{
		m_targetAngle = angle;
	}
};

//--------------------------------------------探测策略------------------------------------------------------------------

struct CannotDecide : runtime_error 
{
	CannotDecide():runtime_error("crash\n")
	{
	}
};


//飞行方向障碍点检测
struct DetectStrategy
{
	//记录安全距离范围(从0-90读的最小安全距离)
	unsigned short m_safeScope[91];
	//对象飞行器
	const Object& m_obj;
	//安全距离
	unsigned short m_safeDistance;
	DetectStrategy(const Object& obj, unsigned short safeDistance = 0) :m_obj(obj)
	{
		Modify(safeDistance);
	}
	void Modify(short safeDistance)
	{
		m_safeDistance = safeDistance;
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

	bool Detect(const MyPoint& point) const
	{
		return Detect(point.angle - m_obj.m_targetAngle, point.distance);
	}
	//target-飞行方向
	//point-检测点
	//返回值-true：有障碍物；false：无障碍物
	bool Detect(const Angle &target, const MyPoint& point) const
	{
		//在检测范围之内
		return Detect(point.angle.absDiff(target), point.distance);
	}
	//angle传入角度为转换到0-90°之后的角度
	bool Detect(const Angle &angle, unsigned short distance) const
	{

		if (angle.IsBelong((unsigned short)90, (unsigned short)0))
		{//不在检测范围之内
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


	//顺时针移动
	Angle ClockwiseMove(const MyPoint& p) const
	{
		if (p.distance < m_obj.m_size)
		{
			throw CannotDecide();
		}
		Angle a = asin((double)m_obj.m_size / p.distance);
		//左安全边界横穿障碍点
		return p.angle + a;
	}
	Angle AnticlockwiseMove(const MyPoint& p) const
	{
		if (p.distance < m_obj.m_size)
		{
			throw CannotDecide();
		}
		Angle a = asin((double)m_obj.m_size / p.distance);
		//右安全边界横穿障碍点
		return p.angle - a;
	}


	//顺时针扫描到target
	const MyPoint* ClockwiseScan(const Angle& target, const vector<MyPoint> &map) const
	{
		for (unsigned short i = 90; i != 0xffff; --i)
		{
			if (Detect(target, map[target - i]))
			{
				return &map[target - i];
			}
		}
		return NULL;
	}
	//逆时针扫描到target
	const MyPoint* AnticlockwiseScan(const Angle& target, const vector<MyPoint> &map) const
	{
		for (unsigned short i = 90; i != 0xffff; --i)
		{
			if (Detect(target, map[target + i]))
			{
				return &map[target + i];
			}
		}
		return NULL;
	}

	//从obstacle障碍点开始
	//顺时针寻找可行方案
	const Angle ClockwiseSearch(const MyPoint* obstacle, const vector<MyPoint> &map) const
	{
		if (obstacle)
		{
			Angle lastCandidate(obstacle->angle);
			for (unsigned short i = 0; i <= 360;)
			{
				Angle candidate = ClockwiseMove(*obstacle);

				i += lastCandidate.absDiff(candidate);
				lastCandidate = candidate;

				obstacle = AnticlockwiseScan(candidate, map);
				if (!obstacle)
				{
					return candidate;
				}
			}
		}
		//未找到解决方案
		return Angle((unsigned short)0xffff);
	}
	//从obstacle障碍点开始
	//逆时针寻找可行方案
	const Angle AnticlockwiseSearch(const MyPoint* obstacle, const vector<MyPoint> &map) const
	{
		if (obstacle)
		{
			Angle lastCandidate(obstacle->angle);
			for (unsigned short i = 0; i <= 360;)
			{
				Angle candidate = AnticlockwiseMove(*obstacle);

				i += lastCandidate.absDiff(candidate);
				lastCandidate = candidate;

				obstacle = ClockwiseScan(candidate, map);
				if (!obstacle)
				{
					return candidate;
				}
			}
		}
		//未找到解决方案
		return Angle((unsigned short)0xffff);
	}
};


//-----------------------------飞行策略-----------------------------------------------------

//当前飞行策略
struct FlyStrategy
{
	virtual const MyPoint Strategy(const vector<MyPoint> &map, const DetectStrategy &stt, FlyStrategy **currentStrategy) const = 0;
	virtual ~FlyStrategy()
	{
	}
};

//正常飞行策略
struct NormalStrategy :public FlyStrategy
{
	virtual const MyPoint Strategy(const vector<MyPoint> &map, const DetectStrategy &stt, FlyStrategy **currentStrategy) const;
};

//凸多边形避障策略
struct CovexPolygonStrategy : public FlyStrategy
{
	virtual const MyPoint Strategy(const vector<MyPoint> &map, const DetectStrategy &stt, FlyStrategy **currentStrategy) const;
};

const MyPoint NormalStrategy::Strategy(const vector<MyPoint> &map, const DetectStrategy &stt, FlyStrategy **currentStrategy) const
{
	MyPoint destination(0,stt.m_obj.m_targetAngle,stt.m_safeDistance );

	//偏左侧障碍
	const MyPoint* leftObstacle = stt.ClockwiseScan(stt.m_obj.m_targetAngle, map);
	//偏右侧障碍物
	const MyPoint* rightObstacle = stt.AnticlockwiseScan(stt.m_obj.m_targetAngle, map);
	//当前飞行方向无障碍
	if (leftObstacle == NULL && rightObstacle == NULL)
	{
		return destination;
	}

	//cw为顺时针寻找出路
	//acw为逆时针寻找出路
	Angle cw((unsigned short)0xffff), acw((unsigned short)0xffff);
	cw = stt.ClockwiseSearch(rightObstacle, map);
	acw = stt.AnticlockwiseSearch(leftObstacle, map);


	//选择和目标方向夹角小的角度
	destination.angle = cw.absDiff(stt.m_obj.m_targetAngle) < acw.absDiff(stt.m_obj.m_targetAngle) ? cw : acw;

	if (destination.angle.absDiff(stt.m_obj.m_targetAngle) <= 360)
	{//与目标方向夹角为锐角，则切换到凸多边形壁障模式
		delete *currentStrategy;
		*currentStrategy = new CovexPolygonStrategy();
	}
	else
	{
		cout << "CovexPloygon" << endl;
		throw runtime_error("CovexPloygon");
	}
	return destination;
}

const MyPoint CovexPolygonStrategy::Strategy(const vector<MyPoint> &map, const DetectStrategy &stt, FlyStrategy **currentStrategy) const
{
	MyPoint destination;
	//Angle planAngle = stt.m_obj.m_targetAngle;

	//偏左侧障碍
	const MyPoint* leftObstacle = stt.ClockwiseScan(stt.m_obj.m_targetAngle, map);
	//偏右侧障碍物
	const MyPoint* rightObstacle = stt.AnticlockwiseScan(stt.m_obj.m_targetAngle, map);

	//当前飞行方向无障碍
	if (leftObstacle == NULL && rightObstacle == NULL)
	{
		delete *currentStrategy;
		*currentStrategy = new NormalStrategy();
		destination.angle = stt.m_obj.m_targetAngle;
		destination.distance = stt.m_safeDistance;
		return destination;
	}

	//计划方向
	Angle plan((unsigned short)0xffff);
	//在当目标飞行方向和当前飞行方向之间寻找最优方向
	if (stt.m_obj.m_targetAngle + (unsigned short)90 >= stt.m_obj.m_currentAngle)
	{//顺时针方向寻找出路
		plan = stt.ClockwiseSearch(rightObstacle ? rightObstacle : leftObstacle, map);
	}
	else if (stt.m_obj.m_currentAngle + (unsigned short)90 >= stt.m_obj.m_targetAngle)
	{//逆时针方向寻找出路
		plan = stt.AnticlockwiseSearch(leftObstacle ? leftObstacle : rightObstacle, map);
	}

	if (plan > Angle((unsigned short)360))
	{
		throw runtime_error("no way out");
	}

	destination.angle = plan;
	destination.distance = stt.m_safeDistance;
	return destination;
}

struct DecisionStrategy
{
	FlyStrategy *m_currentStrategy;
	DecisionStrategy() : m_currentStrategy(new NormalStrategy())
	{

	}
	~DecisionStrategy()
	{
		delete m_currentStrategy;
	}
	const MyPoint Strategy(const vector<MyPoint> &map, const DetectStrategy &stt)
	{
		return m_currentStrategy->Strategy(map, stt, &m_currentStrategy);
	}
};


//两点之间的距离
unsigned short Distance(const MyPoint& p1, const MyPoint& p2)
{
	unsigned short angle = p1.angle - p2.angle;
	double cosa = cos(angle / 180 * M_PI);
	double sina = sin(angle / 180 * M_PI);
	return (unsigned short)sqrt(pow(sina * p1.distance, 2) + pow(p2.distance - p1.distance * cosa, 2));
}


/*
const MyPoint Decision(const DetectStrategy &stt, const Object &obj, const vector<MyPoint> &map)
{
	if (map.size() != 360)
	{
		return MyPoint();
	}
	MyPoint destination( 0,0,stt.m_safeDistance );



	return destination;
}
*/

/*
int main()
{
	Object obj(500);
	DetectStrategy stt(obj, 500);
	
	MyPoint p1{ 0,(unsigned short)274, 1111 }, p2{ 0, (unsigned short)315, 200 };
	stt.Detect(p1);
	stt.Detect(p1);
	//auto d1 = stt.ClockwiseMove(p1);
	//auto d2 = stt.Move(p2);
	//DecisionStrategy ds;
	//vector<Point> map(360);
	//for (unsigned short i = 00; i < 360; ++i)
	//{
	//	Point p1{0,i,3000};
	//	map[i] = p1;
	//}
	//map[1].distance = 700;
	//try
	//{
	//	map[1].distance = 700;
	//	Point p = ds.Strategy(map, stt);
	//	cout << p.angle << endl;

	//	obj.m_currentAngle = p.angle;
	//
	//	map[1].distance = 7000;
	//	p = ds.Strategy(map, stt);
	//	cout << p.angle << endl;
	//}
	//catch (exception &e)
	//{
	//	cout << e.what() << endl;
	//}
	//
return		0;
}
*/
