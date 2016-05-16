#pragma once

#include <math.h>

struct Angle
{
	//1 means Anticlockwise
	enum DIRECTION{CLOCKWISE,ANTICLOCKWISE};
	unsigned short m_direction : 1;
	unsigned short  m_value : 7;

	Angle(short value= 0) : m_direction(0) ,  m_value(value)
	{}
	Angle(const Angle &a)
	{
		m_direction = a.m_direction;
		m_value = a.m_value;
	}
	Angle & operator = (unsigned short a)
	{
		m_value = a;
		return *this;
	}

	//covert from radian to angle
	void Radian(double radian)
	{
		m_value = (unsigned short)ceil(radian/M_PI * 180.0)%360;
	}

	//only compare the value without considing the direction
	bool operator < (const Angle &a) const
	{
		return m_value < a.m_value;
	}
	bool operator > (const Angle &a) const
	{
		return m_value > a.m_value;
	}

	//roll by original direction
	Angle & operator ++ ()
	{
		m_value = (++m_value) % 360;
		return *this;
	}
	Angle operator ++ (int)
	{
		Angle tmp(*this);
		++(*this);
		return tmp;
	}
	Angle & operator -- ()
	{
		m_value = (m_value == 0) ? 359 : m_value - 1;
		return *this;
	}
	Angle operator -- (int)
	{
		Angle tmp(*this);
		--(*this);
		return tmp;
	}
	
	//same direction to accumulation, 
	//opposite direction to offset
	Angle& operator += (short a) 
	{
		if(a >= 0)
		{
			m_value = (m_value + a) % 360;
		}
		else
		{
			*this -= (-a);
		}
		return *this;
	}
	Angle operator + (short a) const
	{
		Angle tmp(*this);
		tmp += a;
		return tmp;
	}
	Angle& operator -= (short a) 
	{
		
		if(a > 0)
		{
			m_value = (m_value < a) ? 360 + m_value - a : m_value - a;
		}
		else
		{
			*this += (-a);
		}
		return *this;
	}
	Angle operator - (short a) const
	{
		Angle tmp(*this);
		tmp -= a;
		return tmp;
	}

	//the difference between this angle with a angle,the difference value isn't always greater than 180.
	unsigned short absDiff(const Angle &a) const
	{
		Angle tmp(a);
		if(m_direction ^ tmp.m_direction)
		{
			tmp += 180;	
		}	
		unsigned short diff = (m_value > tmp.m_value) ? m_value - tmp.m_value : tmp.m_value - m_value;
		if(diff > 180)
		{
			diff = 360 - diff;
		}
		return diff;
	}
	//Is the angle between the start to end? irrespective the end point
	bool between(const Angle &start,const Angle &end) const
	{
		if(start.m_direction ^ end.m_direction)
		{//different directional angle cann't consider
			return false;
		}
		else
		{
			Angle tmp(*this);
			if(tmp.m_direction ^ start.m_direction)
			{
				tmp.m_value = 360 - tmp.m_value;
			}
			return 


		}	

	}
};
	
