#pragma once

#include <math.h>

struct Angle
{
	//0 means Clockwise, default value
	//1 means Anticlockwise
	enum DIRECTION{CLOCKWISE,ANTICLOCKWISE};
	unsigned short m_direction : 1;
	unsigned short  m_value : 7;

	Angle(short value= 0) : m_direction(0) ,  m_value(value)
	{}
	Angle(double radian) : m_direction(0)
	{
		m_value = (unsigned short)ceil(radian/M_PI * 180.0)%360;
	}
	Angle & operator = (unsigned short a)
	{
		m_value = a;
		return *this;
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
	Angle operator + (const Angle &a) const
	{
		return *this + a.m_value;
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
	Angle operator - (const Angle &a) const
	{
		return *this - a.m_value;
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
};
	
