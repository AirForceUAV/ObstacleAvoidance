#pragma once

#include "../include/Pilot.hh"

struct DetectStrategy
{
	//record the maximal safe distance from 0 degree to 90 degree.
	unsigned short m_safeScope[91];
        //safe distance
	unsigned short m_safeDistance;
	//the object of pilot
	const Pilot &m_obj;
       
	DetectStrategy(const Pilot& obj,unsigned short safeDistance = 0):m_obj(obj);
	{
		Modify(safeDistance);
	}
	//modify safe distance
	void Modify(short safeDistance)
	{
		m_safeDistance = safeDistance;
		//caculating the angle of the tangent of center of the pilot to to maximal reachable position with .
		short angle = (short)((atan((double)m_obj.m_size / m_safeDistance) / M_PI) * 180.0);

		//caculating the maximal safe distance between 0 to the angle
		for (unsigned char i = 0;i < angle;++i)
		{
			m_safeScope[i] = (unsigned short)ceil(safeDistance * cos(i / 180.0 * M_PI) + sqrt(m_obj.m_size * m_obj.m_size - pow(m_safeDistance * sin(i / 180.0 * M_PI), 2)));
		}

		//caculating the maximal safe distance between the angle to 90
		for (unsigned char i = angle;i <= 90;++i)
		{
			m_safeScope[i] = (short)ceil(m_obj.m_size / cos((90 - i) / 180.0 * M_PI));
		}
	}
	
	bool Detect(const Point& point) const
	{
		return Detect(point.angle - m_obj.m_targetAngle,point.distance);
	}












}
