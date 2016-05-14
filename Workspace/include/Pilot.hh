#pragma once

#include "Angle.hh"


struct Pilot
{
	unsigned short m_size;
	//destination direction
	Angle m_target;
	//current flying direction
	Angle m_current;
	Pilot(short size):m_size(size)
	{
	}
};


