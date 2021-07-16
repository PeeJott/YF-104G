#include "Timer.h"

Timer::Timer(double time) : m_time{ time }, m_timeElapsed{ 0.0 }, m_isActive{ false }
{

}

Timer::~Timer()
{

}

void Timer::zeroInit()
{

}
void Timer::coldInit()
{

}
void Timer::hotInit()
{

}
void Timer::airborneInit()
{

}


void Timer::startTimer()
{
	m_isActive = true;
	m_timeElapsed = 0.0;
}

void Timer::updateLoop(double& dt)
{
	if (m_isActive)
	{
		m_timeElapsed += dt;
	}

	if (m_timeElapsed > m_time)
	{
		m_isActive = false;
	}

}