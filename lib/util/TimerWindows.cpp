#include "TimerWindows.h"

SubTimer::SubTimer(QObject *parent) : QObject (parent)
{
	// Set resolution to the minimum supported by the system
    TIMECAPS tc;
    timeGetDevCaps(&tc, sizeof(TIMECAPS));
	timerRes = qMin(qMax(tc.wPeriodMin, (UINT) 0), tc.wPeriodMax);
    timeBeginPeriod(timerRes);

    timerId = 0;
    threadPrioritySet = false;
    running = false;
}

SubTimer::~SubTimer()
{
	stop();
}

void SubTimer::startTimer(int milliSeconds)
{
//	SetPriorityClass(GetCurrentProcess(), HIGH_PRIORITY_CLASS); //REALTIME_PRIORITY_CLASS
	timerId = timeSetEvent(milliSeconds, 0,	SubTimer::PeriodicCallback, (DWORD_PTR)this, 1);
	running = true;
}

void SubTimer::stop()
{
	running = false;
	threadPrioritySet = false;

	if (timerId)
		timeKillEvent(timerId);

	if (timerRes)
		timeEndPeriod(timerRes);

	timerId = 0;
}

void SubTimer::fire()
{
	if (running)
	{
		if (not threadPrioritySet)
		{
			threadPrioritySet = true;
			SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_TIME_CRITICAL);
		}

		emit timeOut();
	}
}

bool SubTimer::isActive()
{
	return timerId;
}


