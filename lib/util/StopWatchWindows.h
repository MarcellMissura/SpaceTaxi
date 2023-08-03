#ifndef STOPWATCHWINDOWS_H_
#define STOPWATCHWINDOWS_H_

#include "windows.h"

class SubStopWatch
{
	LARGE_INTEGER cpuTicksAtProgramStart;
	LARGE_INTEGER cpuTicksAtLastRestart;
    LARGE_INTEGER cpuTicksPerSecond;
    LARGE_INTEGER currentCpuTicks;

public:
    SubStopWatch();
    ~SubStopWatch(){};
    void start();
	double elapsedTime();
	double programTime();
	double systemTime();
};

#endif /* STOPWATCHWINDOWS_H_ */
