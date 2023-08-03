#include "StopWatchWindows.h"

SubStopWatch::SubStopWatch()
{
	// High precision tick counters for real time measurement.
	QueryPerformanceFrequency(&cpuTicksPerSecond);
	QueryPerformanceCounter(&cpuTicksAtProgramStart);
	cpuTicksAtLastRestart = cpuTicksAtProgramStart;
}

// Resets the elapsed time to 0.
void SubStopWatch::start()
{
	QueryPerformanceCounter(&cpuTicksAtLastRestart);
}

// Returns a time stamp expressed in seconds since the last restart.
double SubStopWatch::elapsedTime()
{
	QueryPerformanceCounter(&currentCpuTicks);
	return ((double)currentCpuTicks.QuadPart - (double)cpuTicksAtLastRestart.QuadPart) / (double)cpuTicksPerSecond.QuadPart;
}

// Returns a time stamp expressed in seconds since program start.
double SubStopWatch::programTime()
{
	QueryPerformanceCounter(&currentCpuTicks);
	return ((double)currentCpuTicks.QuadPart - (double)cpuTicksAtProgramStart.QuadPart) / (double)cpuTicksPerSecond.QuadPart;
}

// Returns a system timestamp expressed in seconds since I don't know when.
double SubStopWatch::systemTime()
{
	QueryPerformanceCounter(&currentCpuTicks);
	return ((double)currentCpuTicks.QuadPart) / (double)cpuTicksPerSecond.QuadPart;
}

