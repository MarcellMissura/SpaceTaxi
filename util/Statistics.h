#ifndef STATISTICS_H_
#define STATISTICS_H_

#include "util/Vector.h"
#include "util/Vec2.h"

class Statistics
{
    static uint seed;

public:
    Statistics();
    ~Statistics(){}
    static void init();
    static void setSeed(uint s);
    static uint getSeed();
    static void reSeed();

    static double min(const Vector<double>& values);
    static uint min(const Vector<uint>& values);
    static int min(const Vector<int>& values);
    static double max(const Vector<double>& values);
    static uint max(const Vector<uint>& values);
    static int max(const Vector<int>& values);
    static Vec2 minmax(const Vector<double>& values);
    static double mean(const Vector<double>& values);
    static double mean(const Vector<int>& values);
    static double mean(const Vector<uint>& values);
    static Vec2 mean(const Vector<Vec2>& values);
    static double stddev(const Vector<double>& values);
    static double stddev(const Vector<int>& values);
    static double stddev(const Vector<uint>& values);
    static Vec2 meanstddev(const Vector<double>& values);
    static Vec2 meanstddev(const Vector<int>& values);
    static Vec2 meanstddev(const Vector<uint>& values);
    static VecN<4> minmeanstddevmax(const Vector<double>& values);
    static VecN<4> minmeanstddevmax(const Vector<int>& values);
    static VecN<4> minmeanstddevmax(const Vector<uint>& values);
    static double softmin(const Vector<double>& values);
    static double softmax(const Vector<double>& values);
    static double median(const Vector<double>& values);
    static Vector<int> histogram(const Vector<double>& list, uint bins=100, double min=0, double max=1);

    static int randomInt(int low=0, int high=RAND_MAX);
    static double randomNumber();
    static double randomNumber(double low, double high);
    static double uniformSample();
    static double uniformSample(double low, double high);
    static double normalSample();
    static double normalSample(double mean, double stddev);
};

extern Statistics statistics;

#endif /* STATISTICS_H_ */

