#ifndef STATISTICS_H_
#define STATISTICS_H_

#include "Vector.h"
#include "Vec2.h"
#include "Pose2D.h"

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
    static double median(Vector<double> values);
    static const Vector<Vector<uint> >& cluster(const Vector<double>& list, double eps);
    static const Vector<Vector<uint> >& clusterSorted(const Vector<double>& list, double eps);
    static const Vector<Vector<uint> >& clusterAngles(const Vector<double>& list, double eps);
    static const Vector<Vector<uint> >& cluster(const Vector<Vec2>& list, double eps);
    static const Vector<Vector<uint> >& cluster(const Vector<Pose2D>& list, double eps);
    static const Vector<uint>& voting(const Vector<double>& list, double eps);
    static const Vector<uint>& voting(const Vector<Pose2D>& list, double eps);
    static const Vector<uint>& consensusSet(const Vector<double>& list, double eps);
    static const Vector<uint>& consensusSet(const Vector<Pose2D>& list, double eps);
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

