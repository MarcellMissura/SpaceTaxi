#include "Statistics.h"
#include <QTime>
#include <algorithm>

Statistics statistics;
uint Statistics::seed;
Statistics::Statistics()
{
    setSeed(0);
}

// Initializes the random number generator.
// Warning! The RNG is not thread safe! If you are using the Statistics package
// in a different thread from the main, you need to call Statistics::init() explicitly.
void Statistics::init()
{
    QTime time = QTime::currentTime();
    seed = (uint)time.msec();
    srand(seed);
}

// Seeds the random number generator.
void Statistics::setSeed(uint s)
{
    seed = s;
    srand(seed);
}

// Returns the seed of the random number generator.
uint Statistics::getSeed()
{
    return seed;
}

// Reinitilaizes the random number generator with the same seed.
void Statistics::reSeed()
{
    srand(seed);
}

// Returns the softmin of the values in the list.
double Statistics::softmin(const Vector<double>& list)
{
	if (list.isEmpty())
		return 0;

    double softmin = 0;
    double totalWeight = 0;
    for (uint i=0; i<list.size(); i++)
    {
        double w = exp(-list[i]);
        totalWeight += w;
        softmin += w*list[i];
    }
    softmin /= totalWeight;
    return softmin;
}

// Returns the sofmax of the values in the list.
double Statistics::softmax(const Vector<double>& list)
{
    if (list.isEmpty())
        return 0;

    double softmax = 0;
    double totalWeight = 0;
    for (uint i=0; i<list.size(); i++)
    {
        double w = exp(list[i]);
        totalWeight += w;
        softmax += w*list[i];
    }
    softmax /= totalWeight;
    return softmax;
}

// Returns the minimum of the values in the list.
double Statistics::min(const Vector<double>& list)
{
    if (list.isEmpty())
        return 0;

    double min = list[0];
    for (uint i = 0; i < list.size(); i++)
        min = qMin(min, list[i]);
    return min;
}

// Returns the minimum of the values in the list.
uint Statistics::min(const Vector<uint>& list)
{
    if (list.isEmpty())
        return 0;

    uint min = list[0];
    for (uint i = 0; i < list.size(); i++)
        min = qMin(min, list[i]);
    return min;
}

// Returns the minimum of the values in the list.
int Statistics::min(const Vector<int>& list)
{
    if (list.isEmpty())
        return 0;

    int min = list[0];
    for (uint i = 0; i < list.size(); i++)
        min = qMin(min, list[i]);
    return min;
}

// Returns the maximum of the values in the list.
double Statistics::max(const Vector<double>& list)
{
    if (list.isEmpty())
        return 0;

    double max = list[0];
    for (uint i = 0; i < list.size(); i++)
        max = qMax(list[i], max);
    return max;
}

// Returns the maximum of the values in the list.
uint Statistics::max(const Vector<uint> &list)
{
    if (list.isEmpty())
        return 0;

    uint max = list[0];
    for (uint i = 0; i < list.size(); i++)
        max = qMax(list[i], max);
    return max;
}

// Returns the maximum of the values in the list.
int Statistics::max(const Vector<int> &list)
{
    if (list.isEmpty())
        return 0;

    int max = list[0];
    for (uint i = 0; i < list.size(); i++)
        max = qMax(list[i], max);
    return max;
}

// Returns the mean of the values in the list.
double Statistics::mean(const Vector<double>& list)
{
    double mean = 0;
    for (uint i = 0; i < list.size(); i++)
        mean += list[i];
    mean /= list.size();
    return mean;
}

// Returns the mean of the values in the list.
double Statistics::mean(const Vector<int>& list)
{
    double mean = 0;
    for (uint i = 0; i < list.size(); i++)
        mean += list[i];
    mean /= list.size();
    return mean;
}

// Returns the mean of the values in the list.
double Statistics::mean(const Vector<uint>& list)
{
    double mean = 0;
    for (uint i = 0; i < list.size(); i++)
        mean += list[i];
    mean /= list.size();
    return mean;
}

// Returns the mean of the values in the list.
Vec2 Statistics::mean(const Vector<Vec2>& list)
{
    Vec2 mean;
    for (uint i = 0; i < list.size(); i++)
        mean += list[i];
    mean /= list.size();
    return mean;
}

// Returns the mean of the values in the list.
double Statistics::stddev(const Vector<double>& list)
{
    double m = mean(list);
    double stddev = 0;

    for (uint i = 0; i < list.size(); i++)
        stddev += (list[i]-m)*(list[i]-m);
    stddev = sqrt(stddev/(list.size()-1)); // http://www.visiondummy.com/2014/03/divide-variance-n-1/

    return stddev;
}

// Returns the mean of the values in the list.
double Statistics::stddev(const Vector<int>& list)
{
    double m = mean(list);
    double stddev = 0;

    for (uint i = 0; i < list.size(); i++)
        stddev += (list[i]-m)*(list[i]-m);
    stddev = sqrt(stddev/(list.size()-1)); // http://www.visiondummy.com/2014/03/divide-variance-n-1/

    return stddev;
}

// Returns the mean of the values in the list.
double Statistics::stddev(const Vector<uint>& list)
{
    double m = mean(list);
    double stddev = 0;

    for (uint i = 0; i < list.size(); i++)
        stddev += (list[i]-m)*(list[i]-m);
    stddev = sqrt(stddev/(list.size()-1)); // http://www.visiondummy.com/2014/03/divide-variance-n-1/

    return stddev;
}

// Returns the minimum (in vec.x) and the maximum (in vec.y) of the values in the list.
Vec2 Statistics::minmax(const Vector<double>& list)
{
	if (list.isEmpty())
        return Vec2();

	double min = list[0];
	double max = list[0];
    for (uint i = 0; i < list.size(); i++)
	{
		min = qMin(list[i], min);
		max = qMax(max, list[i]);
	}
    return Vec2(min, max);
}

// Returns the mean (in x) and the standard deviation (in y) of the values in the list.
Vec2 Statistics::meanstddev(const Vector<double>& list)
{
    double m = mean(list);
    double stddev = 0;

    for (uint i = 0; i < list.size(); i++)
        stddev += (list[i]-m)*(list[i]-m);
    stddev = sqrt(stddev/(list.size()-1)); // http://www.visiondummy.com/2014/03/divide-variance-n-1/

    return Vec2(m, stddev);
}

// Returns the mean (in x) and the standard deviation (in y) of the values in the list.
Vec2 Statistics::meanstddev(const Vector<int>& list)
{
    double m = mean(list);
    double stddev = 0;

    for (uint i = 0; i < list.size(); i++)
        stddev += (list[i]-m)*(list[i]-m);
    stddev = sqrt(stddev/(list.size()-1)); // http://www.visiondummy.com/2014/03/divide-variance-n-1/

    return Vec2(m, stddev);
}

// Returns the mean (in x) and the standard deviation (in y) of the values in the list.
Vec2 Statistics::meanstddev(const Vector<uint>& list)
{
    double m = mean(list);
    double stddev = 0;

    for (uint i = 0; i < list.size(); i++)
        stddev += (list[i]-m)*(list[i]-m);
    stddev = sqrt(stddev/(list.size()-1)); // http://www.visiondummy.com/2014/03/divide-variance-n-1/

    return Vec2(m, stddev);
}

// Returns the min, the mean, the standard deviation, and the max of the values in the list.
VecN<4> Statistics::minmeanstddevmax(const Vector<double>& list)
{
    double min_ = min(list);
    double max_ = max(list);
    double m = mean(list);
    double stddev = 0;

    for (uint i = 0; i < list.size(); i++)
        stddev += (list[i]-m)*(list[i]-m);
    stddev = sqrt(stddev/(list.size()-1)); // http://www.visiondummy.com/2014/03/divide-variance-n-1/

    return VecN<4>(min_, m, stddev, max_);
}

// Returns the min, the mean, the standard deviation, and the max of the values in the list.
VecN<4> Statistics::minmeanstddevmax(const Vector<int>& list)
{
    double min_ = min(list);
    double max_ = max(list);
    double m = mean(list);
    double stddev = 0;

    for (uint i = 0; i < list.size(); i++)
        stddev += (list[i]-m)*(list[i]-m);
    stddev = sqrt(stddev/(list.size()-1)); // http://www.visiondummy.com/2014/03/divide-variance-n-1/

    return VecN<4>(min_, m, stddev, max_);
}

// Returns the min, the mean, the standard deviation, and the max of the values in the list.
VecN<4> Statistics::minmeanstddevmax(const Vector<uint>& list)
{
    double min_ = min(list);
    double max_ = max(list);
    double m = mean(list);
    double stddev = 0;

    for (uint i = 0; i < list.size(); i++)
        stddev += (list[i]-m)*(list[i]-m);
    stddev = sqrt(stddev/(list.size()-1)); // http://www.visiondummy.com/2014/03/divide-variance-n-1/

    return VecN<4>(min_, m, stddev, max_);
}

// Generates a 1D histogram over the list with bins number of bins
// over the interval given by [min, max]. The returned vector is a
// counter for each bin indicating how many of the points in the
// list land in the respective bin.
Vector<int> Statistics::histogram(const Vector<double>& list, uint bins, double min, double max)
{
    Vector<int> hist(bins);
    if (list.isEmpty())
		return hist;

    if (min == max)
    {
        min = list[0];
        max = list[0];
        for (uint i = 1; i < list.size(); i++)
        {
            min = qMin(list[i], min);
            max = qMax(max, list[i]);
        }
    }

    for (uint i = 0; i < list.size(); i++)
	{
        uint idx = qBound((uint)0, (uint)qRound((bins-1)*(list[i]-min)/(max-min)), (uint)hist.size()-1);
        hist[idx]++;
	}

    return hist;
}

// Returns a vector of integers, one for every element in list. The integers indicate
// the number of neighbours in list that fall within a distance of eps.
const Vector<uint>& Statistics::voting(const Vector<double>& list, double eps)
{
    //qDebug() << "voting" << list;

    static Vector<uint> votes;
    votes.resize(list.size());
    votes.fill(0);
    for (uint i = 0; i < list.size(); i++)
    {
        for (uint j = i+1; j < list.size(); j++)
        {
            if (fabs(list[i]-list[j]) < eps)
            {
                votes[i]++;
                votes[j]++;
            }
        }
    }

    return votes;
}

// Returns a vector of integers, one for every element in list. The integers indicate
// the number of neighbours in list that fall within a distance of eps.
const Vector<uint>& Statistics::voting(const Vector<Pose2D> &list, double eps)
{
    //qDebug() << "voting" << list;

    static Vector<uint> votes;
    votes.resize(list.size());
    votes.fill(0);
    for (uint i = 0; i < list.size(); i++)
    {
        for (uint j = i+1; j < list.size(); j++)
        {
            if ((list[i]-list[j]).norm() < eps)
            {
                votes[i]++;
                votes[j]++;
                //qDebug() << i << j << list[i] << list[j] << "voted for each other." << votes[i] << votes[j] << "eps:" << (list[i]-list[j]).norm();
            }
        }
    }

    return votes;
}

// Returns a vector of indices of members in list that are in the largest
// consense group with each other. This is computed by counting votes for
// each element of list counting neighbours that are within a distance of
// eps. The most voted member(s) and all their voters are returned as a
// vector of indices of list.
const Vector<uint>& Statistics::consensusSet(const Vector<double>& list, double eps)
{
    //qDebug() << "voting" << list;

    uint mostVotes = 0;
    static Vector<uint> mostVotesIdx;
    static Vector<uint> votes;
    mostVotesIdx.clear();
    votes.resize(list.size());
    votes.fill(0);
    for (uint i = 0; i < list.size(); i++)
    {
        for (uint j = i+1; j < list.size(); j++)
        {
            if (fabs(list[i]-list[j]) < eps)
            {
                votes[i]++;
                votes[j]++;
            }
        }

        if (votes[i] > mostVotes)
        {
            mostVotes = votes[i];
            mostVotesIdx.clear();
        }

        if (votes[i] >= mostVotes)
            mostVotesIdx << i;
    }

    //qDebug() << "votes" << votes;
    //qDebug() << "mostVotesIdx" << mostVotesIdx;

    uint ms = mostVotesIdx.size();
    int closed[list.size()];
    memset(closed, 0, list.size());
    for (uint k = 0; k < ms; k++)
        closed[mostVotesIdx[k]] = 1;
    for (uint k = 0; k < ms; k++)
    {
        uint i = mostVotesIdx[k];
        for (uint j = 0; j < list.size(); j++)
        {
            if (closed[j] == 1)
                continue;

            if (fabs(list[i]-list[j]) < eps)
            {
                mostVotesIdx << j;
                closed[j] = 1;
            }
        }
    }

    return mostVotesIdx;
}

// Returns a vector of indices of members in list that are in the largest
// consense group with each other. This is computed by counting votes for
// each element of list counting neighbours that are within a distance of
// eps. The most voted member(s) and all their voters are returned as a
// vector of indices of list.
const Vector<uint>& Statistics::consensusSet(const Vector<Pose2D> &list, double eps)
{
    //qDebug() << "voting" << list;

    uint mostVotes = 0;
    static Vector<uint> mostVotesIdx;
    static Vector<uint> votes;
    mostVotesIdx.clear();
    votes.resize(list.size());
    votes.fill(0);
    for (uint i = 0; i < list.size(); i++)
    {
        for (uint j = i+1; j < list.size(); j++)
        {
            if ((list[i]-list[j]).norm() < eps)
            {
                votes[i]++;
                votes[j]++;
            }
        }

        if (votes[i] > mostVotes)
        {
            mostVotes = votes[i];
            mostVotesIdx.clear();
        }

        if (votes[i] >= mostVotes)
            mostVotesIdx << i;
    }

    //qDebug() << "votes" << votes;
    //qDebug() << "mostVotesIdx" << mostVotesIdx;

    uint ms = mostVotesIdx.size();
    int closed[list.size()];
    memset(closed, 0, list.size());
    for (uint k = 0; k < ms; k++)
        closed[mostVotesIdx[k]] = 1;
    for (uint k = 0; k < ms; k++)
    {
        uint i = mostVotesIdx[k];
        for (uint j = 0; j < list.size(); j++)
        {
            if (closed[j] == 1)
                continue;

            if ((list[i]-list[j]).norm() < eps)
            {
                //qDebug() << i << "collects" << j;
                mostVotesIdx << j;
                closed[j] = 1;
            }
        }
    }

    return mostVotesIdx;
}

// Returns the median of the values in the list.
double Statistics::median(Vector<double> list)
{
    list.sort();
    return list[list.size()/2];
}

// Clusters a vector of doubles with the DBScan algorithm using eps as the distance threshold.
// A vector of vectors is returned, one vector for each cluster containing the *index* of the
// element in the list belonging to this cluster.
const Vector<Vector<uint> >& Statistics::cluster(const Vector<double>& list, double eps)
{
    // Wouldn't this be much faster by sorting? N log N?

    bool clustered[list.size()];
    for (uint i = 0; i < list.size(); i++)
        clustered[i] = false;

    static Vector<uint> N; // neighbourhood queue
    static Vector<uint> cluster; // current cluster in the making
    static Vector<Vector<uint> > clusters; // resulting clusters
    N.clear();
    cluster.clear();
    clusters.clear();
    for (uint i = 0; i < list.size(); i++)
    {
        if (clustered[i])
            continue;

        // Begin a cluster with item i.
        cluster.clear();
        cluster << i;
        clustered[i] = true;

        // Queue all item i's eps neighbours (except itself) into the neighbourhood queue.
        N.clear();
        for (uint j = i+1; j < list.size(); j++)
            if (!clustered[j] && fabs(list[j]-list[i]) < eps)
                N << j;

        for (uint k = 0; k < N.size(); k++)
        {
            if (clustered[N[k]])
                continue;

            clustered[N[k]] = true;
            cluster << N[k];
            for (uint j = 0; j < list.size(); j++)
                if (!clustered[j] && fabs(list[j]-list[N[k]]) < eps)
                    N << j;
        }

        clusters << cluster;
    }

    return clusters;
}

// Clusters a vector of doubles with a fast sweep algorithm in O(N log N).
// A vector of vectors is returned, one vector for each cluster containing the *index* of the
// element in the list belonging to this cluster.
const Vector<Vector<uint> >& Statistics::clusterSorted(const Vector<double>& list, double eps)
{
    //qDebug() << "clustering" << list;

    static Vector<uint> cluster; // current cluster in the making
    static Vector<Vector<uint> > clusters; // resulting clusters

    clusters.clear();
    if (list.isEmpty())
        return clusters;

    static Vector<double> local;
    local = list;
    local.sort();
    uint clusterRefIdx = 0;
    cluster << local[0];
    for (uint i = 1; i < local.size(); i++)
    {
        if (fabs(local[i]-local[clusterRefIdx]) > eps)
        {
            clusters << cluster;
            cluster.clear();
            clusterRefIdx = i;
        }
        cluster << i;
    }
    clusters << cluster;

    return clusters;
}

// Clusters a vector of angles with the DBScan algorithm using eps as the distance threshold.
// A vector of vectors is returned, one vector for each cluster containing the *index* of the
// element in the list belonging to this cluster.
const Vector<Vector<uint> >& Statistics::clusterAngles(const Vector<double>& list, double eps)
{
    bool clustered[list.size()];
    for (uint i = 0; i < list.size(); i++)
        clustered[i] = false;

    static Vector<uint> N; // neighbourhood queue
    static Vector<uint> cluster; // current cluster in the making
    static Vector<Vector<uint> > clusters; // resulting clusters
    N.clear();
    cluster.clear();
    clusters.clear();
    for (uint i = 0; i < list.size(); i++)
    {
        if (clustered[i])
            continue;

        // Begin a cluster with item i.
        cluster.clear();
        cluster << i;
        clustered[i] = true;

        // Queue all item i's eps neighbours (except itself) into the neighbourhood queue.
        N.clear();
        for (uint j = i+1; j < list.size(); j++)
        {
            double d = fabs(list[j]-list[i]);
            if (!clustered[j] && (d < eps || fabs(d-PI) < eps))
            {
//                qDebug() << j << list[j] << "pushed.";
                N << j;
            }
        }

        for (uint k = 0; k < N.size(); k++)
        {
            if (clustered[N[k]])
                continue;

            clustered[N[k]] = true;
            cluster << N[k];
//            qDebug() << N[k] << list[N[k]] << "popped.";
            for (uint j = 0; j < list.size(); j++)
            {
                double d = fabs(list[j]-list[N[k]]);
//                qDebug() << "trying" << j << list[j] << "and" << N[k] << list[N[k]] << "d:" << d << fabs(d-PI);
                if (!clustered[j] && (d < eps || fabs(d-PI) < eps))
                {
//                    qDebug() << j << list[j] << "pushed.";
                    N << j;
                }
            }
        }

        clusters << cluster;
    }

    return clusters;
}

// Clusters a vector of Vec2 with the DBScan algorithm using eps as the distance threshold.
// A vector of vectors is returned, one vector for each cluster containing the *index* of the
// element in list belonging to this cluster.
const Vector<Vector<uint> > &Statistics::cluster(const Vector<Vec2>& list, double eps)
{
    bool clustered[list.size()];
    for (uint i = 0; i < list.size(); i++)
        clustered[i] = false;

    //qDebug() << "clustering" << list;

    static Vector<Vector<uint> > clusters;
    static Vector<uint> cluster;
    static Vector<uint> N;
    clusters.clear();
    cluster.clear();
    N.clear();
    for (uint i = 0; i < list.size(); i++)
    {
        if (clustered[i])
            continue;

        cluster.clear();
        cluster << i;
        clustered[i] = true;

        N.clear();
        for (uint j = i+1; j < list.size(); j++)
            if (!clustered[j] && (list[j]-list[i]).norm() < eps)
                N << j;

        for (uint k = 0; k < N.size(); k++)
        {
            if (clustered[N[k]])
                continue;

            clustered[N[k]] = true;
            cluster << N[k];
            for (uint j = 0; j < list.size(); j++)
                if (!clustered[j] && (list[j]-list[N[k]]).norm() < eps)
                    N << j;
        }

        clusters << cluster;
    }

    return clusters;
}

// Returns a random integer in [0, high].
int Statistics::randomInt(int low, int high)
{
    return low + rand() % (high-low+1);
}

// Returns a sample from a uniformly distributed open (0, 1) interval.
double Statistics::randomNumber() {return uniformSample();}
double Statistics::uniformSample()
{
    double r = ((double)rand())/((double)RAND_MAX);
    return r;
}

// Returns a sample from a uniformly distributed open (low, high) interval.
double Statistics::randomNumber(double low, double high) {return uniformSample(low, high);}
double Statistics::uniformSample(double low, double high)
{
	return low + uniformSample() * (high - low);
}

// Returns a sample from a zero mean, 1 standard deviation normal distribution.
// This method could be used more efficiently if returning both, the cos and
// the sin result, or it could be entirely replaced by the ziggurat algorithm.
double Statistics::normalSample()
{
  double u1=uniformSample();
  double u2=uniformSample();
  return cos(8.*atan(1.)*u2)*sqrt(-2.*log(u1));
  //return sin(8.*atan(1.)*u1)*sqrt(-2.*log(u2));
}

// Returns a sample from a Gaussian distribution with given mean and standard deviation.
double Statistics::normalSample(double mean, double stddev)
{
    return normalSample()*stddev+mean;
}
