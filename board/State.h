#ifndef STATE_H_
#define STATE_H_

#include <QList>
#include <QStringList>
#include <QMutex>
#include <typeinfo>
#include <QDebug>
#include "World.h"
#include <QMutex>

// Represents the current state of the robot and its perception of the world.
struct State
{
    int frameId; // An all purpose frame id starting at 0 and counting up henceforth.
    bool stop; // Stops the robot control execution. Used for debugging.

    // Performance measurements.
    double time; // Current robot control time since program start. Mandatory field.
    double iterationTime; // How long did the last iteration really take? (10 ms?)
    double executionTime; // The execution time of the last rc iteration. (<< 10 ms)
    double receptionTime; // How much time between laser data?
    double drawTime; // Execution time of the draw function in ms.
    double bufferTime; // Copying into buffer time in ms.
    double senseTime; // Runtime of the sense function in ms.
    double laserTime; // Runtime of the laser data processing functions in ms.
    double localMapTime; // Runtime of the local map processing functions in ms.
    double perTime; // Runtime of the laser data processing functions in ms.
    double slamTime; // Runtime of the entire slam function in ms.
    double localizationTime; // Runtime of global and local snapping in ms.
    double mapUpdateTime; // Runtime of updating the map in ms.
    double actTime; // Copying into buffer time in ms.
    double pathTime; // Runtime of the path planning in ms.
    double trajectoryTime; // Runtime of the trajectory planning in ms.

    // Map performance values.
    double mapRam; // in MB.

    // Path performance values.
    double pathLength; // in meters

    // A* performance values.
    int aasExpansions;
    int aasClosed;
    int aasCollided;
    int aasOpen;
    int aasProcessed;
    int aasFinished;
    int aasDried;
    int aasDepth;
    int aasPathFails;
    double aasScore;

    // The whole world. :)
    static World world;

    // Copies of the taxies so that they can be buffered and plotted.
    UnicycleAgent uniTaxi;

    static QMutex bigMutex;

public:

    State();
    ~State();
    void init();
    void buffer(int maxLength = 0);
    void bufferToFile() const;
    void resetFile() const;
    void clear();
    void save() const;
    void load(QString fileName = "");
    int size() const;
    State& operator[](int i);
    double operator()(int i) const;
    double operator()(QString key) const;
    double getMember(int i) const;
    double getMember(QString key) const;
    void setMember(int i, double v);
    void setMember(QString key, double v);
    static int findIndex(double t);

private:

    // Registers a member variable for index based access.
    template <typename T>
    void registerMember(QString name, T* member)
    {
        memberNames << name;
        memberOffsets << (quint64)member - (quint64)this;
        memberTypes << QString(typeid(*member).name());
    }

    // This is for the findIndex() function to remember where it was called last.
    static int stateIndexOffset;

    // This offset is for the state history ring buffer. It points at the oldest
    // state in the state history.
    static int bufferOffset;

    // These members are static so that buffering into history does not create copies.
    static Vector<quint64> memberOffsets;
    static Vector<QString> memberTypes;
    static Vector<State> history;
    static QMutex mutex;

public:
    static QStringList memberNames; // Contains the names of the members in the right order.
};

QDebug operator<<(QDebug dbg, const State &v);

extern State state;

#endif /* STATE_H_ */

