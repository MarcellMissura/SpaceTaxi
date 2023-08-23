#include "State.h"
#include "Config.h"
#include <QFile>
#include <QDataStream>

/*
 * The State is a blackboard for data sharing between the components of the framework.
 * It is a globally accessible object (read and write) with built in introspection
 * capabilities.
 *
 * The primary function of the state is to provide a central and easily accessible place
 * for the storage of variables and objects, such as sensor values, maps, debug values and
 * motor commands. The state has globally accessible public members, because it is needed
 * and used everywhere in the framework. Anywhere in the code, just include "blackboard/State.h"
 * and you can immediately access the State object. For example, try state.time to access the
 * current time. All relevant state members are public and there are no cumbersome getters and
 * setters. The typical use case for the state is that a robot interface reads sensor data and
 * writes them into state, a robot control reads those data and computes intermediate results,
 * for example a map, and writes motor commands into the state. The gui reads data from the
 * State and visualizes it on the screen and the robot interface reads the motor commands and
 * sends them to the robot.
 *
 * To add your own custom member to the state, the header, the constructor, and the init() method
 * have to be modified. The header to declare the variable, the constructor to initialize it, and
 * the init() method to register a human readible name with a reference to the member variable
 * for automatic plotting in the graph widget.
 *
 * The state object is not thread safe as such. Access to the members of the state object is not
 * mutexed or synchronized in any way. Most of the time this is sufficient and works perfectly
 * fine. If you need thread safety, you can use the setMember() and getMember() functions instead
 * of the direct access.
 *
 * The state object can keep a history of itself. Upon calling state.buffer(), the state
 * object will add a copy of itself into an internal ring buffer with bounded length. You can
 * access a historical state object by index. For example state[0].frameId tell you the id of
 * the oldest state object on record. state[state.size()-1] accesses the latest state object
 * on record. Note that there is "the state object", which you can access by using state.something,
 * and there is the state history that you can access using an index such as state[0].something.
 * The few private members the state object needs for keeping the history and to implement some
 * introspection features are static so that they don't get copied when the state is buffered
 * into history. You can also declare members as static in order to avoid the buffering of certain
 * members, e.g. a point cloud or a camera image, that can fill up your memory really fast. The
 * state history can be save()-ed to a file. When calling state.save(), you should find an action
 * in the menu in any instance of this framework, the entire state history will be written
 * into data/statehistory.dat. At a later time, you can load the state history again with
 * state.load() and step through the state history with the arrow keys for offline processing.
 *
 * The state history and the introspection capabilities are used by the visualization components
 * of the framework.
 */

State state;

// These members are static so that buffering into history does not create copies.
QMutex State::mutex(QMutex::Recursive);
int State::stateIndexOffset = 0;
int State::bufferOffset = 0;
QStringList State::memberNames;
Vector<quint64> State::memberOffsets;
Vector<QString> State::memberTypes;
Vector<State> State::history;
World State::world;

// In the constructor, members should be initialized.
State::State()
{
    stop = false;

    frameId = 0;
    time = 0;
    realTime = 0;
    iterationTime = 0;
    executionTime = 0;
    senseTime = 0;
    actTime = 0;
    pathTime = 0;
    trajectoryTime = 0;
    bufferTime = 0;
    drawTime = 0;

    aasExpansions = 0;
    aasClosed = 0;
    aasCollided = 0;
    aasOpen = 0;
    aasProcessed = 0;
    aasFinished = 0;
    aasDried = 0;
    aasDepth = 0;
    aasScore = 0;
    aasPathFails = 0;
}

State::~State()
{

}

// The init() method is called after construction of the state object.
// Here, all state object members are registered to build a descriptor meta
// structure that allows index and key based access to the member values.
// This enables the framework to place the state member in a tree structure
// and plot them in the graph widget. If you don't want to plot a certain
// state member, there is no need to register it.
void State::init()
{
    registerMember("time", &time);

    registerMember("time.iterationTime", &iterationTime);
    registerMember("time.executionTime", &executionTime);
    registerMember("time.buffer", &bufferTime);
    registerMember("time.draw", &drawTime);
    registerMember("time.sense", &senseTime);
    registerMember("time.act", &actTime);
    registerMember("time.pathTime", &pathTime);
    registerMember("time.trajectory", &trajectoryTime);

    registerMember("uniAgent.x", &uniTaxi.x);
    registerMember("uniAgent.y", &uniTaxi.y);
    registerMember("uniAgent.th", &uniTaxi.theta);
    registerMember("uniAgent.v", &uniTaxi.v);
    registerMember("uniAgent.w", &uniTaxi.w);
    registerMember("uniAgent.a", &uniTaxi.a);
    registerMember("uniAgent.b", &uniTaxi.b);
    registerMember("uniAgent.closes", &uniTaxi.closes);
    registerMember("uniAgent.collisions", &uniTaxi.collisions);
    registerMember("uniAgent.score", &uniTaxi.score);
    registerMember("uniAgent.stucks", &uniTaxi.stucks);

    registerMember("STAA*.expansions", &aasExpansions);
    registerMember("STAA*.processed", &aasProcessed);
    registerMember("STAA*.closed", &aasClosed);
    registerMember("STAA*.collided", &aasCollided);
    registerMember("STAA*.depth", &aasDepth);
    registerMember("STAA*.open", &aasOpen);
    registerMember("STAA*.dried", &aasDried);
    registerMember("STAA*.finished", &aasFinished);
    registerMember("STAA*.score", &aasScore);
    registerMember("STAA*.pathfails", &aasPathFails);

//	qDebug() << memberNames;
//	qDebug() << memberTypes;
//	qDebug() << memberOffsets;
}

// Clears the state history.
void State::clear()
{
    QMutexLocker locker(&mutex);
    history.clear();
    frameId = 0;
    time = 0;
    bufferOffset = 0;
}

// Saves the entire state history to a file.
void State::save() const
{
    QMutexLocker locker(&mutex);

    QFile file("data/statehistory.dat");
    file.open(QIODevice::WriteOnly);
    QDataStream out(&file);
    for (int i = 0; i < history.size(); i++)
    {
        QByteArray ba((char *)(&(history[i])), sizeof(State));
        out << ba;
    }
    file.close();
}

// Loads the saved state history from file.
void State::load(QString fileName)
{
    QMutexLocker locker(&mutex);

    if (fileName == "")
        fileName = "statehistory.dat";
    QFile file("data/" + fileName);
    file.open(QIODevice::ReadOnly);
    QDataStream in(&file);

    history.clear();
    bufferOffset = 0;

    QByteArray ba;
    int count = 0;
    while (!in.atEnd() && count < config.bufferSize)
    {
        in >> ba;
        history << *(State*)(ba.data());
        ++count;
    }

    //history.reverse(); // when you need to revert the order

    file.close();

    // Renumber the frames just in case.
    for (uint i = 0; i < history.size(); i++)
    {
        history[i].frameId = i;
        history[i].time -= history[0].time;
    }

    if (!history.isEmpty())
        *this = history.last();
}

// Appends the current state to the state history.
// The maxLength parameter defines how many states are kept in history.
void State::buffer(int maxLength)
{
    QMutexLocker locker(&mutex);

    // No buffer size set.
    if (maxLength <= 0)
    {
        history.push_back(*this);
        return;
    }

    // The buffer is smaller than the maximum size.
    if (history.size() < maxLength)
    {
        history.push_back(*this);
        return;
    }

    // Append to ring buffer.
    history[bufferOffset] = *this;
    bufferOffset = (bufferOffset + 1) % history.size();

    // Shed the oldest states if the maxLength has been reduced.
    while (maxLength < history.size())
    {
        history.removeAt(bufferOffset);
        if (bufferOffset >= history.size())
            bufferOffset=0;
    }

    return;
}

// Writes the current state directly to file.
void State::bufferToFile() const
{
    QMutexLocker locker(&mutex);

    QFile file("data/statehistory.dat");
    file.open(QIODevice::Append);
    QDataStream out(&file);
    out << time;
    //out << timeDiff;
    //out << robot;
    file.close();
}

// Returns the amount of buffered historical state objects.
int State::size() const
{
    QMutexLocker locker(&mutex);
    return history.size();
}

// Returns a historical state object.
// i = 0 returns the oldest state in the buffer.
// i = 1 returns the state object one after that.
// To get the most recent state, request i = state.size()-1.
// Out of bounds requests will wrap.
State& State::operator[](int i)
{
    QMutexLocker locker(&mutex);

    if (history.empty())
        return *this;

    uint idx = mod(i+bufferOffset, history.size());
    return history[idx];
}

// Returns the value of the ith member of this object.
double State::operator()(int i) const
{
    return getMember(i);
}

// Returns the value of the member that was registered with the given key.
// If no member was registered with this key, 0 is returned.
// Index based access is faster than key based access.
double State::operator()(QString key) const
{
    return getMember(key);
}

// Returns the value of the ith member of this object.
double State::getMember(int i) const
{
    QMutexLocker locker(&mutex);

	if (memberTypes[i].startsWith('i'))
        return (double)(*((int*)((quint64)this+memberOffsets[i])));
	else if (memberTypes[i].startsWith('b'))
        return (double)(*((bool*)((quint64)this+memberOffsets[i])));
	else if (memberTypes[i].startsWith('f'))
        return (double)(*((float*)((quint64)this+memberOffsets[i])));
    return *((double*)((quint64)this+memberOffsets[i]));
}

// Returns the value of the member that was registered with the given key.
// If no member was registered with this key, 0 is returned.
// Index based access is faster than key based access.
double State::getMember(QString key) const
{
    QMutexLocker locker(&mutex);

	int memberIndex = 0;
	bool memberFound = false;
    while (!memberFound)
		if (memberNames[memberIndex++] == key)
			memberFound = true;

	if (memberFound)
		return getMember(memberIndex-1);

	return 0;
}

// Sets the ith member to value v.
void State::setMember(int i, double v)
{
    QMutexLocker locker(&mutex);

    if (memberTypes[i].startsWith('i'))
        *((int*)((quint64)this+memberOffsets[i])) = (int)v;
    else if (memberTypes[i].startsWith('b'))
        *((bool*)((quint64)this+memberOffsets[i])) = (bool)v;
    else if (memberTypes[i].startsWith('f'))
        *((float*)((quint64)this+memberOffsets[i])) = (float)v;
    else
        *((double*)((quint64)this+memberOffsets[i])) = (double)v;
}

// Sets the value of the member that was registered with the given key.
// If no member was registered with this key, nothing happens.
// Index based access is faster than key based access.
void State::setMember(QString key, double v)
{
    QMutexLocker locker(&mutex);

    int memberIndex = 0;
    bool memberFound = false;
    while (!memberFound)
        if (memberNames[memberIndex++] == key)
            memberFound = true;

    if (memberFound)
        setMember(memberIndex-1, v);
    else
        qDebug() << "Unable to find state member" << key;
}

// Returns the floor state index for the given time t.
// This is needed to find the state history index for a real time t for plotting.
int State::findIndex(double t)
{
    int stateIndex = stateIndexOffset;
    while (stateIndex < state.size()-1 && state[stateIndex].time <= t)
    {
        stateIndex++;
        stateIndexOffset++;
    }
    while (stateIndex > 0 && state[stateIndex-1].time > t)
    {
        stateIndex--;
        stateIndexOffset--;
    }

    return stateIndex;
}

QDebug operator<<(QDebug dbg, const State &v)
{
	for (int i = 0; i < v.memberNames.length(); i++)
		dbg.nospace() << "id:" << i << ", key:" << v.memberNames[i] << ", val:" << v.getMember(i) << "\n";
	return dbg.space();
}
