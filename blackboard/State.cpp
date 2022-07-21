#include "State.h"
#include "globals.h"
#include "Config.h"
#include <QMutexLocker>
#include <QFile>
#include <QDataStream>

/*
 * The State is a globally accessible, bufferable object with built in introspection
 * capabilities. It's a blackboard.
 *
 * The primary function of the state is to provide a central and easily accessible place
 * for the storage of variables and objects, such as raw sensor values, higher level
 * sensors values, outputs of behaviors, and debug values. The state has globally
 * accessible public members, because it is needed and used everywhere in the framework.
 * Anywhere in the code, just write for example state.time to access the current time,
 * providing the state header is included with #include "framework/State.h". Since all
 * relevant state members are public, we can abandon the cumbersome maintenance of getters
 * and setters. The member of the sate change quite frequently anyways. The typical use
 * case for the state is of course data sharing. Most frequently a main control loop would
 * execute some kind of a controller or simulation and write data into the state, which is
 * then read and visualized by a gui component.
 *
 * The state object is not thread safe. Access to the state object is not mutexed or
 * synchronized in any way. The reason why this still works is due to the following facts.
 * 1. No state member is written from two different threads. The main control loop writes
 * and sometimes reads into the state while the GUI only ever reads state members.
 * Therefore, it cannot happen that two threads try to concurrently write the same state
 * member, where there result would be undefined. On x86 read and write operations on
 * up to 8 aligned bytes (such as doubles) are atomic. That means that a half
 * written double does not exist, which would be an undefined number that can do bad
 * things to your robot. 2. It is possible, however, that structures such a vectors or images
 * are partially overwritten by one thread, while another thread is reading it. Atomic doubles
 * prevent the appearance of corrupt numbers, but it can happen that for example half of a
 * vector is read, then the vector is overwritten with new data, and then the second half is
 * read, which means one would have a slightly corrupted image. As we expect sensor values
 * to change continuously, and sensor values are the only input to the system after all, this
 * is usually not a problem that would result in system failure.
 *
 * The state object is bufferable. This means that upon calling state.buffer(), the state
 * object will add a copy of itself into an internal history with bounded length. To access
 * a historical state object use for example state[-1].supportLegSign, which would tell you
 * on which leg the robot was standing one buffered state before the current one. You can
 * provide positive numbers as well, they are the same as using negative ones. Of course,
 * the state is buffered once in each cycle. The few private members the state object needed
 * for keeping the history and to implement the introspection features are static so that
 * they don't get copied when the state is buffered into history.
 *
 * The introspection capabilities are used for visualization. All state members can be displayed
 * as time series on a graph. To select which variables you want to see, you want to click on
 * their names, so they need to be associated with a string (key). Unlike Java, C++ does not
 * have a native introspection interface, so the class meta data (keys) have to be provided
 * manually. For each state member, the header, the constructor, and the init() method have to
 * be modified. The header to declare the variable, the constructor to initialize it, and the
 * init() method to register a key and a reference to the member variable. Of course it is
 * displeasing having to go through these places just to create a state member, but at least it's
 * all in one object and there is an easy pattern to follow. On the upside, what you get for the
 * extra effort is index based and key based read access to the state members, such as state(0),
 * which would be the program time, or state("txAction.pose.leftLegPose.hip.y") for the commanded
 * position of the left hip y servo. These access methods are used by the GUI to automatically
 * generate visualizations of the state. The introspection interface and the history can be
 * combined. Here are some examples.
 *
 * state[-10](4) gives you the step counter from 10 cycles ago, because the step counter is the 4th
 * member of State counting from 0.
 *
 * state[11]("fusedAngle.x") gives you the lateral fused angle 11 cycles ago.
 *
 * But the best way is to simply refer to the state members directly such as state.time, or
 * state[10].debug if you wanted to read the debug values ten frames ago.
 */

State state;

// These members are static so that buffering into history does not create copies.
QMutex State::mutex;
int State::stateIndexOffset = 0;
QStringList State::memberNames;
QList<quint64> State::memberOffsets;
QList<QString> State::memberTypes;
QList<State> State::history;

World State::world;

// In the constructor members should be initialized where needed.
State::State()
{
    debug = 0;
    stop = 0;

    frameId = 0;
    time = 0;
    realTime = 0;

    iterationTime = 0;
    executionTime = 0;
    pathTime = 0;
    senseTime = 0;
    actTime = 0;
    trajectoryTime = 0;

    aasExpansions = 0;
    aasClosed = 0;
    aasCollided = 0;
    aasOpen = 0;
    aasProcessed = 0;
    aasFinished = 0;
    aasDried = 0;
}

State::~State()
{

}

// The init() method should be called after construction of the state object.
// Here, all state object members are registered to build a descriptor meta
// structure that allows index and key based access to the member values.
// If you don't want to see a certain state member on the gui, there is no
// need to register it.
void State::init()
{
    registerMember("time", &time);
    registerMember("debug", &debug);

    registerMember("time.iterationTime", &iterationTime);
    registerMember("time.executionTime", &executionTime);
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
    registerMember("uniAgent.isTooClose", &uniTaxi.isTooClose);
    registerMember("uniAgent.closes", &uniTaxi.closes);
    registerMember("uniAgent.collisions", &uniTaxi.collisions);
    registerMember("uniAgent.score", &uniTaxi.score);
    registerMember("uniAgent.stucks", &uniTaxi.stucks);

    registerMember("STAA*.expansions", &aasExpansions);
    registerMember("STAA*.closed", &aasClosed);
    registerMember("STAA*.collided", &aasCollided);
    registerMember("STAA*.open", &aasOpen);
    registerMember("STAA*.processed", &aasProcessed);
    registerMember("STAA*.dried", &aasDried);
    registerMember("STAA*.finished", &aasFinished);

//	qDebug() << memberNames;
//	qDebug() << memberTypes;
//	qDebug() << memberOffsets;
}

// Clears the state history.
void State::clear()
{
	QMutexLocker locker(&mutex);
	history.clear();
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

// Loads the passed data into the state history.
void State::load(QString fileName)
{
	QMutexLocker locker(&mutex);

	history.clear();

	if (fileName == "")
		fileName = "statehistory.dat";
	QFile file("data/" + fileName);
	file.open(QIODevice::ReadOnly);
	QDataStream in(&file);

	QByteArray ba;
	while (!in.atEnd())
	{
		in >> ba;
		history << *(State*)(ba.data());
	}

	file.close();

	if (!history.isEmpty())
		*this = history.first();
}

// Appends the current state values to the history.
void State::buffer(int maxLength)
{
    QMutexLocker locker(&mutex);
    history.push_front(*this);
    while (maxLength > 0 && history.size() > maxLength)
        history.pop_back();
}

// Returns the amount of buffered historical state objects.
int State::size() const
{
	QMutexLocker locker(&mutex);
	return history.size();
}

// Returns a historical state object.
// i = 0 returns the current state.
// i = -1 (or 1) returns the state object from the iteration before and so on.
// To get the oldest known state you must request i = (+/-)state.size().
// Yes, unlike usual arrays, this operator handles items indices from 0 to state.size()!
// If abs(i) > state.size() the oldest known state object is returned.
State& State::operator[](int i)
{
	QMutexLocker locker(&mutex);

    if (i == 0 || history.empty())
		return *this;

    i = qMin(qAbs(i), history.size()) - 1;
	return history[i];
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
//	QMutexLocker locker(&mutex);

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
//	QMutexLocker locker(&mutex);

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
// This is needed to find the state history index for a real time t.
int State::findIndex(double t)
{
    int stateIndex = qBound(1, int( double(state.size()-1) - t/config.rcIterationTime + stateIndexOffset), state.size()-1);
    while (stateIndex < state.size()-1 && state[stateIndex].time > t)
    {
        stateIndex++;
        stateIndexOffset++;
    }
    while (stateIndex > 1 && state[stateIndex-1].time <= t)
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
