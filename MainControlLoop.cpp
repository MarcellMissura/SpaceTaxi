#include "MainControlLoop.h"
#include "blackboard/State.h"
#include "blackboard/Config.h"
#include "blackboard/Command.h"
#include "util/Statistics.h"
#include <QDebug>
#include <QGraphicsScene>

// The main control loop is the main thread of the framework.
// It acts on the world object in the state by periodically calling its step() function.
// A high precision timer drives the main thread by periodically calling the step
// function of the main control loop in a seperate thread. The world is located in the
// state for convenience reasons. This way, the world gets buffered into the state
// history, and global access is possible to the parts of the world through state.world.

MainControlLoop::MainControlLoop(QObject *parent) : QObject(parent)
{
    // Connect the internal timer.
    // It must be a direct connection so that the step() function is called in the thread of the timer.
    connect(&timer, SIGNAL(timeout()), this, SLOT(step()), Qt::DirectConnection);

    running = false;
    lastUpdateTimestamp = 0;
    lastStartTimestamp = 0;
}

MainControlLoop::~MainControlLoop()
{
    stop();
}

void MainControlLoop::init()
{
    state.world.init();
    state.uniTaxi.init(Vec2());
}

void MainControlLoop::reset()
{
    state.world.reset();
}

// Starts the main control loop.
void MainControlLoop::start()
{
	running = true;
    timer.start((int)((config.rcIterationTime/config.speedUp)*1000));
	lastStartTimestamp = stopWatch.programTime();
}

// Stops the main control loop.
void MainControlLoop::stop()
{
	running = false;
	timer.stop();
}

// The main loop of the game. It's periodically called by the timer.
void MainControlLoop::step()
{
//    if (!running)
//        return;

    stopWatch.start();

    // Measure how much real time passed since the last tick.
    state.time += config.rcIterationTime;
    state.realTime = stopWatch.time();
    state.iterationTime = (state.realTime - lastUpdateTimestamp)*1000;
    lastUpdateTimestamp = state.realTime;
    state.frameId++;

    // Step the world.
    state.world.step();
    if (state.stop > 0)
        stop();

    // Buffer the state into history.
    if (state.world.unicycleAgents.size() > 0)
        state.uniTaxi = state.world.unicycleAgents[0];
    state.buffer(config.bufferSize);

    // Measure execution time.
    state.executionTime = stopWatch.elapsedTimeMs();
    //qDebug() << state.executionTime << state.trajectoryTime;
}
