#include "MainControlLoop.h"
#include "board/State.h"
#include "board/Config.h"
#include "board/Command.h"

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

bool MainControlLoop::isRunning() const
{
    return running;
}

void MainControlLoop::reset()
{
    state.world.reset();
}

// Starts the main control loop.
void MainControlLoop::start()
{
    // The frequency of the main control loop is determined by the selected
    // frequency (5 Hz, 10 Hz, 20 Hz, 30 Hz) and the chosen speedUp factor. For
    // example, if 10 Hz has been selected, the control loop would fire every
    // 100 milliseconds if the speedUp = 1. At a speedUp = 2 and frequency of
    // 10 Hz, the control loop fires every 50 milliseconds and so on. The
    // simulated time is not affected by the speedup. At a frequency of 10 Hz,
    // the simulated time is always 100 ms.
    running = true;
    timer.start( (int) (1000.0/(config.simSpeedUp*command.frequency)) );
}

// Stops the main control loop.
void MainControlLoop::stop()
{
    running = false;
    timer.stop();
}

// The main control loop. It's periodically called by the timer.
void MainControlLoop::step()
{
    QMutexLocker locker(&state.bigMutex);

    state.frameId++;
    state.iterationTime = stopWatch.elapsedTimeMs();
    state.time += 1.0/command.frequency; // in seconds
    stopWatch.start();

    // Step the world.
    StopWatch sw;
    sw.start();
    state.world.step();
    if (state.stop)
        stop();
    state.executionTime = sw.elapsedTimeMs();

    // Buffer the state into history.
    sw.start();
    if (state.world.unicycleAgents.size() > 0)
        state.uniTaxi = state.world.unicycleAgents[0];
    state.buffer(config.bufferSize);
    state.bufferTime = sw.elapsedTimeMs();
    //qDebug() << state.executionTime << state.trajectoryTime;
}
