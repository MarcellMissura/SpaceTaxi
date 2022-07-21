#include "Joystick.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/joystick.h>
#include <QFileInfo>
#include <QDebug>

Joystick::Joystick(QObject *parent) : QObject(parent)
{
    jd = -1;
    isConnected = false;
	timer.setInterval(10);
	connect(&timer, SIGNAL(timeout()), this, SLOT(update()));

	for (int i = 0; i < numOfButtons; i++)
	{
		buttonStateBefore << false;
		buttonState << false;
	}

	for (int i = 0; i < numOfAxes; i++)
	{
		axisState << 0;
		axisStateBefore << 0;
        axisOffset << 0;
	}
}

Joystick::~Joystick()
{
    if (jd != -1)
        close(jd);
}

// Tries to detect the first connected joystick.
// Returns true on success and false if no joystick was found.
// The connected() and disconnected() signals are also emitted as appropriate.
bool Joystick::init()
{
    jd = open("/dev/input/js0", O_RDONLY | O_NONBLOCK);

    bool joy1Present = (jd != -1);
    if (!isConnected && joy1Present)
        emit connected();
    if (isConnected && !joy1Present)
        emit disconnected();
    isConnected = joy1Present;

    // Read the initial state.
    struct js_event e;
    ssize_t ret;
    while((ret = read(jd, &e, sizeof(struct js_event))) > 0)
    {
        // button press or release event
        if (e.type & JS_EVENT_BUTTON && e.number < numOfButtons)
            buttonState[e.number] = e.value ? true : false;

        // axis position changed
        else if (e.type & JS_EVENT_AXIS && e.number < numOfAxes)
            axisOffset[e.number] = (float)e.value/32767.0;
    }

    return joy1Present;
}

// Starts periodic polling of the joystick (active mode).
// This is required if the joystick is used in active mode.
void Joystick::startPolling(int ms)
{
	timer.setInterval(ms);
	timer.start(ms);
}

// Starts periodic polling of the joystick (active mode).
// This is required if the joystick is used in active mode.
void Joystick::toggleOnOff()
{
	if (timer.isActive())
		timer.stop();
	else
		timer.start();
}

// Tells you if the joystick is active (polling) or not.
bool Joystick::isActive()
{
	return timer.isActive();
}

// Polls the current joystick state. The return value is an indicator of the joystick connection state.
// Signals are automatically emitted for lost and found connections, detected button presses and releases
// and detected stick motion. Also the internal public data structures are updated (buttonState, axisState)
// for your convenience.
bool Joystick::update()
{
    if (!isConnected)
        init();

    QFileInfo checkFile("/dev/input/js0");
    if (isConnected && !checkFile.exists())
    {
        isConnected = false;
        emit disconnected();
    }

    if (!isConnected)
        return false;

    bool buttonPressDetected = false;
    bool buttonReleaseDetected = false;
    bool axisMoveDetected = false;

    struct js_event e;
    ssize_t ret;
    while((ret = read(jd, &e, sizeof(struct js_event))) > 0)
    {
        // button press or release event
        if (e.type & JS_EVENT_BUTTON && e.number < numOfButtons)
        {
            buttonState[e.number] = e.value ? true : false;
            if (e.value)
                buttonPressDetected = true;
            else
                buttonReleaseDetected = true;
        }

        // axis position changed
        else if (e.type & JS_EVENT_AXIS && e.number < numOfAxes)
        {
            axisState[e.number] = qBound(-1.0, -((double)e.value/32767.0 - axisOffset[e.number]), 1.0);
            axisMoveDetected = true;
        }
    }

    float threshold = 0.06;
	axisState[0] = qAbs(axisState[0]) > threshold ? axisState[0] : 0.0;
	axisState[1] = qAbs(axisState[1]) > threshold ? axisState[1] : 0.0;
	axisState[2] = qAbs(axisState[2]) > threshold ? axisState[2] : 0.0;
	axisState[3] = qAbs(axisState[3]) > threshold ? axisState[3] : 0.0;

    if (buttonPressDetected)
        emit buttonPressed(buttonState);
    if (buttonReleaseDetected)
        emit buttonReleased(buttonState);
    if (axisMoveDetected)
        emit joystickMoved(axisState);

	return true;
}
