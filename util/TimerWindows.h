#ifndef WINDOWSMMTIMER_H
#define WINDOWSMMTIMER_H

#include "windows.h"
#include <QObject>

class SubTimer : public QObject
{
    Q_OBJECT

    int timerId;
    int timerRes;
    bool threadPrioritySet;
    bool running;

public:
    SubTimer(QObject *parent = 0);
    ~SubTimer();
    bool isActive();

public slots:
    void startTimer(int milliSeconds = 0);
	void stop();

signals:
	void timeOut();

private:
	void fire();
	static void CALLBACK PeriodicCallback(UINT uID, UINT uMsg, DWORD dwUser, DWORD dw1, DWORD dw2) {((SubTimer*)dwUser)->fire();};

};

#endif
