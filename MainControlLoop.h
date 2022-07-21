#ifndef MAINCONTROLLOOP_H
#define MAINCONTROLLOOP_H

#include "util/StopWatch.h"
#include "util/Timer.h"
#include <QTimer>

class MainControlLoop : public QObject
{
    Q_OBJECT

    bool running;
    StopWatch stopWatch; // for precise performance measuring
    QTimer timer; // drives the rc thread
    double lastUpdateTimestamp;
    double lastStartTimestamp;

public:

    MainControlLoop(QObject *parent = 0);
    ~MainControlLoop();

    void init();
    bool isRunning();

public slots:
    void start();
    void stop();
    void step();
    void reset();

signals:
    void messageOut(QString);
    void configChangedOut();
};

#endif
