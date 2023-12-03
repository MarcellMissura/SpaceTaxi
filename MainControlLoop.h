#ifndef MAINCONTROLLOOP_H
#define MAINCONTROLLOOP_H

#include "lib/util/StopWatch.h"
#include "lib/util/Timer.h"
#include <QTimer>

class MainControlLoop : public QObject
{
    Q_OBJECT

    bool running;
    StopWatch stopWatch; // for precise performance measuring
    QTimer timer; // drives the rc thread

public:

    MainControlLoop(QObject *parent = 0);
    ~MainControlLoop();

    void init();
    bool isRunning() const;

public slots:
    void start();
    void stop();
    void step();
    void reset();

};

#endif
