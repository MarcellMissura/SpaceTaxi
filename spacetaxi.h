﻿#ifndef SPACETAXI_H
#define SPACETAXI_H

#include "ui_spacetaxi.h"
#include <QMainWindow>
#include <QSplitter>
#include "gui/GraphWidget.h"
#include "gui/ConfigWidget.h"
#include "gui/CheckBoxWidget.h"
#include "gui/GraphicsViewWidget.h"
#include "gui/GraphicsScene.h"
#include "util/Joystick.h"
#include "blackboard/State.h"
#include "blackboard/Config.h"
#include "blackboard/Command.h"
#include "MainControlLoop.h"
#include "Experimenter.h"

class SpaceTaxi : public QMainWindow
{
    Q_OBJECT

    Ui::SpaceTaxiClass ui;

    CheckBoxWidget checkboxWidget;
	GraphWidget graphWidget;
	ConfigWidget configWidget;
	GraphicsViewWidget graphicsViewWidget;
    GraphicsScene graphicsScene;

    QSignalMapper mapSelectionMapper;
    QSignalMapper pathSelectionMapper;
    QSignalMapper trajectoryPlannerSelectionMapper;
    QSignalMapper trajectoryTypeSelectionMapper;
    QSignalMapper predictionTypeSelectionMapper;
    QSignalMapper heuristicSelectionMapper;
    QSignalMapper frequencySelectionMapper;

	QSplitter* verticalSplitterTop;
	QSplitter* verticalSplitterBottom;
	QSplitter* horizontalSplitter;

	QAction* joystickAction;
    QAction* keyboardAction;
    QAction* showWorldPathAction;

    double currentStateIndex;
    double tscale;
    double tscaleBefore;
    double playDirection;
    bool recording;
    QTimer guiUpdateTimer;

    QString robotName;
    Joystick joystick;
    MainControlLoop mainControlLoop; // This is the main control loop of the robot control and the simulation.
    Experimenter experimenter;

    void buildMenu();
    
public:
    SpaceTaxi(QWidget *parent = 0);
    ~SpaceTaxi();

public slots:
	void joystickConnected();
	void joystickDisconnected();
	void joystickButtonPressed(QList<bool>);
	void joystickMoved(QList<double>);
	void configChanged();
	void topSplitterMoved();
	void bottomSplitterMoved();
	void messageIn(QString m);
	void toggleConfig();
	void toggleGraph();
	void animate();
	void record();
	void recordStart();
	void recordStop();
	void play();
	void reverse();
	void stop();
	void frameBack();
	void frameForward();
	void jumpToStart();
	void jumpToEnd();
	void jumpToFrame(int);
    void loadFrame(int);

	void reset();
	void saveConfig();
	void loadConfig();
	void saveStateHistory();
	void loadStateHistory(QString fileName = "");
    void saveWorld();
    void loadWorld();
    void exportWorld();

	void toggleJoystick();
    void toggleKeyboard();
    void toggleDebug();
    void toggleMap(int h);
    void toggleGeometricModel();
    void toggleSimulationDebug();
    void toggleTeaching();
    void clearRuleBase();
    void toggleTrajectoryTrace();
    void togglePlanTrace();
    void toggleDropOffPoints();
    void toggleTargets();
    void toggleWorldObstacles();
    void toggleWorldMap();
    void toggleExpandedWorldMap();
    void toggleWorldPath();
    void toggleWorldVisibilityGraph();
    void toggleLocalVisibilityGraph();
    void toggleSensedGrid();
    void toggleDijkstraMap();
    void togglePathPlanner(int d);
    void toggleTrajectoryControl(int d);
    void togglePredictionType(int d);
    void toggleHeuristic(int d);
    void toggleTrajectoryType(int d);
    void toggleFrequency(int f);
    void toggleRayModel();
    void toggleForceFieldReflex();
    void toggleStucknessReflex();
    void toggleTimeAbort();
    void toggleClosing();
    void toggleDynamicPath();
    void toggleBody();
    void toggleGhostMode();

signals:
    void frameChanged();
    void progressOut(int);

protected:
    void keyPressEvent(QKeyEvent *event);
    void keyReleaseEvent(QKeyEvent *event);
    bool eventFilter(QObject *obj, QEvent *event);
};

#endif // SPACETAXI_H
