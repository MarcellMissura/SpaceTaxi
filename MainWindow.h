﻿#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "ui_gui.h"
#include <QMainWindow>
#include <QSplitter>
#include "gui/GraphWidget.h"
#include "gui/ConfigWidget.h"
#include "gui/CheckBoxWidget.h"
#include "gui/GraphicsViewWidget.h"
#include "gui/GraphicsScene.h"
#include "lib/util/Joystick.h"
#include "board/State.h"
#include "board/Config.h"
#include "board/Command.h"
#include "MainControlLoop.h"
#include "Experimenter.h"

class MainWindow : public QMainWindow
{
    Q_OBJECT

    Ui::MainWindowClass ui;

    CheckBoxWidget checkboxWidget;
    GraphWidget graphWidget;
    ConfigWidget configWidget;
    GraphicsViewWidget graphicsViewWidget;
    GraphicsScene graphicsScene;

    QSignalMapper mapSelectionMapper;
    QSignalMapper trajectoryPlannerSelectionMapper;
    QSignalMapper frequencySelectionMapper;
    QSignalMapper pathSelectionMapper;
    QSignalMapper trajectoryTypeSelectionMapper;
    QSignalMapper predictionTypeSelectionMapper;
    QSignalMapper heuristicSelectionMapper;

    QSplitter* verticalSplitterTop;
    QSplitter* verticalSplitterBottom;
    QSplitter* horizontalSplitter;

    QAction* joystickAction;
    QAction* keyboardAction;
    QAction* showWorldPathAction;

    int cfi; // current frame index
    int fim; // frame index modifier
    bool recording;
    QTimer guiUpdateTimer; // For gui updates during recording.
    QTimer animationTimer; // For the playback of the state history.

    QString robotName;
    Joystick joystick;
    MainControlLoop mainControlLoop; // This is the main control loop of the robot control and the simulation.
    Experimenter experimenter;

    void buildMenu();

public:
    MainWindow(QWidget *parent = 0);
    ~MainWindow();

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
    void stop();
    void frameBack();
    void frameForward();
    void jumpToStart();
    void jumpToEnd();
    void jumpToFrame(int);
    void loadFrame(int);
    void updateGui();

    void reset();
    void saveConfig();
    void loadConfig();
    void saveStateHistory();
    void loadStateHistory(QString fileName = "");
    void exportWorld();

    void toggleJoystick();
    void toggleKeyboard();
    void toggleMap(int mapId);

    void toggleSelectPose();
    void toggleSelectTarget();
    void poseRecorded(const Pose2D& pose);

    void toggleLocalMap();
    void toggleSimulationDebug();
    void toggleTeaching();
    void clearRuleBase();
    void toggleNavGoals();
    void toggleTargets();
    void toggleLabels();
    void toggleWorldPolygons();
    void toggleWorldMap();
    void togglePaths();
    void toggleWorldVisibilityGraph();
    void toggleLocalVisibilityGraph();
    void toggleVisibilityPolygon();
    void toggleSafetyZone();
    void toggleLaser();
    void toggleCostmap();
    void toggleDijkstraMap();
    void toggleTrajectoryControl(int d);
    void togglePredictionType(int d);
    void toggleHeuristic(int d);
    void toggleTrajectoryType(int d);
    void toggleFrequency(int f);
    void toggleRayModel();
    void toggleSafetyZoneReflex();
    void toggleEmergencyBrakeReflex();
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

#endif
