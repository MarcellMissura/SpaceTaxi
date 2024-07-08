#ifndef MAINWINDOW_H
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

    QSplitter* verticalSplitterTop;
    QSplitter* verticalSplitterBottom;
    QSplitter* horizontalSplitter;

    QSignalMapper mapSelectionMapper;
    QSignalMapper trajectoryPlannerSelectionMapper;
    QSignalMapper frequencySelectionMapper;
    QSignalMapper pathSelectionMapper;
    QSignalMapper trajectoryTypeSelectionMapper;
    QSignalMapper predictionTypeSelectionMapper;
    QSignalMapper heuristicSelectionMapper;

    QAction* joystickAction;
    QAction* keyboardAction;

    int cfi; // current frame index
    int fim; // frame index modifier
    bool recording;
    QTimer guiUpdateTimer; // For gui updates during recording.
    QTimer animationTimer; // For the playback of the state history.

    QString robotName;
    Joystick joystick;
    MainControlLoop mainControlLoop;
    Experimenter experimenter;

    void buildMenu();

public:
    MainWindow(QWidget *parent = 0);
    ~MainWindow();

public slots:

    void reset();
    void saveConfig();
    void loadConfig();
    void saveStateHistory();
    void loadStateHistory(QString fileName = "");
    void toggleFileBuffering();
    void saveMap();
    void loadMap();
    void exportWorld();

    void toggleJoystick();
    void toggleKeyboard();
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
    void framesBack();
    void frameForward();
    void framesForward();
    void jumpToStart();
    void jumpToEnd();
    void jumpToFrame();
    void loadFrame(int);
    void updateGui();

    void toggleMap(int mapId);

    void toggleSelectPose();
    void toggleSelectTarget();
    void toggleClearMap();
    void toggleFillMap();
    void poseSelected(const Pose2D& pose);
    void mapEdited(const Polygon& pol);

    void toggleLabels();
    void toggleBody();
    void toggleSimulationDebug();

    void toggleShowOdometry();
    void toggleLaser();
    void toggleVisibilityPolygon();
    void toggleSafetyZone();
    void toggleCostmap();
    void toggleDijkstraMap();
    void toggleLocalMap();
    void toggleRayModel();

    void toggleTargets();
    void toggleLocalVisibilityGraph();
    void toggleWorldVisibilityGraph();
    void togglePaths();

    void togglePose();    
    void toggleLineMap();
    void togglePoseGraph();
    void togglePolygonMap();
    void toggleWorldPolygons();
    void toggleNavGoals();

    void toggleFrequency(int f);
    void toggleTrajectoryControl(int d);
    void togglePredictionType(int d);
    void toggleHeuristic(int d);
    void toggleTrajectoryType(int d);
    void toggleSafetyZoneReflex();
    void toggleEmergencyBrakeReflex();
    void toggleStucknessReflex();
    void toggleTimeAbort();
    void toggleClosing();
    void toggleDynamicPath();
    void toggleKeepLineObservations();
    void toggleOdometry();
    void toggleMapUpdate();
    void toggleSlam();

    void toggleGhostMode();
    void toggleDrawing();
    void toggleTeaching();
    void clearRuleBase();

signals:
    void frameChanged();
    void progressOut(int);

protected:
    void keyPressEvent(QKeyEvent *event);
    void keyReleaseEvent(QKeyEvent *event);
    bool eventFilter(QObject *obj, QEvent *event);
};

#endif
