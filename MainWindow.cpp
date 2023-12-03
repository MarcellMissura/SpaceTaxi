#include "MainWindow.h"
#include <QMainWindow>
#include <QMenuBar>
#include <QDesktopWidget>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    QCoreApplication::setOrganizationName("MarcellMissura");
    QCoreApplication::setApplicationName("SpaceTaxi");

    QWidget* cw = new QWidget();
    ui.setupUi(cw); // Builds the ui file (gui.ui) into the central widget.
    setCentralWidget(cw); // The central widget has to be set because it's a QMainWindow.
    resize(QDesktopWidget().availableGeometry(this).size() * 0.75); // Resize to 75% of the screen.

    robotName = "config";

    // Initialize the state and the config.
    state.init();
    config.init();
    config.load(robotName);

    // Initialize everything else that needs to be initialized.
    mainControlLoop.init();
    graphWidget.init();
    checkboxWidget.init();
    graphicsScene.init();
    configWidget.init();
    graphicsViewWidget.setScene(&graphicsScene);
    graphicsViewWidget.init();
    //experimenter.init();

    // Build the splitter separated gui components.
    verticalSplitterTop = new QSplitter();
    verticalSplitterTop->addWidget(&configWidget);
    verticalSplitterTop->addWidget(&graphicsViewWidget);

    verticalSplitterBottom = new QSplitter();
    verticalSplitterBottom->addWidget(&checkboxWidget);
    verticalSplitterBottom->addWidget(&graphWidget);

    horizontalSplitter = new QSplitter(Qt::Vertical);
    horizontalSplitter->addWidget(verticalSplitterTop);
    horizontalSplitter->addWidget(verticalSplitterBottom);

    // Restore the gui layout from the settings.
    QSettings settings;
    verticalSplitterTop->restoreState(settings.value("verticalSplitterTop").toByteArray());
    verticalSplitterBottom->restoreState(settings.value("verticalSplitterBottom").toByteArray());
    horizontalSplitter->restoreState(settings.value("horizontalSplitter").toByteArray());

    QHBoxLayout* centralLayout = new QHBoxLayout(ui.centralWidget);
    centralLayout->setMargin(0);
    ui.centralWidget->setLayout(centralLayout);
    centralLayout->addWidget(horizontalSplitter);

    connect(verticalSplitterTop, SIGNAL(splitterMoved(int, int)), this, SLOT(topSplitterMoved()));
    connect(verticalSplitterBottom, SIGNAL(splitterMoved(int, int)), this, SLOT(bottomSplitterMoved()));


    // Build the menu bar.
    buildMenu();

    // Connect signals to slots.
    connect(this, SIGNAL(progressOut(int)), ui.frameSlider, SLOT(setValue(int)));
    connect(ui.frameSlider, SIGNAL(sliderMoved(int)), this, SLOT(jumpToFrame(int)));

    connect(&configWidget, SIGNAL(configChangedOut()), &graphicsScene, SLOT(update()));
    connect(&configWidget, SIGNAL(configChangedOut()), &graphicsViewWidget, SLOT(update()));
    connect(&checkboxWidget, SIGNAL(stateMemberStatusChanged(int, bool)), &graphWidget, SLOT(setShowCurve(int, bool)));
    connect(&graphWidget, SIGNAL(messageOut(QString)), this, SLOT(messageIn(QString)));

    // Set the joystick into polling / event based mode and connect the joystick signals.
    joystick.startPolling(10);
    connect(&joystick, SIGNAL(connected()), this, SLOT(joystickConnected()));
    connect(&joystick, SIGNAL(disconnected()), this, SLOT(joystickDisconnected()));
    connect(&joystick, SIGNAL(buttonPressed(QList<bool>)), this, SLOT(joystickButtonPressed(QList<bool>)));
    connect(&joystick, SIGNAL(joystickMoved(QList<double>)), this, SLOT(joystickMoved(QList<double>)));

    toggleGraph();
    toggleConfig();
    //showFullScreen();

    connect(&graphicsViewWidget, SIGNAL(poseRecorded(const Pose2D&)), this, SLOT(poseRecorded(const Pose2D&)));


    // Animation components.
    cfi = 0;
    fim = 0;
    recording = false;
    graphicsViewWidget.recording = false;
    guiUpdateTimer.setInterval(30);
    //guiUpdateTimer.start();
    connect(&guiUpdateTimer, SIGNAL(timeout()), this, SLOT(updateGui()));

    animationTimer.setInterval(100);
    connect(&animationTimer, SIGNAL(timeout()), this, SLOT(animate()));
}

MainWindow::~MainWindow()
{
    QSettings settings;
    settings.setValue("verticalSplitterTop", verticalSplitterTop->saveState());
    settings.setValue("verticalSplitterBottom", verticalSplitterBottom->saveState());
    settings.setValue("horizontalSplitter", horizontalSplitter->saveState());
}

// Build the menu bar.
void MainWindow::buildMenu()
{
    QMenuBar* menuBar = new QMenuBar();
    setMenuBar(menuBar);

    QMenu* fileMenu = menuBar->addMenu(tr("&File"));

    QAction* saveStateAction = fileMenu->addAction(tr("&Save State"));
    saveStateAction->setToolTip(tr("Saves the state history."));
    //saveStateAction->setShortcut(QKeySequence(tr("Ctrl+S")));
    connect(saveStateAction, SIGNAL(triggered()), this, SLOT(saveStateHistory()));

    QAction* loadStateAction = fileMenu->addAction(tr("&Load State"));
    loadStateAction->setToolTip(tr("Loads the state history."));
    //loadStateAction->setShortcut(QKeySequence(tr("Ctrl+L")));
    connect(loadStateAction, SIGNAL(triggered()), this, SLOT(loadStateHistory()));

    fileMenu->addSeparator();

    QAction* saveConfigAction = fileMenu->addAction(tr("&Save Config"));
    saveConfigAction->setToolTip(tr("Saves the config."));
    saveConfigAction->setShortcut(QKeySequence(tr("Ctrl+C")));
    connect(saveConfigAction, SIGNAL(triggered()), this, SLOT(saveConfig()));

    QAction* loadConfigAction = fileMenu->addAction(tr("&Load Config"));
    loadConfigAction->setToolTip(tr("Loads the config."));
    loadConfigAction->setShortcut(QKeySequence(tr("Ctrl+R")));
    connect(loadConfigAction, SIGNAL(triggered()), this, SLOT(loadConfig()));

    fileMenu->addSeparator();

    QAction* exportAction = fileMenu->addAction(tr("&Export Shown Data"));
    exportAction->setToolTip(tr("Exports the data currently visible in the graph widget."));
    exportAction->setShortcut(QKeySequence(tr("Ctrl+Shift+E")));
    connect(exportAction, SIGNAL(triggered()), &graphWidget, SLOT(exportData()));

    QAction* export2Action = fileMenu->addAction(tr("&Export Polygons"));
    export2Action->setToolTip(tr("Exports the currently loaded static obstacles in the World."));
    export2Action->setShortcut(QKeySequence(tr("Ctrl+Shift+P")));
    connect(export2Action, SIGNAL(triggered()), this, SLOT(exportWorld()));


    QMenu* viewMenu = menuBar->addMenu(tr("&View"));

    QAction* configViewAction = viewMenu->addAction(tr("&Config Widget"));
    configViewAction->setToolTip(tr("Toggles the config widget."));
    configViewAction->setShortcut(QKeySequence(tr("C")));
    configViewAction->setCheckable(true);
    configViewAction->setChecked(false);
    connect(configViewAction, SIGNAL(triggered()), this, SLOT(toggleConfig()));

    QAction* graphViewAction = viewMenu->addAction(tr("&Graph Widget"));
    graphViewAction->setToolTip(tr("Toggles the graph widget."));
    graphViewAction->setShortcut(QKeySequence(tr("G")));
    graphViewAction->setCheckable(true);
    graphViewAction->setChecked(false);
    connect(graphViewAction, SIGNAL(triggered()), this, SLOT(toggleGraph()));

    viewMenu->addSeparator();

    QAction* showAxisAction = viewMenu->addAction(tr("&Axis"));
    showAxisAction->setToolTip(tr("Toggles the axis."));
    showAxisAction->setShortcut(QKeySequence(tr("A")));
    showAxisAction->setCheckable(true);
    showAxisAction->setChecked(graphicsViewWidget.showAxis);
    connect(showAxisAction, SIGNAL(triggered()), &graphicsViewWidget, SLOT(toggleAxis()));

    QAction* showGridAction = viewMenu->addAction(tr("&Grid"));
    showGridAction->setToolTip(tr("Toggles the grid."));
    //showGridAction->setShortcut(QKeySequence(tr("CTRL+G")));
    showGridAction->setCheckable(true);
    showGridAction->setChecked(graphicsViewWidget.showGrid);
    connect(showGridAction, SIGNAL(triggered()), &graphicsViewWidget, SLOT(toggleGrid()));

    QAction* showRulerAction = viewMenu->addAction(tr("&Ruler"));
    showRulerAction->setToolTip(tr("Toggles the ruler."));
    showRulerAction->setShortcut(QKeySequence(tr("R")));
    showRulerAction->setCheckable(true);
    showRulerAction->setChecked(graphicsViewWidget.showRuler);
    connect(showRulerAction, SIGNAL(triggered()), &graphicsViewWidget, SLOT(toggleRuler()));

    QAction* showFrameInfoAction = viewMenu->addAction(tr("&Frame Info"));
    showFrameInfoAction->setToolTip(tr("Toggles the frame info."));
    showFrameInfoAction->setCheckable(true);
    showFrameInfoAction->setChecked(graphicsViewWidget.showFrameInfo);
    connect(showFrameInfoAction, SIGNAL(triggered()), &graphicsViewWidget, SLOT(toggleFrameInfo()));

    QAction* simulationDebugAction = viewMenu->addAction(tr("&Simulation Debug"));
    simulationDebugAction->setToolTip(tr("Toggles the simulation debug drawing."));
    simulationDebugAction->setCheckable(true);
    simulationDebugAction->setChecked(command.showSimulationDebugDraw);
    connect(simulationDebugAction, SIGNAL(triggered()), this, SLOT(toggleSimulationDebug()));

    QAction* showLabelsAction = viewMenu->addAction(tr("&Labels"));
    showLabelsAction->setToolTip(tr("Toggles all labels."));
    //showLabelsAction->setShortcut(QKeySequence(tr("F")));
    showLabelsAction->setCheckable(true);
    showLabelsAction->setChecked(command.showLabels);
    connect(showLabelsAction, SIGNAL(triggered()), this, SLOT(toggleLabels()));

    viewMenu->addSeparator();

    QAction* showWorldObstaclesAction = viewMenu->addAction(tr("&World Polygons"));
    showWorldObstaclesAction->setToolTip(tr("Toggles the world polygons."));
    showWorldObstaclesAction->setShortcut(QKeySequence(tr("Shift+W")));
    showWorldObstaclesAction->setCheckable(true);
    showWorldObstaclesAction->setChecked(command.showWorldPolygons);
    connect(showWorldObstaclesAction, SIGNAL(triggered()), this, SLOT(toggleWorldPolygons()));

    QAction* showWorldMapAction = viewMenu->addAction(tr("&World Map"));
    showWorldMapAction->setToolTip(tr("Toggles the world map."));
    showWorldMapAction->setShortcut(QKeySequence(tr("W")));
    showWorldMapAction->setCheckable(true);
    showWorldMapAction->setChecked(command.showWorldMap);
    connect(showWorldMapAction, SIGNAL(triggered()), this, SLOT(toggleWorldMap()));

    QAction* showNavGoalsAction = viewMenu->addAction(tr("&Nav Goals"));
    showNavGoalsAction->setToolTip(tr("Toggles the nav goals."));
    showNavGoalsAction->setShortcut(QKeySequence(tr("N")));
    showNavGoalsAction->setCheckable(true);
    showNavGoalsAction->setChecked(command.showNavGoals);
    connect(showNavGoalsAction, SIGNAL(triggered()), this, SLOT(toggleNavGoals()));

    viewMenu->addSeparator();

    QAction* showBodyAction = viewMenu->addAction(tr("&Body"));
    showBodyAction->setToolTip(tr("Toggles the body."));
    showBodyAction->setShortcut(QKeySequence(tr("B")));
    showBodyAction->setCheckable(true);
    showBodyAction->setChecked(command.showBody);
    connect(showBodyAction, SIGNAL(triggered()), this, SLOT(toggleBody()));

    QAction* showTargetsAction = viewMenu->addAction(tr("&Targets"));
    showTargetsAction->setToolTip(tr("Toggles the targets."));
    showTargetsAction->setShortcut(QKeySequence(tr("T")));
    showTargetsAction->setCheckable(true);
    showTargetsAction->setChecked(command.showTargets);
    connect(showTargetsAction, SIGNAL(triggered()), this, SLOT(toggleTargets()));

    QAction* showLidarAction = viewMenu->addAction(tr("&Laser Sensor"));
    showLidarAction->setToolTip(tr("Toggles the simulated laser sensor."));
    showLidarAction->setShortcut(QKeySequence(tr("Shift+L")));
    showLidarAction->setCheckable(true);
    showLidarAction->setChecked(command.showLaser);
    connect(showLidarAction, SIGNAL(triggered()), this, SLOT(toggleLaser()));

    QAction* showRayModelAction = viewMenu->addAction(tr("&Ray Model"));
    showRayModelAction->setToolTip(tr("Toggles the ray model."));
    showRayModelAction->setShortcut(QKeySequence(tr("Y")));
    showRayModelAction->setCheckable(true);
    showRayModelAction->setChecked(command.showRayModel);
    connect(showRayModelAction, SIGNAL(triggered()), this, SLOT(toggleRayModel()));

    QAction* showVisibilityPolygonAction = viewMenu->addAction(tr("&Visibility Polygon"));
    showVisibilityPolygonAction->setShortcut(QKeySequence(tr("V")));
    showVisibilityPolygonAction->setCheckable(true);
    showVisibilityPolygonAction->setChecked(command.showVisibilityPolygon);
    connect(showVisibilityPolygonAction, SIGNAL(triggered()), this, SLOT(toggleVisibilityPolygon()));

    QAction* showSafetyZoneAction = viewMenu->addAction(tr("&Safety Zone"));
    //showSafetyZoneAction->setShortcut(QKeySequence(tr("S")));
    showSafetyZoneAction->setCheckable(true);
    showSafetyZoneAction->setChecked(command.showSafetyZone);
    connect(showSafetyZoneAction, SIGNAL(triggered()), this, SLOT(toggleSafetyZone()));

    QAction* showOccupancyGridAction = viewMenu->addAction(tr("&Costmap"));
    showOccupancyGridAction->setShortcut(QKeySequence(tr("S")));
    showOccupancyGridAction->setCheckable(true);
    showOccupancyGridAction->setChecked(command.showCostmap);
    connect(showOccupancyGridAction, SIGNAL(triggered()), this, SLOT(toggleCostmap()));

    QAction* showWorldModelAction = viewMenu->addAction(tr("&Local Map"));
    showWorldModelAction->setShortcut(QKeySequence(tr("P")));
    showWorldModelAction->setCheckable(true);
    showWorldModelAction->setChecked(command.showLocalMap);
    connect(showWorldModelAction, SIGNAL(triggered()), this, SLOT(toggleLocalMap()));

    QAction* showDijkstraMapAction = viewMenu->addAction(tr("&Dijkstra Map"));
    //showDijkstraMapAction->setShortcut(QKeySequence(tr("D")));
    showDijkstraMapAction->setCheckable(true);
    showDijkstraMapAction->setChecked(command.showDijkstraMap);
    connect(showDijkstraMapAction, SIGNAL(triggered()), this, SLOT(toggleDijkstraMap()));

    QAction* showWorldVisGraphAction = viewMenu->addAction(tr("&World Visibility Graph"));
    showWorldVisGraphAction->setToolTip(tr("Toggles the worls visiblity graph."));
    //showWorldVisGraphAction->setShortcut(QKeySequence(tr("V")));
    showWorldVisGraphAction->setCheckable(true);
    showWorldVisGraphAction->setChecked(command.showWorldVisibilityGraph);
    connect(showWorldVisGraphAction, SIGNAL(triggered()), this, SLOT(toggleWorldVisibilityGraph()));

    QAction* showLocalVisGraphAction = viewMenu->addAction(tr("&Local Visibility Graph"));
    showLocalVisGraphAction->setToolTip(tr("Toggles the local visiblity graph."));
    //showLocalVisGraphAction->setShortcut(QKeySequence(tr("V")));
    showLocalVisGraphAction->setCheckable(true);
    showLocalVisGraphAction->setChecked(command.showLocalVisibilityGraph);
    connect(showLocalVisGraphAction, SIGNAL(triggered()), this, SLOT(toggleLocalVisibilityGraph()));

    showWorldPathAction = viewMenu->addAction(tr("&Paths"));
    showWorldPathAction->setToolTip(tr("Toggles the paths."));
    //showWorldPathAction->setShortcut(QKeySequence(tr("W")));
    showWorldPathAction->setCheckable(true);
    showWorldPathAction->setChecked(command.showPaths);
    connect(showWorldPathAction, SIGNAL(triggered()), this, SLOT(togglePaths()));

    viewMenu->addSeparator();

    QAction* resampleColorsAction = viewMenu->addAction(tr("&Resample Colors"));
    resampleColorsAction->setToolTip(tr("Resamples the colors of the curves."));
    connect(resampleColorsAction, SIGNAL(triggered()), &graphWidget, SLOT(resampleColors()));


    QMenu* mapMenu = menuBar->addMenu(tr("&Map"));

    QActionGroup* mapActionGroup = new QActionGroup(mapMenu);

    QAction* voidAction = mapActionGroup->addAction(tr("&Void"));
    mapMenu->addAction(voidAction);
    voidAction->setShortcut(QKeySequence(tr("0")));
    voidAction->setCheckable(true);
    voidAction->setChecked(state.world.mapId == 0);
    connect(voidAction, SIGNAL(triggered()), &mapSelectionMapper, SLOT(map()));

    QAction* simpleAction = mapActionGroup->addAction(tr("&Simple"));
    mapMenu->addAction(simpleAction);
    simpleAction->setShortcut(QKeySequence(tr("1")));
    simpleAction->setCheckable(true);
    simpleAction->setChecked(state.world.mapId == 1);
    connect(simpleAction, SIGNAL(triggered()), &mapSelectionMapper, SLOT(map()));

    QAction* uAction = mapActionGroup->addAction(tr("&U Trap"));
    mapMenu->addAction(uAction);
    uAction->setShortcut(QKeySequence(tr("2")));
    uAction->setCheckable(true);
    uAction->setChecked(state.world.mapId == 2);
    connect(uAction, SIGNAL(triggered()), &mapSelectionMapper, SLOT(map()));

    QAction* tunnelAction = mapActionGroup->addAction(tr("&Tunnel"));
    mapMenu->addAction(tunnelAction);
    tunnelAction->setShortcut(QKeySequence(tr("3")));
    tunnelAction->setCheckable(true);
    tunnelAction->setChecked(state.world.mapId == 3);
    connect(tunnelAction, SIGNAL(triggered()), &mapSelectionMapper, SLOT(map()));

    QAction* apartmentAction = mapActionGroup->addAction(tr("&Apartment"));
    mapMenu->addAction(apartmentAction);
    apartmentAction->setShortcut(QKeySequence(tr("4")));
    apartmentAction->setCheckable(true);
    apartmentAction->setChecked(state.world.mapId == 4);
    connect(apartmentAction, SIGNAL(triggered()), &mapSelectionMapper, SLOT(map()));

    QAction* officeAction = mapActionGroup->addAction(tr("&Office"));
    mapMenu->addAction(officeAction);
    officeAction->setShortcut(QKeySequence(tr("5")));
    officeAction->setCheckable(true);
    officeAction->setChecked(state.world.mapId == 5);
    connect(officeAction, SIGNAL(triggered()), &mapSelectionMapper, SLOT(map()));

    QAction* wareHouseAction = mapActionGroup->addAction(tr("&Warehouse"));
    mapMenu->addAction(wareHouseAction);
    wareHouseAction->setShortcut(QKeySequence(tr("6")));
    wareHouseAction->setCheckable(true);
    wareHouseAction->setChecked(state.world.mapId == 6);
    connect(wareHouseAction, SIGNAL(triggered()), &mapSelectionMapper, SLOT(map()));

    QAction* clutterAction = mapActionGroup->addAction(tr("&Clutter"));
    mapMenu->addAction(clutterAction);
    clutterAction->setShortcut(QKeySequence(tr("7")));
    clutterAction->setCheckable(true);
    clutterAction->setChecked(state.world.mapId == 7);
    connect(clutterAction, SIGNAL(triggered()), &mapSelectionMapper, SLOT(map()));

    mapSelectionMapper.setMapping(voidAction, 0);
    mapSelectionMapper.setMapping(simpleAction, 1);
    mapSelectionMapper.setMapping(uAction, 2);
    mapSelectionMapper.setMapping(tunnelAction, 3);
    mapSelectionMapper.setMapping(apartmentAction, 4);
    mapSelectionMapper.setMapping(officeAction, 5);
    mapSelectionMapper.setMapping(wareHouseAction, 6);
    mapSelectionMapper.setMapping(clutterAction, 7);
    connect(&mapSelectionMapper, SIGNAL(mapped(int)), this, SLOT(toggleMap(int)));


    QMenu* controllerMenu = menuBar->addMenu(tr("&Controller"));

    QActionGroup* trajectoryPlannerActionGroup = new QActionGroup(controllerMenu);

    QAction* pdAction = trajectoryPlannerActionGroup->addAction(tr("&PD Control"));
    controllerMenu->addAction(pdAction);
    pdAction->setCheckable(true);
    pdAction->setChecked(command.trajectoryPlanningMethod == command.PD);
    connect(pdAction, SIGNAL(triggered()), &trajectoryPlannerSelectionMapper, SLOT(map()));

    QAction* bezierControlAction = trajectoryPlannerActionGroup->addAction(tr("&Bezier Control"));
    controllerMenu->addAction(bezierControlAction);
    bezierControlAction->setCheckable(true);
    bezierControlAction->setChecked(command.trajectoryPlanningMethod == command.Bezier);
    connect(bezierControlAction, SIGNAL(triggered()), &trajectoryPlannerSelectionMapper, SLOT(map()));

    QAction* reelControlAction = trajectoryPlannerActionGroup->addAction(tr("&Reel Control"));
    controllerMenu->addAction(reelControlAction);
    reelControlAction->setCheckable(true);
    reelControlAction->setChecked(command.trajectoryPlanningMethod == command.Reel);
    connect(reelControlAction, SIGNAL(triggered()), &trajectoryPlannerSelectionMapper, SLOT(map()));

    QAction* dwaAction = trajectoryPlannerActionGroup->addAction(tr("&DWA"));
    controllerMenu->addAction(dwaAction);
    dwaAction->setCheckable(true);
    dwaAction->setChecked(command.trajectoryPlanningMethod == command.DWA);
    connect(dwaAction, SIGNAL(triggered()), &trajectoryPlannerSelectionMapper, SLOT(map()));

    QAction* staaAction = trajectoryPlannerActionGroup->addAction(tr("&STAA*"));
    controllerMenu->addAction(staaAction);
    staaAction->setCheckable(true);
    staaAction->setChecked(command.trajectoryPlanningMethod == command.STAA);
    connect(staaAction, SIGNAL(triggered()), &trajectoryPlannerSelectionMapper, SLOT(map()));

    QAction* rbAction = trajectoryPlannerActionGroup->addAction(tr("&RuleBase"));
    controllerMenu->addAction(rbAction);
    rbAction->setCheckable(true);
    rbAction->setChecked(command.trajectoryPlanningMethod == command.RuleBase);
    connect(rbAction, SIGNAL(triggered()), &trajectoryPlannerSelectionMapper, SLOT(map()));

    QAction* scAction = trajectoryPlannerActionGroup->addAction(tr("&SpeedControl"));
    controllerMenu->addAction(scAction);
    scAction->setCheckable(true);
    scAction->setChecked(command.trajectoryPlanningMethod == command.SpeedControl);
    connect(scAction, SIGNAL(triggered()), &trajectoryPlannerSelectionMapper, SLOT(map()));

    trajectoryPlannerSelectionMapper.setMapping(pdAction, command.PD);
    trajectoryPlannerSelectionMapper.setMapping(bezierControlAction, command.Bezier);
    trajectoryPlannerSelectionMapper.setMapping(reelControlAction, command.Reel);
    trajectoryPlannerSelectionMapper.setMapping(dwaAction, command.DWA);
    trajectoryPlannerSelectionMapper.setMapping(scAction, command.RuleBase);
    trajectoryPlannerSelectionMapper.setMapping(staaAction, command.STAA);
    trajectoryPlannerSelectionMapper.setMapping(scAction, command.SpeedControl);
    connect(&trajectoryPlannerSelectionMapper, SIGNAL(mapped(int)), this, SLOT(toggleTrajectoryControl(int)));

    controllerMenu->addSeparator();

    QAction* emergencyBrakeAction = controllerMenu->addAction(tr("&Emergency Brake Reflex"));
    emergencyBrakeAction->setToolTip(tr("Toggles the emergency brake reflex."));
    //forceFieldReflexAction->setShortcut(QKeySequence(tr("F")));
    emergencyBrakeAction->setCheckable(true);
    emergencyBrakeAction->setChecked(command.emergencyBrakeReflex);
    connect(emergencyBrakeAction, SIGNAL(triggered()), this, SLOT(toggleEmergencyBrakeReflex()));

    QAction* safetyZoneReflexAction = controllerMenu->addAction(tr("&Safety Zone Reflex"));
    safetyZoneReflexAction->setToolTip(tr("Toggles the safety zone reflex."));
    //forceFieldReflexAction->setShortcut(QKeySequence(tr("F")));
    safetyZoneReflexAction->setCheckable(true);
    safetyZoneReflexAction->setChecked(command.safetyZoneReflex);
    connect(safetyZoneReflexAction, SIGNAL(triggered()), this, SLOT(toggleSafetyZoneReflex()));

    QAction* stucknessReflexAction = controllerMenu->addAction(tr("&Stuckness Reflex"));
    stucknessReflexAction->setToolTip(tr("Toggles the stuckness reflex."));
    //stucknessReflexAction->setShortcut(QKeySequence(tr("F")));
    stucknessReflexAction->setCheckable(true);
    stucknessReflexAction->setChecked(command.stucknessReflex);
    connect(stucknessReflexAction, SIGNAL(triggered()), this, SLOT(toggleStucknessReflex()));


    QMenu* commandMenu = menuBar->addMenu(tr("&Command"));

    QAction* selectPoseAction = commandMenu->addAction(tr("&Select Pose"));
    selectPoseAction->setToolTip(tr("Select initial pose for localization."));
    selectPoseAction->setShortcut(QKeySequence(tr("Ctrl+P")));
    selectPoseAction->setCheckable(true);
    selectPoseAction->setChecked(command.selectPose);
    connect(selectPoseAction, SIGNAL(triggered()), this, SLOT(toggleSelectPose()));

    QAction* selectTargetAction = commandMenu->addAction(tr("&Select Target"));
    selectTargetAction->setToolTip(tr("Select target pose for navigation."));
    selectTargetAction->setShortcut(QKeySequence(tr("Ctrl+T")));
    selectTargetAction->setCheckable(true);
    selectTargetAction->setChecked(command.selectTarget);
    connect(selectTargetAction, SIGNAL(triggered()), this, SLOT(toggleSelectTarget()));

    commandMenu->addSeparator();

    QAction* experimenterAction = commandMenu->addAction(tr("&Experimenter"));
    experimenterAction->setToolTip(tr("Toggles experimenter."));
    experimenterAction->setShortcut(QKeySequence(tr("E")));
    experimenterAction->setCheckable(true);
    experimenterAction->setChecked(experimenter.running);
    connect(experimenterAction, SIGNAL(triggered()), &experimenter, SLOT(startstop()));

    commandMenu->addSeparator();

    QAction* teachingAction = commandMenu->addAction(tr("&Teaching"));
    teachingAction->setToolTip(tr("Toggles the teaching mode."));
    //teachingAction->setShortcut(QKeySequence(tr("Ctrl+T")));
    teachingAction->setCheckable(true);
    teachingAction->setChecked(command.learn);
    connect(teachingAction, SIGNAL(triggered()), this, SLOT(toggleTeaching()));

    QAction* clearRuleBaseAction = commandMenu->addAction(tr("&Clear RuleBase"));
    clearRuleBaseAction->setToolTip(tr("Clears the rule base."));
    //clearRuleBaseAction->setShortcut(QKeySequence(tr("L")));
    clearRuleBaseAction->setCheckable(false);
    clearRuleBaseAction->setChecked(false);
    connect(clearRuleBaseAction, SIGNAL(triggered()), this, SLOT(clearRuleBase()));

    commandMenu->addSeparator();

    QAction* timeAbortAction = commandMenu->addAction(tr("&Use Time Abort"));
    timeAbortAction->setToolTip(tr("Toggles the time abort mode."));
    //timeAbortAction->setShortcut(QKeySequence(tr("U")));
    timeAbortAction->setCheckable(true);
    timeAbortAction->setChecked(command.useTimeAbort);
    connect(timeAbortAction, SIGNAL(triggered()), this, SLOT(toggleTimeAbort()));

    QAction* useClosingAction = commandMenu->addAction(tr("&Use Closing"));
    useClosingAction->setToolTip(tr("Toggles the closing feature."));
    //timeAbortAction->setShortcut(QKeySequence(tr("U")));
    useClosingAction->setCheckable(true);
    useClosingAction->setChecked(command.useClosing);
    connect(useClosingAction, SIGNAL(triggered()), this, SLOT(toggleClosing()));

    QAction* dynamicPathAction = commandMenu->addAction(tr("&Use Dynamic Path"));
    dynamicPathAction->setToolTip(tr("Toggles the predicted paths."));
    //dynamicPathAction->setShortcut(QKeySequence(tr("U")));
    dynamicPathAction->setCheckable(true);
    dynamicPathAction->setChecked(command.useDynamicPath);
    connect(dynamicPathAction, SIGNAL(triggered()), this, SLOT(toggleDynamicPath()));

    commandMenu->addSeparator();

    QActionGroup* predictionActionGroup = new QActionGroup(commandMenu);

    QAction* unicyclePredictionAction = predictionActionGroup->addAction(tr("&Unicycle prediction"));
    commandMenu->addAction(unicyclePredictionAction);
    unicyclePredictionAction->setCheckable(true);
    unicyclePredictionAction->setChecked(command.predictionType == command.Unicycle);
    connect(unicyclePredictionAction, SIGNAL(triggered()), &predictionTypeSelectionMapper, SLOT(map()));

    QAction* holonomicPredictionAction = predictionActionGroup->addAction(tr("&Holonomic prediction"));
    commandMenu->addAction(holonomicPredictionAction);
    holonomicPredictionAction->setCheckable(true);
    holonomicPredictionAction->setChecked(command.predictionType == command.Holonomic);
    connect(holonomicPredictionAction, SIGNAL(triggered()), &predictionTypeSelectionMapper, SLOT(map()));

    QAction* noPredictionAction = predictionActionGroup->addAction(tr("&No prediction"));
    commandMenu->addAction(noPredictionAction);
    noPredictionAction->setCheckable(true);
    noPredictionAction->setChecked(command.predictionType == command.None);
    connect(noPredictionAction, SIGNAL(triggered()), &predictionTypeSelectionMapper, SLOT(map()));

    predictionTypeSelectionMapper.setMapping(unicyclePredictionAction, command.Unicycle);
    predictionTypeSelectionMapper.setMapping(holonomicPredictionAction, command.Holonomic);
    predictionTypeSelectionMapper.setMapping(noPredictionAction, command.None);
    connect(&predictionTypeSelectionMapper, SIGNAL(mapped(int)), this, SLOT(togglePredictionType(int)));

    commandMenu->addSeparator();

    QActionGroup* heuristicActionGroup = new QActionGroup(commandMenu);

    QAction* euklidieanAction = heuristicActionGroup->addAction(tr("&Euklidean Heuristic"));
    commandMenu->addAction(euklidieanAction);
    euklidieanAction->setCheckable(true);
    euklidieanAction->setChecked(command.heuristic == command.Euklidean);
    connect(euklidieanAction, SIGNAL(triggered()), &heuristicSelectionMapper, SLOT(map()));

    QAction* pathEuklidieanAction = heuristicActionGroup->addAction(tr("&PathEuklidean Heuristic"));
    commandMenu->addAction(pathEuklidieanAction);
    pathEuklidieanAction->setCheckable(true);
    pathEuklidieanAction->setChecked(command.heuristic == command.PathEuklidean);
    connect(pathEuklidieanAction, SIGNAL(triggered()), &heuristicSelectionMapper, SLOT(map()));

    QAction* rtrAction = heuristicActionGroup->addAction(tr("&RTR Heuristic"));
    commandMenu->addAction(rtrAction);
    rtrAction->setCheckable(true);
    rtrAction->setChecked(command.heuristic == command.RTR);
    connect(rtrAction, SIGNAL(triggered()), &heuristicSelectionMapper, SLOT(map()));

    QAction* dockRtrAction = heuristicActionGroup->addAction(tr("&Dock RTR Heuristic"));
    commandMenu->addAction(dockRtrAction);
    dockRtrAction->setCheckable(true);
    dockRtrAction->setChecked(command.heuristic == command.DOCK_RTR);
    connect(dockRtrAction, SIGNAL(triggered()), &heuristicSelectionMapper, SLOT(map()));

    QAction* rtrMinAction = heuristicActionGroup->addAction(tr("&RTR Min Heuristic"));
    commandMenu->addAction(rtrMinAction);
    rtrMinAction->setCheckable(true);
    rtrMinAction->setChecked(command.heuristic == command.RTR_MIN);
    connect(rtrMinAction, SIGNAL(triggered()), &heuristicSelectionMapper, SLOT(map()));

    QAction* rtrMaxAction = heuristicActionGroup->addAction(tr("&RTR Max Heuristic"));
    commandMenu->addAction(rtrMaxAction);
    rtrMaxAction->setCheckable(true);
    rtrMaxAction->setChecked(command.heuristic == command.RTR_MAX);
    connect(rtrMaxAction, SIGNAL(triggered()), &heuristicSelectionMapper, SLOT(map()));

    QAction* mcAction = heuristicActionGroup->addAction(tr("&PathRTR MC Heuristic"));
    commandMenu->addAction(mcAction);
    mcAction->setCheckable(true);
    mcAction->setChecked(command.heuristic == command.MinimalConstruct);
    connect(mcAction, SIGNAL(triggered()), &heuristicSelectionMapper, SLOT(map()));

    QAction* gridDijkstraAction = heuristicActionGroup->addAction(tr("&PathRTR Dijkstra Heuristic"));
    commandMenu->addAction(gridDijkstraAction);
    gridDijkstraAction->setCheckable(true);
    gridDijkstraAction->setChecked(command.heuristic == command.GridDijkstra);
    connect(gridDijkstraAction, SIGNAL(triggered()), &heuristicSelectionMapper, SLOT(map()));

    heuristicSelectionMapper.setMapping(euklidieanAction, command.Euklidean);
    heuristicSelectionMapper.setMapping(pathEuklidieanAction, command.PathEuklidean);
    heuristicSelectionMapper.setMapping(rtrAction, command.RTR);
    heuristicSelectionMapper.setMapping(dockRtrAction, command.DOCK_RTR);
    heuristicSelectionMapper.setMapping(rtrMinAction, command.RTR_MIN);
    heuristicSelectionMapper.setMapping(rtrMaxAction, command.RTR_MAX);
    heuristicSelectionMapper.setMapping(gridDijkstraAction, command.GridDijkstra);
    connect(&heuristicSelectionMapper, SIGNAL(mapped(int)), this, SLOT(toggleHeuristic(int)));

    commandMenu->addSeparator();

    QActionGroup* trajectoryTypeActionGroup = new QActionGroup(commandMenu);

    QAction* arcAction = trajectoryTypeActionGroup->addAction(tr("&Arc Trajectory"));
    commandMenu->addAction(arcAction);
    arcAction->setCheckable(true);
    arcAction->setChecked(command.trajectoryType == command.Arc);
    connect(arcAction, SIGNAL(triggered()), &trajectoryTypeSelectionMapper, SLOT(map()));

    QAction* b0Action = trajectoryTypeActionGroup->addAction(tr("&B0 Trajectory"));
    commandMenu->addAction(b0Action);
    b0Action->setCheckable(true);
    b0Action->setChecked(command.trajectoryType == command.B0);
    connect(b0Action, SIGNAL(triggered()), &trajectoryTypeSelectionMapper, SLOT(map()));

    QAction* fresnelAction = trajectoryTypeActionGroup->addAction(tr("&Fresnel Trajectory"));
    commandMenu->addAction(fresnelAction);
    fresnelAction->setCheckable(true);
    fresnelAction->setChecked(command.trajectoryType == command.Fresnel);
    connect(fresnelAction, SIGNAL(triggered()), &trajectoryTypeSelectionMapper, SLOT(map()));

    trajectoryTypeSelectionMapper.setMapping(arcAction, command.Arc);
    trajectoryTypeSelectionMapper.setMapping(b0Action, command.B0);
    trajectoryTypeSelectionMapper.setMapping(fresnelAction, command.Fresnel);
    connect(&trajectoryTypeSelectionMapper, SIGNAL(mapped(int)), this, SLOT(toggleTrajectoryType(int)));

    commandMenu->addSeparator();

    QActionGroup* frequencyActionGroup = new QActionGroup(commandMenu);

    QAction* hz5Action = frequencyActionGroup->addAction(tr("&5Hz"));
    commandMenu->addAction(hz5Action);
    hz5Action->setCheckable(true);
    hz5Action->setChecked(command.frequency == 5);
    connect(hz5Action, SIGNAL(triggered()), &frequencySelectionMapper, SLOT(map()));

    QAction* hz10Action = frequencyActionGroup->addAction(tr("&10Hz"));
    commandMenu->addAction(hz10Action);
    hz10Action->setCheckable(true);
    hz10Action->setChecked(command.frequency == 10);
    connect(hz10Action, SIGNAL(triggered()), &frequencySelectionMapper, SLOT(map()));

    QAction* hz20Action = frequencyActionGroup->addAction(tr("&20Hz"));
    commandMenu->addAction(hz20Action);
    hz20Action->setCheckable(true);
    hz20Action->setChecked(command.frequency == 20);
    connect(hz20Action, SIGNAL(triggered()), &frequencySelectionMapper, SLOT(map()));

    QAction* hz30Action = frequencyActionGroup->addAction(tr("&30Hz"));
    commandMenu->addAction(hz30Action);
    hz30Action->setCheckable(true);
    hz30Action->setChecked(command.frequency == 30);
    connect(hz30Action, SIGNAL(triggered()), &frequencySelectionMapper, SLOT(map()));

    frequencySelectionMapper.setMapping(hz5Action, 5);
    frequencySelectionMapper.setMapping(hz10Action, 10);
    frequencySelectionMapper.setMapping(hz20Action, 20);
    frequencySelectionMapper.setMapping(hz30Action, 30);
    connect(&frequencySelectionMapper, SIGNAL(mapped(int)), this, SLOT(toggleFrequency(int)));

    commandMenu->addSeparator();

    QAction* ghostModeAction = commandMenu->addAction(tr("&Ghost Mode"));
    ghostModeAction->setToolTip(tr("Toggles the dumb agents flag."));
    //forceFieldReflexAction->setShortcut(QKeySequence(tr("F")));
    ghostModeAction->setCheckable(true);
    ghostModeAction->setChecked(command.ghostMode);
    connect(ghostModeAction, SIGNAL(triggered()), this, SLOT(toggleGhostMode()));


    menuBar->addSeparator();
    QAction* fakeSeparator1 = menuBar->addAction("     ");
    fakeSeparator1->setEnabled(false);

    joystickAction = menuBar->addAction(tr("&Joystick"));
    joystickAction->setToolTip(tr("Toggles the joystick."));
    joystickAction->setShortcut(QKeySequence(tr("J")));
    joystickAction->setCheckable(true);
    connect(joystickAction, SIGNAL(triggered()), this, SLOT(toggleJoystick()));

    QAction* keyboardAction = menuBar->addAction(tr("&Keyboard"));
    keyboardAction->setToolTip(tr("Toggles the keyboard."));
    keyboardAction->setShortcut(QKeySequence(tr("K")));
    keyboardAction->setCheckable(true);
    connect(keyboardAction, SIGNAL(triggered()), this, SLOT(toggleKeyboard()));

    QAction* recordAction = menuBar->addAction(tr("Record"));
    recordAction->setCheckable(true);
    recordAction->setToolTip(tr("Toggles recording."));
    recordAction->setShortcut(QKeySequence(tr("Return")));
    connect(recordAction, SIGNAL(triggered()), this, SLOT(record()));

    QAction* resetAction = menuBar->addAction(tr("&Reset"));
    resetAction->setToolTip(tr("Resets the simulation state."));
    //resetAction->setShortcut(QKeySequence(tr("R")));
    connect(resetAction, SIGNAL(triggered()), this, SLOT(reset()));

    menuBar->addSeparator();
    QAction* fakeSeparator2 = menuBar->addAction("     ");
    fakeSeparator2->setEnabled(false);

    QAction* jumpToStartAction = menuBar->addAction(tr("|<"));
    jumpToStartAction->setToolTip(tr("Sets the player to the first frame."));
    jumpToStartAction->setShortcut(QKeySequence(tr("Backspace")));
    connect(jumpToStartAction, SIGNAL(triggered()), this, SLOT(jumpToStart()));

    QAction* frameBackAction = menuBar->addAction(tr("<"));
    frameBackAction->setToolTip(tr("Rewinds the player by one frame."));
    //frameBackAction->setShortcut(QKeySequence(tr("Backspace")));
    connect(frameBackAction, SIGNAL(triggered()), this, SLOT(frameBack()));

    QAction* playAction = menuBar->addAction(tr("Play"));
    playAction->setToolTip(tr("Starts the playback."));
    playAction->setShortcut(QKeySequence(tr("Space")));
    connect(playAction, SIGNAL(triggered()), this, SLOT(play()));

    //QAction* stopAction = menuBar->addAction(tr("Stop"));
    //stopAction->setToolTip(tr("Stops the playback."));
    //stopAction->setShortcut(QKeySequence(tr("Space")));
    //connect(stopAction, SIGNAL(triggered()), this, SLOT(stop()));

    QAction* frameForwardAction = menuBar->addAction(tr(">"));
    frameForwardAction->setToolTip(tr("Advances the player by one frame."));
    //frameForwardAction->setShortcut(QKeySequence(tr("Backspace")));
    connect(frameForwardAction, SIGNAL(triggered()), this, SLOT(frameForward()));

    QAction* jumpToEndAction = menuBar->addAction(tr(">|"));
    jumpToEndAction->setToolTip(tr("Sets the player to the last frame."));
}

// Vertical splitter synchronization.
void MainWindow::topSplitterMoved()
{
    verticalSplitterBottom->setSizes(verticalSplitterTop->sizes());
}

void MainWindow::bottomSplitterMoved()
{
    verticalSplitterTop->setSizes(verticalSplitterBottom->sizes());
}

void MainWindow::joystickConnected()
{
    messageIn("Joystick connected.");
    joystickAction->setVisible(true);
    joystickAction->setChecked(true);
    command.joystick = true;
}

void MainWindow::joystickDisconnected()
{
    messageIn("Joystick disconnected.");
    joystickAction->setVisible(false);
    command.joystick = false;
}

// Handles the joystick buttons. Some of them select and move sliders in the config widget.
// Some control the robot by switching between walk and halt, executing kicks etc.
void MainWindow::joystickButtonPressed(QList<bool> button)
{
    // Buttons 1 to 4 control robot behaviors.
    if (button[1])
    {
        toggleTeaching();
        //command.joystick = false;
    }

    if (button[2])
    {
        Rule rule = state.world.unicycleAgents[0].ruleBase.getCurrentRule();
        messageIn("Rule " + QString::number(rule.id) + " deleted.");
        state.world.unicycleAgents[0].ruleBase.deleteCurrentRule();
    }

    if (button[0] || button[3])
    {
        toggleJoystick();
    }

    // The other buttons control the config widget and they should only do it if the config widget is visible.
    if (configWidget.isHidden())
        return;
/*
    // Save config.
    else if (button[4] && button[6])
    {
        config.save(robotName);
        messageIn("Config saved.");
        return;
    }

    // Reset config.
    else if (button[5] && button[7])
    {
        config.load(robotName);
        configChanged();
        messageIn("Config reset.");
        return;
    }
*/
    // Switch to the next slider.
    else if (button[6])
        configWidget.selectNextSlider();

    // Switch to the previous slider.
    else if (button[4])
        configWidget.selectPrevSlider();

    // Increase slider value.
    else if (button[5])
        configWidget.increaseSelectedSlider();

    // Decrease slider value.
    else if (button[7])
        configWidget.decreaseSelectedSlider();
}

// Handles the joystick sticks.
void MainWindow::joystickMoved(QList<double> axis)
{
    command.ax = axis[1];
    command.ay = axis[0];
    //qDebug() << axis;
}

// This is needed when the entire config has changed, e.g. when the robot model was
// changed, the config was reset or a new robot was detected.
void MainWindow::configChanged()
{
    configWidget.configChangedIn();
    graphicsViewWidget.update();
}

// Slot for internal message strings.
void MainWindow::messageIn(QString m)
{
    graphicsViewWidget.messageIn(m);
}

// Toggles the config widget.
void MainWindow::toggleConfig()
{
    if (configWidget.isHidden())
        configWidget.show();
    else
        configWidget.hide();
}

// Toggles the graph widget.
void MainWindow::toggleGraph()
{
    if (verticalSplitterBottom->isHidden())
        verticalSplitterBottom->show();
    else
        verticalSplitterBottom->hide();
}

void MainWindow::toggleJoystick()
{
    command.joystick = !command.joystick;
    if (command.joystick)
        messageIn("Joystick control activated.");
    else
        messageIn("Joystick control deactivated.");
}

void MainWindow::toggleKeyboard()
{
    command.keyboard = !command.keyboard;
    if (command.keyboard)
        messageIn("Keyboard control enabled.");
    else
        messageIn("Keyboard control disabled.");
}

void MainWindow::toggleSafetyZoneReflex()
{
    command.safetyZoneReflex = !command.safetyZoneReflex;
    if (command.safetyZoneReflex)
        messageIn("Safety zone reflex enabled.");
    else
        messageIn("Safety zone reflex disabled.");
    update();
}

void MainWindow::toggleNavGoals()
{
    command.showNavGoals = !command.showNavGoals;
    graphicsScene.init();
    update();
}

void MainWindow::toggleTargets()
{
    command.showTargets = !command.showTargets;
    update();
}

void MainWindow::toggleLabels()
{
    command.showLabels = !command.showLabels;
    graphicsViewWidget.update();
}

void MainWindow::toggleWorldPolygons()
{
    command.showWorldPolygons = !command.showWorldPolygons;
    graphicsScene.init();
    update();
}

void MainWindow::toggleWorldMap()
{
    command.showWorldMap = !command.showWorldMap;
    update();
}

void MainWindow::togglePaths()
{
    command.showPaths = !command.showPaths;
    update();
}

void MainWindow::toggleLocalMap()
{
    command.showLocalMap = !command.showLocalMap;
    update();
}

void MainWindow::toggleVisibilityPolygon()
{
    command.showVisibilityPolygon = !command.showVisibilityPolygon;
    update();
}

void MainWindow::toggleSafetyZone()
{
    command.showSafetyZone = !command.showSafetyZone;
    update();
}

void MainWindow::toggleSimulationDebug()
{
    command.showSimulationDebugDraw = !command.showSimulationDebugDraw;
    update();
}

void MainWindow::toggleRayModel()
{
    command.showRayModel = !command.showRayModel;
    update();
}

void MainWindow::toggleLaser()
{
    command.showLaser = (command.showLaser+1) % 4;
    update();
}

void MainWindow::toggleWorldVisibilityGraph()
{
    command.showWorldVisibilityGraph = !command.showWorldVisibilityGraph;
    update();
}

void MainWindow::toggleLocalVisibilityGraph()
{
    command.showLocalVisibilityGraph = !command.showLocalVisibilityGraph;
    update();
}

void MainWindow::toggleTeaching()
{
    command.learn = !command.learn;
    if(command.learn)
        messageIn("Teaching enabled!");
    else
        messageIn("Teaching off.");
}

void MainWindow::clearRuleBase()
{
    state.world.unicycleAgents[0].ruleBase.clear();
    messageIn("Rule base cleared.");
}

void MainWindow::toggleCostmap()
{
    command.showCostmap = !command.showCostmap;
    update();
}

void MainWindow::toggleDijkstraMap()
{
    command.showDijkstraMap = !command.showDijkstraMap;
    update();
}

void MainWindow::toggleEmergencyBrakeReflex()
{
    command.emergencyBrakeReflex = !command.emergencyBrakeReflex;
    if (command.emergencyBrakeReflex)
        messageIn("Emergency brake reflex enabled.");
    else
        messageIn("Emergency brake disabled.");
    update();
}

void MainWindow::toggleStucknessReflex()
{
    command.stucknessReflex = !command.stucknessReflex;
    if (command.stucknessReflex)
        messageIn("Stuckness reflex enabled.");
    else
        messageIn("Stuckness reflex disabled.");
    update();
}

void MainWindow::toggleTimeAbort()
{
    command.useTimeAbort = !command.useTimeAbort;
    if (command.useTimeAbort)
        messageIn("Time abort enabled.");
    else
        messageIn("Time abort disabled.");
    update();
}

void MainWindow::toggleClosing()
{
    command.useClosing = !command.useClosing;
    if (command.useClosing)
        messageIn("Closing enabled.");
    else
        messageIn("Closing disabled.");
    update();
}

void MainWindow::toggleDynamicPath()
{
    command.useDynamicPath = !command.useDynamicPath;
    if (command.useDynamicPath)
        messageIn("Dynamic path enabled.");
    else
        messageIn("Dynamic path disabled.");
    update();
}

void MainWindow::toggleBody()
{
    command.showBody = !command.showBody;
    graphicsViewWidget.update();
}

void MainWindow::toggleSelectPose()
{
    command.selectPose = !command.selectPose;
    if (command.selectPose)
        messageIn("Pose selection enabled.");
    else
        messageIn("Pose selection disabled.");
    graphicsViewWidget.togglePoseSelection();
}

void MainWindow::toggleSelectTarget()
{
    command.selectTarget = !command.selectTarget;
    if (command.selectTarget)
        messageIn("Target selection enabled.");
    else
        messageIn("Target selection disabled.");
    graphicsViewWidget.togglePoseSelection();
}

void MainWindow::poseRecorded(const Pose2D &pose)
{
    if (command.selectPose)
    {
        //messageIn("Initial pose set.");
    }
    else if (command.selectTarget)
    {
        state.world.setMainTarget(pose);
        messageIn("Navigation target set.");
    }
    command.selectPose = false;
    command.selectTarget = false;
}

void MainWindow::toggleGhostMode()
{
    command.ghostMode = !command.ghostMode;
    if (command.ghostMode)
        messageIn("Ghost mode enabled.");
    else
        messageIn("Ghost mode disabled.");
}

void MainWindow::toggleTrajectoryControl(int d)
{
    command.trajectoryPlanningMethod = d;
    state.world.setParams(command.trajectoryPlanningMethod, command.trajectoryType, command.predictionType, command.heuristic, command.frequency);

    if (d == command.DWA)
        messageIn("DWA");
    else if (d == command.Arc)
        messageIn("Arc Control");
    else if (d == command.Bezier)
        messageIn("Bezier Control");
    else if (d == command.Reel)
        messageIn("Reel Control");
    else if (d == command.STAA)
        messageIn("ST Aborting A*");
    else if (d == command.PD)
        messageIn("PD Control");
    else if (d == command.RuleBase)
        messageIn("RuleBase");
    else if (d == command.SpeedControl)
        messageIn("Speed Control");
    else
        messageIn("Invalid Trajectory Planner " + d);

    update();
}

void MainWindow::togglePredictionType(int d)
{
    command.predictionType = d;
    state.world.setParams(command.trajectoryPlanningMethod, command.trajectoryType, command.predictionType, command.heuristic, command.frequency);

    if (d == command.Unicycle)
        messageIn("Unicycle prediction");
    else if (d == command.Holonomic)
        messageIn("Holonomic prediction");
    else if (d == command.None)
        messageIn("No prediction");
}

void MainWindow::toggleHeuristic(int d)
{
    command.heuristic = d;
    state.world.setParams(command.trajectoryPlanningMethod, command.trajectoryType, command.predictionType, command.heuristic, command.frequency);

    if (d == command.Euklidean)
        messageIn("Euklidean heuristic");
    else if (d == command.PathEuklidean)
        messageIn("PathEuklidean heuristic");
    else if (d == command.RTR)
        messageIn("RTR heuristic");
    else if (d == command.DOCK_RTR)
        messageIn("Dock RTR heuristic");
    else if (d == command.RTR_MIN)
        messageIn("RTR Min heuristic");
    else if (d == command.RTR_MAX)
        messageIn("RTR Max heuristic");
    else if (d == command.MinimalConstruct)
        messageIn("Minimal Construct heuristic");
    else if (d == command.GridDijkstra)
        messageIn("Grid Dijkstra heuristic");
}

void MainWindow::toggleTrajectoryType(int d)
{
    command.trajectoryType = d;
    state.world.setParams(command.trajectoryPlanningMethod, command.trajectoryType, command.predictionType, command.heuristic, command.frequency);

    if (d == command.Arc)
        messageIn("Switched to Arc trajectory type.");
    else if (d == command.B0)
        messageIn("Switched to B0 trajectory type.");
    else if (d == command.Fresnel)
        messageIn("Switched to Fresnel trajectory type.");
}

void MainWindow::toggleFrequency(int f)
{
    command.frequency = f;
    state.clear();
    state.world.setParams(command.trajectoryPlanningMethod, command.trajectoryType, command.predictionType, command.heuristic, command.frequency);
    if (mainControlLoop.isRunning())
    {
        mainControlLoop.stop();
        mainControlLoop.start();
    }

    if (f == 5)
        messageIn("Switched to 5Hz.");
    else if (f == 10)
        messageIn("Switched to 10Hz.");
    else if (f == 20)
        messageIn("Switched to 20Hz.");
    else
        messageIn("Switched to 30Hz.");
}

void MainWindow::toggleMap(int mapId)
{
    state.clear();
    state.world.setMap(mapId, max(config.unicycleAgents, 1.0));
    graphicsScene.init();
    graphicsViewWidget.init();

    if (mapId == 0)
    {
        messageIn("Switched to Void map.");
    }
    if (mapId == 1)
    {
        messageIn("Switched to Simple map.");
    }
    if (mapId == 2)
    {
        messageIn("Switched to U Trap map.");
    }
    if (mapId == 3)
    {
        messageIn("Switched to Tunnel map.");
    }
    if (mapId == 4)
    {
        messageIn("Switched to Apartment map.");
    }
    if (mapId == 5)
    {
        messageIn("Switched to Office map.");
    }
    if (mapId == 6)
    {
        messageIn("Switched to Warehouse map.");
    }
    if (mapId == 7)
    {
        messageIn("Switched to Clutter map.");
    }
}

// This function is called by the animation timer to update the gui
// when replaying the state history.
void MainWindow::animate()
{
    if (recording)
    {
        animationTimer.stop();
        return;
    }

    if (cfi+fim < 0 || cfi+fim > state.size()-1 || state.stop)
    {
        state.stop = false;
        fim = 0;
        animationTimer.stop();
    }
    else
    {
        loadFrame(cfi+fim);
    }
}

// Toggles the record mode. In record mode, the robot control thread is
// running and data are collected in the state history.
void MainWindow::record()
{
    if (recording)
        recordStop();
    else
        recordStart();
}

// Toggles the record mode. In record mode the robot control thread is running and data are collected in the state history.
void MainWindow::recordStart()
{
    animationTimer.stop();
    recording = true;
    graphicsViewWidget.startRecording();
    guiUpdateTimer.start();
    mainControlLoop.start();
}

// Toggles the record mode. In record mode the robot control thread is running and data are collected in the state history.
void MainWindow::recordStop()
{
    recording = false;
    graphicsViewWidget.stopRecording();
    mainControlLoop.stop();
    guiUpdateTimer.stop();
    cfi = max(0,state.size()-1);
}

// Play button handler.
void MainWindow::play()
{
    if (recording)
        return;

    if (fim != 0)
    {
        fim = 0;
        animationTimer.stop();
    }
    else
    {
        fim = 1;
        animationTimer.start();
    }
}

// Stop button handler.
void MainWindow::stop()
{
    animationTimer.stop();
    update();
}

void MainWindow::frameForward()
{
    if (recording)
        return;

    animationTimer.stop();
    if (cfi >= state.size()-1)
    {
        // At the end of the state history, step the controller forward and produce a new frame.
        cfi++;
        mainControlLoop.step();
        update();
    }
    else
    {
        loadFrame(cfi+1);
    }
}

void MainWindow::frameBack()
{
    if (recording)
        return;

    animationTimer.stop();
    loadFrame(cfi-1);
}

void MainWindow::jumpToStart()
{
    if (recording)
        return;

    animationTimer.stop();
    loadFrame(0);
}

void MainWindow::jumpToEnd()
{
    if (recording)
        return;

    animationTimer.stop();
    loadFrame(state.size()-1);
}

void MainWindow::jumpToFrame(int f)
{
    if (recording)
        return;

    animationTimer.stop();
    cfi = qMax(0, int((double)((state.size()-1) * f)/1000.0));
    loadFrame(cfi);
}

// This slot is called when the user browses the state history with the slider or with
// the frame forward, frame backward, and playback functions.
void MainWindow::loadFrame(int cfi)
{
    if (recording)
        return;

    if (cfi < 0 || cfi >= state.size())
        return;

    this->cfi = cfi;

    // The browsing of the state history is implemented in a generic way such that
    // the current state is overwritten with an older version of the state from
    // state history.
    state = state[cfi];

    // However, a couple of things require specific attention.

    if (state.world.unicycleAgents.size() > 0)
        state.world.unicycleAgents[0] = state.uniTaxi;

    // 1. The motion controllers of the agents (holo taxi, car taxi) are static members
    // and such they are not reset to an older state when the taxi is overwritten by
    // a copy. The sense() act() loop needs to be executed explicitly so that the
    // controllers can recompute their output using the copied state as input.
    state.world.stepAgents();

    // 2. The physical simulation is also static. The loaded state has to be recreated
    // in the simulation so that the simulation can continue seamlessly.
    state.world.physicsTransformOut();

    updateGui();
    emit progressOut(qMax(0, (int)(1000.0 * double(cfi)/(state.size()-1))));
}

// Refreshes the gui components.
void MainWindow::updateGui()
{
    graphicsScene.init();
    graphicsViewWidget.update();
    graphWidget.update();
}

void MainWindow::saveConfig()
{
    config.save(robotName);
    messageIn("Config saved.");
}

void MainWindow::loadConfig()
{
    config.load(robotName);
    configChanged();
    messageIn("Config reset.");
}

void MainWindow::saveStateHistory()
{
    recordStop();
    state.save();
    messageIn("State history saved.");
}

void MainWindow::loadStateHistory(QString fileName)
{
    messageIn("Loading...");
    recordStop();
    state.load(fileName);
    jumpToStart();
    messageIn("State history loaded.");
}

void MainWindow::exportWorld()
{
    state.world.logObstaclesToTxt();
    messageIn("World exported.");
}

void MainWindow::reset()
{
    state.world.reset();
    //mainControlLoop.reset();
    //graphicsScene.reset();
    //graphicsViewWidget.reset();
    messageIn("Reset");
}

// ---- KEYBOARD HANDLING ---- //

// Global event filter.
// I'm using this to route all key presses into the keyPressEvent of this class
// no matter where the focus is. This works much better than calling grabKeyBoard().
bool MainWindow::eventFilter(QObject *obj, QEvent *event)
{
    if (event->type() == QEvent::KeyPress)
    {
        QKeyEvent *keyEvent = static_cast<QKeyEvent*>(event);
        keyPressEvent(keyEvent);
        return true;
    }
    else
    {
        // standard event processing
        return QObject::eventFilter(obj, event);
    }
}

void MainWindow::keyPressEvent(QKeyEvent *event)
{
    if (event->isAutoRepeat())
        return;

    // CTRL-G toggles the curves (graphs) and config widgets.
    if (event->key() == Qt::Key_G && event->modifiers() & Qt::ControlModifier)
    {
        toggleConfig();
    }

    else if (event->key() == Qt::Key_Backspace)
    {
        jumpToStart();
    }
    else if (event->key() == Qt::Key_Escape)
    {
        close();
    }
    //    else if (event->key() == Qt::Key_D)
    //    {
    //        toggleDwa(-1);
    //    }
    else if (event->key() == Qt::Key_Up)
    {
        if (recording)
        {
            command.ax = config.agentLinearAccelerationLimit;
            command.v = 0.8*config.agentLinearVelocityLimitForward;
        }
        else
        {
            fim = 10;
            animationTimer.start();
        }
    }
    else if (event->key() == Qt::Key_Down)
    {
        if (recording)
        {
            command.ax = -config.agentLinearAccelerationLimit;
            command.v = -0.8*config.agentLinearVelocityLimitBackward;
        }
        else
        {
            fim = -10;
            animationTimer.start();
        }
    }
    else if (event->key() == Qt::Key_Right)
    {
        if (recording)
        {
            command.ay = -config.agentAngularAccelerationLimit;
            command.w = -0.8*config.agentAngularVelocityLimit;
        }
        else
        {
            frameForward();
        }
    }
    else if (event->key() == Qt::Key_Left)
    {
        if (recording)
        {
            command.ay = config.agentAngularAccelerationLimit;
            command.w = 0.8*config.agentAngularVelocityLimit;
        }
        else
        {
            frameBack();
        }
    }
    else if (event->key() == Qt::Key_Plus)
    {
        config.debugLevel = config.debugLevel+1;
        configChanged();
    }
    else if (event->key() == Qt::Key_Minus)
    {
        config.debugLevel = config.debugLevel-1;
        configChanged();
    }

    else
    {
        //landscapeWidget.keyPressEvent(event);
    }
}

void MainWindow::keyReleaseEvent(QKeyEvent *event)
{
    if (event->isAutoRepeat())
        return;

    if (event->key() == Qt::Key_Up)
    {
        if (recording)
        {
            command.ax = 0;
            command.v = 0;
        }
        else
        {
            fim = 0;
            animationTimer.stop();
        }
    }
    else if (event->key() == Qt::Key_Down)
    {
        if (recording)
        {
            command.ax = 0;
            command.v = 0;
        }
        else
        {
            fim = 0;
            animationTimer.stop();
        }
    }
    else if (event->key() == Qt::Key_Right)
    {
        if (recording)
        {
            command.ay = 0;
            command.w = 0;
        }
    }
    else if (event->key() == Qt::Key_Left)
    {
        if (recording)
        {
            command.ay = 0;
            command.w = 0;
        }
    }
}
