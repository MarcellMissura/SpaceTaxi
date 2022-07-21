#include "spacetaxi.h"
#include <QMainWindow>
#include <QMenuBar>
#include <QDesktopWidget>

SpaceTaxi::SpaceTaxi(QWidget *parent)
    : QMainWindow(parent)
{
    QCoreApplication::setOrganizationName("MarcellMissura");
    QCoreApplication::setApplicationName("SpaceTaxi");

    QWidget* cw = new QWidget();
    ui.setupUi(cw); // Here, the ui file (spacetaxi.ui) is built into the central widget.
    setCentralWidget(cw); // The central widget has to be set because it's a QMainWindow.
    resize(QDesktopWidget().availableGeometry(this).size() * 0.5); // Resize to 50% of the screen.

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

    connect(&mainControlLoop, SIGNAL(messageOut(QString)), this, SLOT(messageIn(QString)));
    connect(&mainControlLoop, SIGNAL(configChangedOut()), this, SLOT(configChanged()));

    // Set the joystick into polling / event based mode and connect the joystick signals.
    joystick.startPolling(10);
    connect(&joystick, SIGNAL(connected()), this, SLOT(joystickConnected()));
    connect(&joystick, SIGNAL(disconnected()), this, SLOT(joystickDisconnected()));
    connect(&joystick, SIGNAL(buttonPressed(QList<bool>)), this, SLOT(joystickButtonPressed(QList<bool>)));
    connect(&joystick, SIGNAL(joystickMoved(QList<double>)), this, SLOT(joystickMoved(QList<double>)));

    toggleGraph();
    toggleConfig();
    //showFullScreen();

    // Animation components.
    currentStateIndex = 0;
    tscale = 2.0;
    tscaleBefore = 0.2;
    playDirection = 1.0;
    recording = false;
    graphicsViewWidget.recording = false;
    guiUpdateTimer.setInterval(30);
    connect(&guiUpdateTimer, SIGNAL(timeout()), this, SLOT(animate()));
    connect(this, SIGNAL(frameChanged()), &graphicsScene, SLOT(init()));
    connect(this, SIGNAL(frameChanged()), &graphicsViewWidget, SLOT(update()));
    connect(this, SIGNAL(frameChanged()), &graphWidget, SLOT(update()));
}

SpaceTaxi::~SpaceTaxi()
{
    QSettings settings;
    settings.setValue("verticalSplitterTop", verticalSplitterTop->saveState());
    settings.setValue("verticalSplitterBottom", verticalSplitterBottom->saveState());
    settings.setValue("horizontalSplitter", horizontalSplitter->saveState());
}

// Build the menu bar.
void SpaceTaxi::buildMenu()
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

    QAction* showTrajectoryTraceAction = viewMenu->addAction(tr("&Trajectory Trace"));
    showTrajectoryTraceAction->setToolTip(tr("Toggles the trajectory trace."));
    //showTrajectoryTraceAction->setShortcut(QKeySequence(tr("T")));
    showTrajectoryTraceAction->setCheckable(true);
    showTrajectoryTraceAction->setChecked(command.showTrajectoryTrace);
    connect(showTrajectoryTraceAction, SIGNAL(triggered()), this, SLOT(toggleTrajectoryTrace()));

    QAction* showPlanTraceAction = viewMenu->addAction(tr("&Plan Trace"));
    showPlanTraceAction->setToolTip(tr("Toggles the plan trace."));
    //showPlanTraceAction->setShortcut(QKeySequence(tr("T")));
    showPlanTraceAction->setCheckable(true);
    showPlanTraceAction->setChecked(command.showPlanTrace);
    connect(showPlanTraceAction, SIGNAL(triggered()), this, SLOT(togglePlanTrace()));

    viewMenu->addSeparator();

    QAction* showBodyAction = viewMenu->addAction(tr("&Body"));
    showBodyAction->setToolTip(tr("Toggles the body."));
    showBodyAction->setCheckable(true);
    showBodyAction->setChecked(command.showBody);
    connect(showBodyAction, SIGNAL(triggered()), this, SLOT(toggleBody()));

    QAction* showDropOffPointsAction = viewMenu->addAction(tr("&DropOff Points"));
    showDropOffPointsAction->setToolTip(tr("Toggles the drop off points."));
    showDropOffPointsAction->setCheckable(true);
    showDropOffPointsAction->setChecked(command.showDropOffPoints);
    connect(showDropOffPointsAction, SIGNAL(triggered()), this, SLOT(toggleDropOffPoints()));

    QAction* showTargetsAction = viewMenu->addAction(tr("&Targets"));
    showTargetsAction->setToolTip(tr("Toggles the targets."));
    showTargetsAction->setCheckable(true);
    showTargetsAction->setChecked(command.showTargets);
    connect(showTargetsAction, SIGNAL(triggered()), this, SLOT(toggleTargets()));

    QAction* showWorldObstaclesAction = viewMenu->addAction(tr("&World Obstacles"));
    showWorldObstaclesAction->setToolTip(tr("Toggles the world obstacles."));
    showWorldObstaclesAction->setShortcut(QKeySequence(tr("O")));
    showWorldObstaclesAction->setCheckable(true);
    showWorldObstaclesAction->setChecked(command.showWorldObstacles);
    connect(showWorldObstaclesAction, SIGNAL(triggered()), this, SLOT(toggleWorldObstacles()));

    QAction* showWorldMapAction = viewMenu->addAction(tr("&World Map"));
    showWorldMapAction->setToolTip(tr("Toggles the world map."));
    showWorldMapAction->setShortcut(QKeySequence(tr("W")));
    showWorldMapAction->setCheckable(true);
    showWorldMapAction->setChecked(command.showWorldMap);
    connect(showWorldMapAction, SIGNAL(triggered()), this, SLOT(toggleWorldMap()));

    QAction* showExpandedWorldMapAction = viewMenu->addAction(tr("&Expanded World Map"));
    showExpandedWorldMapAction->setToolTip(tr("Toggles the expanded world map."));
    //showExpandedWorldMapAction->setShortcut(QKeySequence(tr("W")));
    showExpandedWorldMapAction->setCheckable(true);
    showExpandedWorldMapAction->setChecked(command.showExpandedWorldMap);
    connect(showExpandedWorldMapAction, SIGNAL(triggered()), this, SLOT(toggleExpandedWorldMap()));

    QAction* showRayModelAction = viewMenu->addAction(tr("&Ray Model"));
    showRayModelAction->setToolTip(tr("Toggles the ray model."));
    showRayModelAction->setShortcut(QKeySequence(tr("Y")));
    showRayModelAction->setCheckable(true);
    showRayModelAction->setChecked(command.showRayModel);
    connect(showRayModelAction, SIGNAL(triggered()), this, SLOT(toggleRayModel()));

    QAction* showOccupancyGridAction = viewMenu->addAction(tr("&Sensed Grid"));
    showOccupancyGridAction->setShortcut(QKeySequence(tr("S")));
    showOccupancyGridAction->setCheckable(true);
    showOccupancyGridAction->setChecked(command.showSensedGrid > 0);
    connect(showOccupancyGridAction, SIGNAL(triggered()), this, SLOT(toggleSensedGrid()));

    QAction* showWorldModelAction = viewMenu->addAction(tr("&Sensed Polygons"));
    showWorldModelAction->setShortcut(QKeySequence(tr("P")));
    showWorldModelAction->setCheckable(true);
    showWorldModelAction->setChecked(command.showGeometricModel);
    connect(showWorldModelAction, SIGNAL(triggered()), this, SLOT(toggleGeometricModel()));

    QAction* showDijkstraMapAction = viewMenu->addAction(tr("&Dijkstra Map"));
    showDijkstraMapAction->setShortcut(QKeySequence(tr("D")));
    showDijkstraMapAction->setCheckable(true);
    showDijkstraMapAction->setChecked(command.showDijkstraMap);
    connect(showDijkstraMapAction, SIGNAL(triggered()), this, SLOT(toggleDijkstraMap()));

    QAction* showWorldVisGraphAction = viewMenu->addAction(tr("&World Visibility Graph"));
    showWorldVisGraphAction->setToolTip(tr("Toggles the worls visiblity graph."));
    //showVisGraphAction->setShortcut(QKeySequence(tr("V")));
    showWorldVisGraphAction->setCheckable(true);
    showWorldVisGraphAction->setChecked(command.showWorldVisibilityGraph);
    connect(showWorldVisGraphAction, SIGNAL(triggered()), this, SLOT(toggleWorldVisibilityGraph()));

    QAction* showLocalVisGraphAction = viewMenu->addAction(tr("&Local Visibility Graph"));
    showLocalVisGraphAction->setToolTip(tr("Toggles the local visiblity graph."));
    showLocalVisGraphAction->setShortcut(QKeySequence(tr("V")));
    showLocalVisGraphAction->setCheckable(true);
    showLocalVisGraphAction->setChecked(command.showLocalVisibilityGraph);
    connect(showLocalVisGraphAction, SIGNAL(triggered()), this, SLOT(toggleLocalVisibilityGraph()));

    showWorldPathAction = viewMenu->addAction(tr("&World Path"));
    showWorldPathAction->setToolTip(tr("Toggles the world path."));
    //showWorldPathAction->setShortcut(QKeySequence(tr("W")));
    showWorldPathAction->setCheckable(true);
    showWorldPathAction->setChecked(command.showWorldPath);
    connect(showWorldPathAction, SIGNAL(triggered()), this, SLOT(toggleWorldPath()));

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

    QAction* outdoorAction = mapActionGroup->addAction(tr("&Outdoor"));
    mapMenu->addAction(outdoorAction);
    outdoorAction->setShortcut(QKeySequence(tr("3")));
    outdoorAction->setCheckable(true);
    outdoorAction->setChecked(state.world.mapId == 3);
    connect(outdoorAction, SIGNAL(triggered()), &mapSelectionMapper, SLOT(map()));

    QAction* apartmentAction = mapActionGroup->addAction(tr("&Apartment"));
    mapMenu->addAction(apartmentAction);
    apartmentAction->setShortcut(QKeySequence(tr("4")));
    apartmentAction->setCheckable(true);
    apartmentAction->setChecked(state.world.mapId == 4);
    connect(apartmentAction, SIGNAL(triggered()), &mapSelectionMapper, SLOT(map()));

    QAction* wareHouseAction = mapActionGroup->addAction(tr("&Warehouse"));
    mapMenu->addAction(wareHouseAction);
    wareHouseAction->setShortcut(QKeySequence(tr("5")));
    wareHouseAction->setCheckable(true);
    wareHouseAction->setChecked(state.world.mapId == 5);
    connect(wareHouseAction, SIGNAL(triggered()), &mapSelectionMapper, SLOT(map()));

    QAction* officeAction = mapActionGroup->addAction(tr("&Office"));
    mapMenu->addAction(officeAction);
    officeAction->setShortcut(QKeySequence(tr("6")));
    officeAction->setCheckable(true);
    officeAction->setChecked(state.world.mapId == 6);
    connect(officeAction, SIGNAL(triggered()), &mapSelectionMapper, SLOT(map()));

    QAction* clutterAction = mapActionGroup->addAction(tr("&Clutter"));
    mapMenu->addAction(clutterAction);
    clutterAction->setShortcut(QKeySequence(tr("7")));
    clutterAction->setCheckable(true);
    clutterAction->setChecked(state.world.mapId == 7);
    connect(clutterAction, SIGNAL(triggered()), &mapSelectionMapper, SLOT(map()));

    mapSelectionMapper.setMapping(voidAction, 0);
    mapSelectionMapper.setMapping(simpleAction, 1);
    mapSelectionMapper.setMapping(uAction, 2);
    mapSelectionMapper.setMapping(outdoorAction, 3);
    mapSelectionMapper.setMapping(apartmentAction, 4);
    mapSelectionMapper.setMapping(wareHouseAction, 5);
    mapSelectionMapper.setMapping(officeAction, 6);
    mapSelectionMapper.setMapping(clutterAction, 7);
    connect(&mapSelectionMapper, SIGNAL(mapped(int)), this, SLOT(toggleMap(int)));


    QMenu* controllerMenu = menuBar->addMenu(tr("&Controller"));

    QActionGroup* pathPlannerActionGroup = new QActionGroup(controllerMenu);

    QAction* pathAstarAction = pathPlannerActionGroup->addAction(tr("&Grid A*"));
    controllerMenu->addAction(pathAstarAction);
    pathAstarAction->setCheckable(true);
    pathAstarAction->setChecked(command.pathPlanningMethod == command.GridAStar);
    connect(pathAstarAction, SIGNAL(triggered()), &pathSelectionMapper, SLOT(map()));

    QAction* lazyThetaStarAction = pathPlannerActionGroup->addAction(tr("&Grid Lazy Theta*"));
    controllerMenu->addAction(lazyThetaStarAction);
    //lazyThetaStarAction->setShortcut(QKeySequence(tr("L")));
    lazyThetaStarAction->setCheckable(true);
    lazyThetaStarAction->setChecked(command.pathPlanningMethod == command.LazyThetaStar);
    connect(lazyThetaStarAction, SIGNAL(triggered()), &pathSelectionMapper, SLOT(map()));

    QAction* dijkstraAction = pathPlannerActionGroup->addAction(tr("&Dijkstra Map"));
    controllerMenu->addAction(dijkstraAction);
    dijkstraAction->setCheckable(true);
    dijkstraAction->setChecked(command.pathPlanningMethod == command.Dijkstra);
    connect(dijkstraAction, SIGNAL(triggered()), &pathSelectionMapper, SLOT(map()));

    QAction* naiveConstructAction = pathPlannerActionGroup->addAction(tr("&Visibility Graph Naive"));
    controllerMenu->addAction(naiveConstructAction);
    naiveConstructAction->setCheckable(true);
    naiveConstructAction->setChecked(command.pathPlanningMethod == command.NaiveConstruct);
    connect(naiveConstructAction, SIGNAL(triggered()), &pathSelectionMapper, SLOT(map()));

    QAction* minimalConstructAction = pathPlannerActionGroup->addAction(tr("&Minimal Construct"));
    controllerMenu->addAction(minimalConstructAction);
    minimalConstructAction->setShortcut(QKeySequence(tr("M")));
    minimalConstructAction->setCheckable(true);
    minimalConstructAction->setChecked(command.pathPlanningMethod == command.MinimalConstruct);
    connect(minimalConstructAction, SIGNAL(triggered()), &pathSelectionMapper, SLOT(map()));

    pathSelectionMapper.setMapping(pathAstarAction, command.GridAStar);
    pathSelectionMapper.setMapping(lazyThetaStarAction, command.LazyThetaStar);
    pathSelectionMapper.setMapping(dijkstraAction, command.Dijkstra);
    pathSelectionMapper.setMapping(naiveConstructAction, command.NaiveConstruct);
    pathSelectionMapper.setMapping(minimalConstructAction, command.MinimalConstruct);
    connect(&pathSelectionMapper, SIGNAL(mapped(int)), this, SLOT(togglePathPlanner(int)));

    controllerMenu->addSeparator();

    QActionGroup* trajectoryPlannerActionGroup = new QActionGroup(controllerMenu);

    QAction* pdAction = trajectoryPlannerActionGroup->addAction(tr("&PD Control"));
    controllerMenu->addAction(pdAction);
    pdAction->setCheckable(true);
    pdAction->setChecked(command.trajectoryPlanningMethod == command.PD);
    connect(pdAction, SIGNAL(triggered()), &trajectoryPlannerSelectionMapper, SLOT(map()));

    QAction* dwaAction = trajectoryPlannerActionGroup->addAction(tr("&DWA"));
    controllerMenu->addAction(dwaAction);
    dwaAction->setCheckable(true);
    dwaAction->setChecked(command.trajectoryPlanningMethod == command.DWA);
    connect(dwaAction, SIGNAL(triggered()), &trajectoryPlannerSelectionMapper, SLOT(map()));

    QAction* staasAction = trajectoryPlannerActionGroup->addAction(tr("&ST Aborting A*"));
    controllerMenu->addAction(staasAction);
    staasAction->setCheckable(true);
    staasAction->setChecked(command.trajectoryPlanningMethod == command.STAA);
    connect(staasAction, SIGNAL(triggered()), &trajectoryPlannerSelectionMapper, SLOT(map()));

    QAction* rbAction = trajectoryPlannerActionGroup->addAction(tr("&RuleBase"));
    controllerMenu->addAction(rbAction);
    rbAction->setCheckable(true);
    rbAction->setChecked(command.trajectoryPlanningMethod == command.RuleBase);
    connect(rbAction, SIGNAL(triggered()), &trajectoryPlannerSelectionMapper, SLOT(map()));

    trajectoryPlannerSelectionMapper.setMapping(pdAction, command.PD);
    trajectoryPlannerSelectionMapper.setMapping(dwaAction, command.DWA);
    trajectoryPlannerSelectionMapper.setMapping(rbAction, command.RuleBase);
    trajectoryPlannerSelectionMapper.setMapping(staasAction, command.STAA);
    connect(&trajectoryPlannerSelectionMapper, SIGNAL(mapped(int)), this, SLOT(toggleTrajectoryControl(int)));

    controllerMenu->addSeparator();

    QAction* forceFieldReflexAction = controllerMenu->addAction(tr("&Force Field Reflex"));
    forceFieldReflexAction->setToolTip(tr("Toggles the force field reflex."));
    //forceFieldReflexAction->setShortcut(QKeySequence(tr("F")));
    forceFieldReflexAction->setCheckable(true);
    forceFieldReflexAction->setChecked(command.forceFieldReflex);
    connect(forceFieldReflexAction, SIGNAL(triggered()), this, SLOT(toggleForceFieldReflex()));

    QAction* stucknessReflexAction = controllerMenu->addAction(tr("&Stuckness Reflex"));
    stucknessReflexAction->setToolTip(tr("Toggles the stuckness reflex."));
    //stucknessReflexAction->setShortcut(QKeySequence(tr("F")));
    stucknessReflexAction->setCheckable(true);
    stucknessReflexAction->setChecked(command.stucknessReflex);
    connect(stucknessReflexAction, SIGNAL(triggered()), this, SLOT(toggleStucknessReflex()));


    QMenu* commandMenu = menuBar->addMenu(tr("&Command"));

    QAction* experimenterAction = commandMenu->addAction(tr("&Experimenter"));
    experimenterAction->setToolTip(tr("Toggles experimenter."));
    experimenterAction->setShortcut(QKeySequence(tr("E")));
    experimenterAction->setCheckable(true);
    experimenterAction->setChecked(experimenter.running);
    connect(experimenterAction, SIGNAL(triggered()), &experimenter, SLOT(startstop()));

    commandMenu->addSeparator();

    QAction* teachingAction = commandMenu->addAction(tr("&Teaching"));
    teachingAction->setToolTip(tr("Toggles the teaching mode."));
    teachingAction->setShortcut(QKeySequence(tr("L")));
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

    QAction* ghostModeAction = controllerMenu->addAction(tr("&Ghost Mode"));
    ghostModeAction->setToolTip(tr("Toggles the dumb agents flag."));
    //forceFieldReflexAction->setShortcut(QKeySequence(tr("F")));
    ghostModeAction->setCheckable(true);
    ghostModeAction->setChecked(command.ghostMode);
    connect(ghostModeAction, SIGNAL(triggered()), this, SLOT(toggleGhostMode()));


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
    heuristicSelectionMapper.setMapping(rtrMinAction, command.RTR_MIN);
    heuristicSelectionMapper.setMapping(rtrMaxAction, command.RTR_MAX);
    heuristicSelectionMapper.setMapping(mcAction, command.MinimalConstruct);
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

    QAction* hz10Action = frequencyActionGroup->addAction(tr("&10Hz"));
    commandMenu->addAction(hz10Action);
    hz10Action->setCheckable(true);
    hz10Action->setChecked(false);
    connect(hz10Action, SIGNAL(triggered()), &frequencySelectionMapper, SLOT(map()));

    QAction* hz20Action = frequencyActionGroup->addAction(tr("&20Hz"));
    commandMenu->addAction(hz20Action);
    hz20Action->setCheckable(true);
    hz20Action->setChecked(false);
    connect(hz20Action, SIGNAL(triggered()), &frequencySelectionMapper, SLOT(map()));

    QAction* hz30Action = frequencyActionGroup->addAction(tr("&30Hz"));
    commandMenu->addAction(hz30Action);
    hz30Action->setCheckable(true);
    hz30Action->setChecked(true);
    connect(hz30Action, SIGNAL(triggered()), &frequencySelectionMapper, SLOT(map()));

    frequencySelectionMapper.setMapping(hz10Action, 10);
    frequencySelectionMapper.setMapping(hz20Action, 20);
    frequencySelectionMapper.setMapping(hz30Action, 30);
    connect(&frequencySelectionMapper, SIGNAL(mapped(int)), this, SLOT(toggleFrequency(int)));


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
void SpaceTaxi::topSplitterMoved()
{
    verticalSplitterBottom->setSizes(verticalSplitterTop->sizes());
}
void SpaceTaxi::bottomSplitterMoved()
{
    verticalSplitterTop->setSizes(verticalSplitterBottom->sizes());
}

void SpaceTaxi::joystickConnected()
{
    messageIn("Joystick connected.");
    joystickAction->setVisible(true);
    joystickAction->setChecked(true);
    command.joystick = true;
}
void SpaceTaxi::joystickDisconnected()
{
    messageIn("Joystick disconnected.");
    joystickAction->setVisible(false);
    command.joystick = false;
}

// Handles the joystick buttons. Some of them select and move sliders in the config widget.
// Some control the robot by switching between walk and halt, executing kicks etc.
void SpaceTaxi::joystickButtonPressed(QList<bool> button)
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
void SpaceTaxi::joystickMoved(QList<double> axis)
{
    command.ax = axis[1];
    command.ay = axis[0];
}

// This is needed when the entire config has changed, e.g. when the robot model was
// changed, the config was reset or a new robot was detected.
void SpaceTaxi::configChanged()
{
    configWidget.configChangedIn();
    graphicsViewWidget.update();
}

// Slot for internal message strings.
void SpaceTaxi::messageIn(QString m)
{
    graphicsViewWidget.messageIn(m);
}

// Toggles the config widget.
void SpaceTaxi::toggleConfig()
{
    if (configWidget.isHidden())
        configWidget.show();
    else
        configWidget.hide();
}

// Toggles the graph widget.
void SpaceTaxi::toggleGraph()
{
    if (verticalSplitterBottom->isHidden())
        verticalSplitterBottom->show();
    else
        verticalSplitterBottom->hide();
}

void SpaceTaxi::toggleJoystick()
{
    command.joystick = !command.joystick;
    if (command.joystick)
        messageIn("Joystick control activated.");
    else
        messageIn("Joystick control deactivated.");
}

void SpaceTaxi::toggleKeyboard()
{
    command.keyboard = !command.keyboard;
    if (command.keyboard)
        messageIn("Keyboard control enabled.");
    else
        messageIn("Keyboard control disabled.");
}

void SpaceTaxi::toggleDebug()
{
    command.debug = !command.debug;
    if (command.debug)
        messageIn("Debug is enabled.");
    else
        messageIn("Debug is disabled.");
}

void SpaceTaxi::toggleTrajectoryTrace()
{
    command.showTrajectoryTrace = !command.showTrajectoryTrace;
    update();
}

void SpaceTaxi::togglePlanTrace()
{
    command.showPlanTrace = !command.showPlanTrace;
    update();
}

void SpaceTaxi::toggleDropOffPoints()
{
    command.showDropOffPoints = !command.showDropOffPoints;
    graphicsScene.init();
    update();
}

void SpaceTaxi::toggleTargets()
{
    command.showTargets = !command.showTargets;
    update();
}

void SpaceTaxi::toggleWorldObstacles()
{
    command.showWorldObstacles = !command.showWorldObstacles;
    graphicsScene.init();
    update();
}

void SpaceTaxi::toggleWorldMap()
{
    command.showWorldMap = !command.showWorldMap;
    command.showExpandedWorldMap = !command.showExpandedWorldMap;
    update();
}

void SpaceTaxi::toggleExpandedWorldMap()
{
    command.showExpandedWorldMap = !command.showExpandedWorldMap;
    update();
}

void SpaceTaxi::toggleWorldPath()
{
    command.showWorldPath = !command.showWorldPath;
    update();
}

void SpaceTaxi::toggleGeometricModel()
{
    command.showGeometricModel = !command.showGeometricModel;
    update();
}

void SpaceTaxi::toggleSimulationDebug()
{
    command.showSimulationDebugDraw = !command.showSimulationDebugDraw;
    update();
}

void SpaceTaxi::toggleRayModel()
{
    command.showRayModel = !command.showRayModel;
    update();
}

void SpaceTaxi::toggleWorldVisibilityGraph()
{
    command.showWorldVisibilityGraph = !command.showWorldVisibilityGraph;
    update();
}

void SpaceTaxi::toggleLocalVisibilityGraph()
{
    command.showLocalVisibilityGraph = !command.showLocalVisibilityGraph;
    update();
}

void SpaceTaxi::toggleTeaching()
{
    command.learn = !command.learn;
    if(command.learn)
        messageIn("Teaching enabled!");
    else
        messageIn("Teaching off.");
}

void SpaceTaxi::clearRuleBase()
{
    state.world.unicycleAgents[0].ruleBase.clear();
    messageIn("Rule base cleared.");
}

void SpaceTaxi::toggleSensedGrid()
{
    command.showSensedGrid = (command.showSensedGrid+1) % 3;
    update();
}

void SpaceTaxi::toggleDijkstraMap()
{
    command.showDijkstraMap = !command.showDijkstraMap;
    update();
}

void SpaceTaxi::toggleForceFieldReflex()
{
    command.forceFieldReflex = !command.forceFieldReflex;
    if (command.forceFieldReflex)
        messageIn("Force field reflex enabled.");
    else
        messageIn("Force field reflex disabled.");
    update();
}

void SpaceTaxi::toggleStucknessReflex()
{
    command.stucknessReflex = !command.stucknessReflex;
    if (command.stucknessReflex)
        messageIn("Stuckness reflex enabled.");
    else
        messageIn("Stuckness reflex disabled.");
    update();
}

void SpaceTaxi::toggleTimeAbort()
{
    command.useTimeAbort = !command.useTimeAbort;
    if (command.useTimeAbort)
        messageIn("Time abort enabled.");
    else
        messageIn("Time abort disabled.");
    update();
}

void SpaceTaxi::toggleClosing()
{
    command.useClosing = !command.useClosing;
    if (command.useClosing)
        messageIn("Closing enabled.");
    else
        messageIn("Closing disabled.");
    update();
}

void SpaceTaxi::toggleDynamicPath()
{
    command.useDynamicPath = !command.useDynamicPath;
    if (command.useDynamicPath)
        messageIn("Dynamic path enabled.");
    else
        messageIn("Dynamic path disabled.");
    update();
}

void SpaceTaxi::toggleBody()
{
    command.showBody = !command.showBody;
    graphicsViewWidget.update();
}

void SpaceTaxi::toggleGhostMode()
{
    command.ghostMode = !command.ghostMode;
    if (command.ghostMode)
        messageIn("Ghost mode enabled.");
    else
        messageIn("Ghost mode disabled.");
}

void SpaceTaxi::togglePathPlanner(int d)
{
    command.pathPlanningMethod = d;
    update();
}

void SpaceTaxi::toggleTrajectoryControl(int d)
{
    command.trajectoryPlanningMethod = d;
    state.world.setParams(command.trajectoryPlanningMethod, command.trajectoryType, command.predictionType, command.heuristic, command.frequency);

    if (d == command.DWA)
        messageIn("DWA");
    else if (d == command.STAA)
        messageIn("ST Aborting A*");
    else if (d == command.PD)
        messageIn("PD Control");
    else if (d == command.RuleBase)
        messageIn("RuleBase");
    else
        messageIn("Invalid Trajectory Planner " + d);

    update();
}

void SpaceTaxi::togglePredictionType(int d)
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

void SpaceTaxi::toggleHeuristic(int d)
{
    command.heuristic = d;
    state.world.setParams(command.trajectoryPlanningMethod, command.trajectoryType, command.predictionType, command.heuristic, command.frequency);

    if (d == command.Euklidean)
        messageIn("Euklidean heuristic");
    else if (d == command.PathEuklidean)
        messageIn("PathEuklidean heuristic");
    else if (d == command.RTR)
        messageIn("RTR heuristic");
    else if (d == command.RTR_MIN)
        messageIn("RTR Min heuristic");
    else if (d == command.RTR_MAX)
        messageIn("RTR Max heuristic");
    else if (d == command.MinimalConstruct)
        messageIn("Minimal Construct heuristic");
    else if (d == command.GridDijkstra)
        messageIn("Grid Dijkstra heuristic");
}

void SpaceTaxi::toggleTrajectoryType(int d)
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

void SpaceTaxi::toggleFrequency(int f)
{
    command.frequency = f;
    state.world.setParams(command.trajectoryPlanningMethod, command.trajectoryType, command.predictionType, command.heuristic, command.frequency);

    if (f == 10)
        messageIn("Switched to 10Hz.");
    else if (f == 20)
        messageIn("Switched to 20Hz.");
    else
        messageIn("Switched to 30Hz.");
}

void SpaceTaxi::toggleMap(int w)
{
    state.frameId = 0;
    state.world.setMap(w, max(config.unicycleAgents, 1.0));
    graphicsScene.init();
    graphicsViewWidget.init();

    if (w == 0)
    {
        messageIn("Switched to Void map.");
    }
    if (w == 1)
    {
        messageIn("Switched to Simple map.");
    }
    if (w == 2)
    {
        messageIn("Switched to U Trap map.");
    }
    if (w == 3)
    {
        messageIn("Switched to Outdoor map.");
    }
    if (w == 4)
    {
        messageIn("Switched to Apartment map.");
    }
    if (w == 5)
    {
        messageIn("Switched to Warehouse map.");
    }
    if (w == 6)
    {
        messageIn("Switched to Office map.");
    }
    if (w == 7)
    {
        messageIn("Switched to Clutter map.");
    }
}

// This function is called by the animation timer to update the gui.
// The animation timer is running in record mode only.
// It's not running in playback mode.
void SpaceTaxi::animate()
{
    if (recording)
    {
        currentStateIndex = 0;
        loadFrame(0);
    }
    else
    {
        if ( (playDirection > 0 && currentStateIndex-playDirection*tscale >= 0)
                ||
            (playDirection < 0 && currentStateIndex-playDirection*tscale <= state.size()-1))
        {
            currentStateIndex = currentStateIndex-playDirection*tscale;
        }
        else
        {
            guiUpdateTimer.stop();
        }

        emit progressOut(qMax(0, (int)(1000.0*(state.size()-1-currentStateIndex)/(state.size()-1))));
        loadFrame(currentStateIndex);
    }
}

// Toggles the record mode. In record mode the robot control thread is running
// and data are collected in the state history.
void SpaceTaxi::record()
{
    if (recording)
    {
        recording = false;
        graphicsViewWidget.stopRecording();
        guiUpdateTimer.stop();
        mainControlLoop.stop();
    }
    else
    {
        recording = true;
        graphicsViewWidget.startRecording();
        guiUpdateTimer.start();
        mainControlLoop.start();
    }
}

// Toggles the record mode. In record mode the robot control thread is running and data are collected in the state history.
void SpaceTaxi::recordStart()
{
    recording = true;
    graphicsViewWidget.startRecording();
    guiUpdateTimer.start();
    mainControlLoop.start();
}

// Toggles the record mode. In record mode the robot control thread is running and data are collected in the state history.
void SpaceTaxi::recordStop()
{
    recording = false;
    graphicsViewWidget.stopRecording();
    mainControlLoop.stop();
    guiUpdateTimer.stop();
}

// Play button handler.
void SpaceTaxi::play()
{
    if (guiUpdateTimer.isActive())
    {
        stop();
    }
    else
    {
        if (!recording)
        {
            guiUpdateTimer.stop();
            playDirection = 1.0;
            guiUpdateTimer.start();
        }
    }
}

// Reverse button handler.
void SpaceTaxi::reverse()
{
    if (guiUpdateTimer.isActive())
    {
        stop();
    }
    else
    {
        if (!recording)
        {
            guiUpdateTimer.stop();
            playDirection = -1.0;
            guiUpdateTimer.start();
        }
    }
}

// Stop button handler.
void SpaceTaxi::stop()
{
    if (!recording)
    {
        guiUpdateTimer.stop();
        update();
    }
}

void SpaceTaxi::frameBack()
{
    if (!recording)
    {
        guiUpdateTimer.stop();
        if (currentStateIndex < state.size()-1)
            currentStateIndex++;
        emit progressOut(qMax(0, (int)(1000.0*(state.size()-1-currentStateIndex)/(state.size()-1))));
        loadFrame(currentStateIndex);
    }
}

void SpaceTaxi::frameForward()
{
    if (!recording)
    {
        guiUpdateTimer.stop();
        if (currentStateIndex > 0)
            currentStateIndex--;
        else
            mainControlLoop.step();

        emit progressOut(qMax(0, (int)(1000.0*(state.size()-1-currentStateIndex)/(state.size()-1))));
        loadFrame(currentStateIndex);
    }
}

void SpaceTaxi::jumpToStart()
{
    if (!recording)
    {
        guiUpdateTimer.stop();
        currentStateIndex = state.size()-1;
        emit progressOut(qMax(0, (int)(1000.0*(state.size()-1-currentStateIndex)/(state.size()-1))));
        loadFrame(currentStateIndex);
    }
}

void SpaceTaxi::jumpToFrame(int f)
{
    if (!recording)
    {
        guiUpdateTimer.stop();
        currentStateIndex = qMax(1, int(state.size()-1 - (double)((state.size()-1) * f)/1000));
        loadFrame(currentStateIndex);
    }
}

// This slot is called when the user browses the state history with the slider or with
// the frame forward, frame backward, and playback functions.
void SpaceTaxi::loadFrame(int cfi)
{
    if (cfi > 0)
    {
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
    }

    emit frameChanged();
}

void SpaceTaxi::jumpToEnd()
{
    if (!recording)
    {
        guiUpdateTimer.stop();
        currentStateIndex = 0;
        emit progressOut(qMax(0, (int)(1000.0*(state.size()-1-currentStateIndex)/(state.size()-1))));
        loadFrame(currentStateIndex);
    }
}

void SpaceTaxi::saveConfig()
{
    config.save(robotName);
    messageIn("Config saved.");
}

void SpaceTaxi::loadConfig()
{
    config.load(robotName);
    configChanged();
    messageIn("Config reset.");
}

void SpaceTaxi::saveStateHistory()
{
    if (recording)
        record();
    state.save();
    messageIn("State history saved.");
}

void SpaceTaxi::loadStateHistory(QString fileName)
{
    messageIn("Loading...");
    if (recording)
        record();
    state.load(fileName);
    jumpToStart();
    messageIn("State history loaded.");
}

void SpaceTaxi::saveWorld()
{
    state.world.save();
    messageIn("World saved.");
}

void SpaceTaxi::loadWorld()
{
    state.world.load();
    messageIn("World loaded.");
}

void SpaceTaxi::exportWorld()
{
    state.world.logObstaclesToTxt();
    messageIn("World exported.");
}

void SpaceTaxi::reset()
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
bool SpaceTaxi::eventFilter(QObject *obj, QEvent *event)
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

void SpaceTaxi::keyPressEvent(QKeyEvent *event)
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
        }
        else
        {
            playDirection = 1;
            tscaleBefore = tscale;
            tscale = 2.0;
            graphicsViewWidget.update();
            if (!guiUpdateTimer.isActive())
                guiUpdateTimer.start();
        }
    }
    else if (event->key() == Qt::Key_Down)
    {
        if (recording)
        {
            command.ax = -config.agentLinearAccelerationLimit;
        }
        else
        {
            playDirection = -1;
            tscaleBefore = tscale;
            tscale = 2.0;
            graphicsViewWidget.update();
            if (!guiUpdateTimer.isActive())
                guiUpdateTimer.start();
        }
    }
    else if (event->key() == Qt::Key_Right)
    {
        if (recording)
        {
            command.ay = -config.agentAngularAccelerationLimit;
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

void SpaceTaxi::keyReleaseEvent(QKeyEvent *event)
{
    if (event->isAutoRepeat())
        return;

    if (event->key() == Qt::Key_Up)
    {
        if (recording)
        {
            command.ax = 0.0;
        }
        else
        {
            tscale = tscaleBefore;
            graphicsViewWidget.update();
        }
    }
    else if (event->key() == Qt::Key_Down)
    {
        if (recording)
        {
            command.ax = 0.0;
        }
        else
        {
            playDirection = 1;
            tscale = tscaleBefore;
            graphicsViewWidget.update();
        }
    }
    else if (event->key() == Qt::Key_Right)
    {
        if (recording)
        {
            command.ay = 0.0;
        }
    }
    else if (event->key() == Qt::Key_Left)
    {
        if (recording)
        {
            command.ay = 0.0;
        }
    }
}
