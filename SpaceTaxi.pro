include(blackboard/blackboard.pri)
include(gui/gui.pri)
include(util/util.pri)
include(pml/pml.pri)
include(geometry/geometry.pri)
include(controller/controller.pri)
include(Box2D/box2d.pri)
include(KeyframePlayer/KeyframePlayer.pri)
TEMPLATE = app
TARGET = SpaceTaxi
QT += core \
    gui \
    xml \
    opengl \
    concurrent
HEADERS += agents/Obstacle.h \
    agents/UnicycleObstacle.h \
    agents/HolonomicObstacle.h \
    agents/UnicycleAgent.h \
    Experimenter.h \
    ExperimentConfig.h \
    World.h \
    spacetaxi.h \
    MainControlLoop.h \
    globals.h
SOURCES += agents/Obstacle.cpp \
    agents/HolonomicObstacle.cpp \
    agents/UnicycleObstacle.cpp \
    agents/UnicycleAgent.cpp \
    Experimenter.cpp \
    ExperimentConfig.cpp \
    World.cpp \
    MainControlLoop.cpp \
    spacetaxi.cpp \
    main.cpp
FORMS += spacetaxi.ui
RESOURCES +=
CONFIG += console
CONFIG += warn_off
CONFIG += c++11
INCLUDEPATH += /usr/include/opencv4
LIBS += -L/usr/include/opencv4 -lopencv_core -lopencv_imgproc
LIBS += -L/usr/include/GL -lGLEW -lglut -lGLU -lGL
INCLUDEPATH += /usr/include/polyclipping
LIBS += -lpolyclipping
QMAKE_CXXFLAGS_DEBUG += -O0
QMAKE_CXXFLAGS_RELEASE -= -O2
QMAKE_CXXFLAGS_RELEASE += -O3
#QMAKE_CXXFLAGS_RELEASE += -ffast-math

