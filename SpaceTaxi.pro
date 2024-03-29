include(board/board.pri)
include(gui/gui.pri)
include(lib/lib.pri)
include(robotcontrol/robotcontrol.pri)
TEMPLATE = app
TARGET = SpaceTaxi
QT += core \
    gui \
    xml \
    opengl \
    concurrent
HEADERS += Experimenter.h \
    ExperimentConfig.h \
    World.h \
    MainWindow.h \
    MainControlLoop.h
SOURCES += Experimenter.cpp \
    ExperimentConfig.cpp \
    World.cpp \
    MainControlLoop.cpp \
    MainWindow.cpp \
    main.cpp
FORMS += gui.ui
RESOURCES +=
CONFIG += console
CONFIG += warn_off
CONFIG += c++14
INCLUDEPATH += /usr/include/opencv4
LIBS += -L/usr/include/opencv4 -lopencv_core -lopencv_imgproc
LIBS += -L/usr/include/GL -lGLEW -lglut -lGLU -lGL
QMAKE_CXXFLAGS_DEBUG += -O0
QMAKE_CXXFLAGS_RELEASE -= -O2
QMAKE_CXXFLAGS_RELEASE += -O3
#QMAKE_CXXFLAGS_RELEASE += -ffast-math

