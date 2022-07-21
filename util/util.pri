HEADERS += util/Timer.h \
    util/Joystick.h \
    util/StopWatch.h \
    util/VecN.h \
    util/VecNi.h \
    util/VecNu.h \
    util/Vec3.h \
    util/Vec2.h \
    util/Vec3i.h \
    util/Vec2i.h \
    util/Vec3u.h \
    util/Vec2u.h \
    util/Grid.h \
    util/DataGrid.h \
    util/Logger.h \
    util/FileLoader.h \
    util/Statistics.h \
    util/ColorUtil.h \
    util/PriorityQueue.h \
    util/LinkedList.h \
    util/Vector.h \
    util/AdjacencyMatrix.h \
    util/fresnelnr.h \
    util/GLlib.h \
    util/Transform3D.h \
    util/Transform2D.h \
    util/Pose2D.h
SOURCES += \
    util/StopWatch.cpp \
    util/Timer.cpp \
    util/Grid.cpp \
    util/Logger.cpp \
    util/Statistics.cpp \
    util/ColorUtil.cpp \
    util/FileLoader.cpp \
    util/AdjacencyMatrix.cpp \
    util/fresnelnr.cpp \
    util/GLlib.cpp \
    util/Transform3D.cpp \
    util/Transform2D.cpp \
    util/Pose2D.cpp
win32:HEADERS += util/TimerWindows.h
win32:SOURCES += util/TimerWindows.cpp
win32:HEADERS += util/StopWatchWindows.h
win32:SOURCES += util/StopWatchWindows.cpp
win32:SOURCES += util/JoystickWindows.cpp
linux:HEADERS += util/TimerLinux.h
linux:SOURCES += util/TimerLinux.cpp
linux:HEADERS += util/StopWatchLinux.h
linux:SOURCES += util/StopWatchLinux.cpp
linux:SOURCES += util/JoystickLinux.cpp
