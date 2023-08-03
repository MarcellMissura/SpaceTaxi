HEADERS += lib/util/Timer.h \
    lib/util/Joystick.h \
    lib/util/StopWatch.h \
    lib/util/VecN.h \
    lib/util/VecNi.h \
    lib/util/VecNu.h \
    lib/util/Vec3.h \
    lib/util/Vec2.h \
    lib/util/Vec3i.h \
    lib/util/Vec2i.h \
    lib/util/Vec3u.h \
    lib/util/Vec2u.h \
    lib/util/Grid.h \
    lib/util/DataGrid.h \
    lib/util/Logger.h \
    lib/util/FileLoader.h \
    lib/util/Statistics.h \
    lib/util/DrawUtil.h \
    lib/util/RGBPixel.h \
    lib/util/PriorityQueue.h \
    lib/util/LinkedList.h \
    lib/util/Vector.h \
    lib/util/RingBuffer.h \
    lib/util/AdjacencyMatrix.h \
    lib/util/fresnelnr.h \
    lib/util/GLlib.h \
    lib/util/Transform3D.h \
    lib/util/Transform2D.h \
    lib/util/Pose2D.h
SOURCES += \
    lib/util/StopWatch.cpp \
    lib/util/Timer.cpp \
    lib/util/Grid.cpp \
    lib/util/Logger.cpp \
    lib/util/Statistics.cpp \
    lib/util/DrawUtil.cpp \
    lib/util/RGBPixel.cpp \
    lib/util/FileLoader.cpp \
    lib/util/AdjacencyMatrix.cpp \
    lib/util/fresnelnr.cpp \
    lib/util/GLlib.cpp \
    lib/util/Transform3D.cpp \
    lib/util/Transform2D.cpp \
    lib/util/Pose2D.cpp
win32:HEADERS += lib/util/TimerWindows.h
win32:SOURCES += lib/util/TimerWindows.cpp
win32:HEADERS += lib/util/StopWatchWindows.h
win32:SOURCES += lib/util/StopWatchWindows.cpp
win32:SOURCES += lib/util/JoystickWindows.cpp
linux:HEADERS += lib/util/TimerLinux.h
linux:SOURCES += lib/util/TimerLinux.cpp
linux:HEADERS += lib/util/StopWatchLinux.h
linux:SOURCES += lib/util/StopWatchLinux.cpp
linux:SOURCES += lib/util/JoystickLinux.cpp
