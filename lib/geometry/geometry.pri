include(clipper2/clipper.pri)
HEADERS += lib/geometry/Line.h \
    lib/geometry/Box.h \
    lib/geometry/Polygon.h \
    lib/geometry/Path.h \
    lib/geometry/PathAStar.h \
    lib/geometry/GridSearchNode.h \
    lib/geometry/VisibilityGraph.h \
    lib/geometry/DijkstraMap.h \
    lib/geometry/GeometricModel.h \
    lib/geometry/GridModel.h \
    lib/geometry/Collision.h
SOURCES += lib/geometry/Line.cpp \
    lib/geometry/Box.cpp \
    lib/geometry/Polygon.cpp \
    lib/geometry/Path.cpp \
    lib/geometry/PathAStar.cpp \
    lib/geometry/GridSearchNode.cpp \
    lib/geometry/VisibilityGraph.cpp \
    lib/geometry/DijkstraMap.cpp \
    lib/geometry/GeometricModel.cpp \
    lib/geometry/GridModel.cpp \
    lib/geometry/Collision.cpp
