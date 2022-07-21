#ifndef GRIDMODEL_H_
#define GRIDMODEL_H_
#include <QDebug>
#include "util/Vector.h"
#include "util/Vec2.h"
#include "util/Vec2u.h"
#include "util/Grid.h"
#include "geometry/Box.h"
#include "agents/Obstacle.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "DijkstraMap.h"

class GridModel : public Grid
{
    cv::Mat M; // The grid is implemented as an openCV matrix.
    Vector<Vec2> path;
    DijkstraMap dij;

public:

    GridModel();
    ~GridModel(){}
    GridModel(const GridModel &o);  // copy constructor
    GridModel& operator=(const GridModel &o);

    GridModel(const cv::Mat &o);
    GridModel& operator=(const cv::Mat &o);

    void operator+=(const GridModel &o);

    void init();
    void clear();

    uint getWidth() const;
    uint getHeight() const;
    Box boundingBox() const;

    void blockBorder();
    void dilate(double radius);
    void erode(double radius);
    void blur(double radius);
    void canny();

    double getAt(uint n) const;
    double getAt(uint i, uint j) const;
    double getAt(Vec2 x) const;
    double getAt(Vec2u idx) const;
    double getAt(const Polygon &p) const;
    void setAt(uint n, double v);
    void setAt(uint i, uint j, double v);
    void setAt(Vec2 x, double v);
    void setAt(Vec2u idx, double v);

    void computeOccupancyGrid(const Vector<Obstacle> &obst);
    void computeOccupancyGrid(const Vector<Vec3> &pointCloud, double floorHeight=0.0, double ceilingHeight=10.0);
    Vector<Obstacle> extractPolygons(Vector<VecN<4> > &hierarchy) const;

    bool isOccupied(uint i, uint j) const;
    bool isOccupied(const Vec2& x) const;
    bool isOccupied(const Vec2u& idx) const;
    bool isOccupied(uint n) const;
    Vec2 getRandomFreePosition() const;

    bool hasLineOfSight(const Vec2u& cellIdxA, const Vec2u& cellIdxB) const;
    bool polygonCollisionCheck(const Polygon &p) const;

    const Vector<Vec2> &computePath(const Vec2 &from, const Vec2 &to);
    const Vector<Vec2> &computeDijkstraPath(const Vec2 &from, const Vec2 &to);
    void initDijkstraMap(const Vec2& to);
    const Vector<Vec2> &getLastPath();

    void draw(QPainter* painter, const QBrush& brush, double opacity = 0.8, double min = 0, double max = 1) const;
    void draw(const QBrush &brush = QColor("#FF0000"), double opacity = 0.8, double min = 0, double max = 1) const;
    void drawBorder(QPainter *painter) const;
    void drawBorder() const;
    void drawDijkstraMap(QPainter *painter) const;
    
    void printOutputs() const;
};

QDebug operator<<(QDebug dbg, const GridModel &w);
QDataStream& operator<<(QDataStream& out, const GridModel &o);
QDataStream& operator>>(QDataStream& in, GridModel &o);

#endif
