#ifndef GRIDMODEL_H_
#define GRIDMODEL_H_
#include <QDebug>
#include "lib/util/Vector.h"
#include "lib/util/Vec2.h"
#include "lib/util/Vec2u.h"
#include "lib/util/Grid.h"
#include "lib/geometry/Box.h"
#include "lib/geometry/Polygon.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "DijkstraMap.h"

class GridModel : public Grid
{
    cv::Mat M; // The grid is implemented as an openCV matrix.
    DijkstraMap dij; // A Dijkstra map for path searches.

public:

    GridModel();
    ~GridModel(){}
    GridModel(const GridModel &o);  // copy constructor
    GridModel& operator=(const GridModel &o);
    GridModel(const cv::Mat &o);
    GridModel& operator=(const cv::Mat &o);

    void init();
    void clear();

    void setMat(const cv::Mat &o);
    const cv::Mat &getMat() const;

    GridModel operator+(const GridModel &o);
    void operator+=(const GridModel &o);
    void operator-=(const GridModel &o);

    void max(const GridModel &o);


    uint getWidth() const;
    uint getHeight() const;
    Box boundingBox() const;

    void blockBorder();
    void dilate(double radius);
    void erode(double radius);
    void blur(double radius);
    void canny();
    void invert();

    double getAt(uint n) const;
    double getAt(uint i, uint j) const;
    double getAt(Vec2 x) const;
    double getAt(Vec2u idx) const;
    uchar getCountAt(Vec2u idx) const;
    double getAt(const Polygon &p) const;
    void setAt(uint n, double v);
    void setAt(uint i, uint j, double v);
    void setAt(Vec2 x, double v);
    void setAt(Vec2u idx, double v);

    void computeOccupancyGrid(const Vector<Polygon> &polygons, int thickness = 1);
    void computeOccupancyGrid(const LinkedList<Polygon> &polygons, int thickness=1);
    void computeOccupancyGrid(const Polygon &polygon, int thickness = 1);
    void computeOccupancyGrid(const Vector<Vec3> &pointCloud, double floorHeight=0.0, double ceilingHeight=10.0);
    void computeOccupancyGrid(const Vector<Vec2> &pointCloud);
    LinkedList<Polygon> extractPolygons() const;

    void clearPolygon(const Polygon &polygon);

    bool isOccupied(uint i, uint j) const;
    bool isOccupied(const Vec2& x) const;
    bool isOccupied(const Vec2u& idx) const;
    bool isOccupied(uint n) const;
    Vec2 getRandomFreePosition() const;

    bool hasLineOfSight(const Vec2u& cellIdxA, const Vec2u& cellIdxB) const;
    bool polygonCollisionCheck(const Polygon &p) const;

    Vector<Vec2> computePath(const Vec2 &from, const Vec2 &to) const;
    Vector<Vec2> computeDijkstraPath(const Vec2 &from, const Vec2 &to) const;
    void initDijkstraMap(const Vec2& to);

    void draw(QPainter* painter, const QBrush& brush, double opacity = 0.8, double min = 0, double max = 1) const;
    void draw(const QBrush &brush = QColor("#FF0000"), double opacity = 0.8, double min = 0, double max = 1) const;
    void drawCounts(const QBrush &brush = QColor("#FF0000"), double opacity = 0.8, double min = 0, double max = 1) const;
    void drawOccupancy(const QColor &color = QColor("#FF0000"), double opacity = 0.8) const;
    void drawBorder(QPainter *painter) const;
    void drawBorder() const;
    void drawDijkstraMap(QPainter *painter) const;
    
    void printOutputs() const;
};

QDebug operator<<(QDebug dbg, const GridModel &w);
QDataStream& operator<<(QDataStream& out, const GridModel &o);
QDataStream& operator>>(QDataStream& in, GridModel &o);

#endif
