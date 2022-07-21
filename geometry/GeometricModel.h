#ifndef GEOMETRICMODEL_H_
#define GEOMETRICMODEL_H_
#include "GridModel.h"
#include "util/Vector.h"
#include "util/Vec2.h"
#include "geometry/Box.h"
#include "Collision.h"
#include <QPainter>
#include <QDebug>
#include "agents/HolonomicObstacle.h"
#include "VisibilityGraph.h"

class GeometricModel
{
public:
    Vector<Obstacle> obstacles; // A Vector of all Obstacles in the model.
    Vector<VecN<4> > hierarchy; // Polygon hierarchy information.
    Vector<Vec2> path; // Last computed shortest path.
    VisibilityGraph visibilityGraph;
    mutable Obstacle tempObstacle;
    uint debug;


    GeometricModel();
    ~GeometricModel(){}

    // Model construction and query methods.
    void setDebug(uint d);
    void clear();
    void reset();
    bool isEmpty() const;
    void setBounds();
    const Box& getBounds() const;
    Box getBoundingBox() const;
    void setObstacles(const Vector<Obstacle>& obst);
    void addObstacles(const Vector<Obstacle>& obst);
    void addObstacles(const Vector<HolonomicObstacle>& obst);
    void addObstacle(const Obstacle& o);
    void setFromGrid(const GridModel& grid);
    Obstacle &getObstacle(int obstIdx);
    const Obstacle& getObstacle(int obstIdx) const;
    const Vector<Obstacle>& getObstacles() const;
    uint size() const;
    int getVertexCount() const;

    void prune(Vec2 p);

    void operator-=(const Pose2D &o);
    void operator+=(const Pose2D &o);
    GeometricModel operator-(const Pose2D &o);
    GeometricModel operator+(const Pose2D &o);

    void reverseOrder();
    void grow(double delta);
    void scale(double alpha);
    void scale(double alpha, double beta);
    void scale(const Vec2& s);
    void rotate(double a);
    void translate(double dx, double dy);
    void translate(const Vec2& v);
    void transform();

    double getSignedDistance(const Vec2& p) const;
    Vec2 moveOutOfObstacles(const Vec2& p) const;
    Vec2 moveIntoFreeSpace(const Vec2& p) const;
    void eraseObstaclesAt(const Vec2 &p);
    Vec2 closestNormal(const Vec2& p) const;
    Vec2 closestPoint(const Vec2& p) const;
    const Vector<Vec2> &computePath(const Vec2& from, const Vec2& to);
    const Vector<Vec2> &getLastPath();
    Pose2D pathBoxIntersection(const Box& box) const;

    void clip(const Box& box);

    // Collision detection methods.
    int pointCollisionCheck(const Vec2 &p) const;
    int lineCollisionCheck(const Line &l) const;
    Vec2 rayIntersection(const Vec2& from, const Vec2& to) const;
    int polygonCollisionCheck(const Polygon &p) const;
    int convexPolygonCollisionCheck(const Polygon &p) const;
    Collision trajectoryCollisionCheck(const Hpm2D &kf) const;
    Collision trajectoryCollisionCheck(const Unicycle &u) const;

    void buildFlatHierarchy();
    void rewriteIds();

    // Drawing methods.
    void draw(const QPen &pen, const QBrush &brush, double opacity=0.5) const;
    void draw(QPainter* painter, const QPen &pen, const QBrush &brush, double opacity=0.5) const;
    void drawBoundingBoxes(QPainter* painter) const;
    void drawVisibilityGraph(QPainter* painter) const;

    void streamOut(QDataStream &out) const;
    void streamIn(QDataStream &in);
};

QDebug operator<<(QDebug dbg, const GeometricModel &w);
QDataStream& operator<<(QDataStream& out, const GeometricModel &o);
QDataStream& operator>>(QDataStream& in, GeometricModel &o);

#endif
