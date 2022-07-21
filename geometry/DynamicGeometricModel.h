#ifndef DYNAMICGEOMETRICMODEL_H_
#define DYNAMICGEOMETRICMODEL_H_
#include "agents/HolonomicObstacle.h"
#include "agents/UnicycleObstacle.h"
#include "util/Vector.h"
#include "util/Vec3.h"
#include "Collision.h"
#include <QPainter>
#include <QDebug>

class DynamicGeometricModel
{
    Vector<HolonomicObstacle> holonomicObstacles; // Obstacles that move like the holonomic model.
    Vector<UnicycleObstacle> unicycleObstacles; // Obstacles that move like the unicycle model.

public:

    DynamicGeometricModel();
    ~DynamicGeometricModel(){}

    void clear();
    bool isEmpty() const;

    void setId(int id);

    void setObstacles(const Vector<HolonomicObstacle>& o);
    void setObstacles(const Vector<UnicycleObstacle>& o);
    void addObstacle(const HolonomicObstacle& o);
    void addObstacle(const UnicycleObstacle& o);
    Obstacle getObstacle(int obstIdx) const;
    Vector<Obstacle> getObstacles() const;
    void eraseObstaclesAt(const Vec2 &p);

    void operator-=(const Pose2D &o);
    void operator+=(const Pose2D &o);
    DynamicGeometricModel operator-(const Pose2D &o);
    DynamicGeometricModel operator+(const Pose2D &o);

    void grow(double delta);
    void transform();
    void computeBoundingBoxes();

    double distance(const Vec2& v) const;
    Vec2 closestNormal(const Vec2& p) const;
    Vec2 closestPoint(const Vec2& p) const;

    // Prediction methods without collision check.
    void predict(double dt);
    DynamicGeometricModel predicted(double dt) const;

    // Collision detection methods.
    Vec2 rayIntersection(const Vec2 &from, const Vec2 &to) const;
    int staticPointCollisionCheck(const Vec2 &p) const;
    int staticLineCollisionCheck(const Line &l) const;
    int staticPolygonCollisionCheck(const Polygon &p) const;
    int dynamicPointCollisionCheck(const Vec2 &p, double dt) const;
    int dynamicPolygonCollisionCheck(const Polygon &p, double dt) const;
    Collision trajectoryCollisionCheck(const Hpm2D &kf) const;
    Collision trajectoryCollisionCheck(const Unicycle &u) const;

    void draw(QPainter* painter, const QPen &pen, const QBrush &brush, double opacity=0.5) const;
    void draw(const QPen &pen, const QBrush &brush, double opacity=0.5) const;
    void drawBoundingBoxes(QPainter* painter) const;
    void drawSweptVolumes(QPainter *painter, double dt) const;
};

QDebug operator<<(QDebug dbg, const DynamicGeometricModel &w);

#endif
