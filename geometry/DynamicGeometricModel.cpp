#include "DynamicGeometricModel.h"
#include "globals.h"
#include "blackboard/Config.h"
#include "blackboard/State.h"
#include "blackboard/Command.h"
#include "util/ColorUtil.h"
#include "Collision.h"
#include "geometry/Box.h"

// The DynamicGeometricModel is a polygonal scene representation of the moving obstacles in
// the environment. Only obstacles of holonomic type are supported. Use the set() function to
// give the model the current state of the obstacles.
// With this object you can...
// - getObstacles() and such to access the obstacles in the scene.
// - predict() a future state.
// - collision check a geometric object or a trajectory with one of the *CollisionCheck()
//   methods against the current state of the scene.

DynamicGeometricModel::DynamicGeometricModel()
{

}

// Overwrites the id of all obstacles with the given one.
// This is only good for visualizing a sequence of predictions.
void DynamicGeometricModel::setId(int id)
{
    for (int i = 0; i < holonomicObstacles.size(); i++)
        holonomicObstacles[i].setId(id);
    for (int i = 0; i < unicycleObstacles.size(); i++)
        unicycleObstacles[i].setId(id);
}

// Sets (overwrites) the holonomic obstacles in the model.
void DynamicGeometricModel::setObstacles(const Vector<HolonomicObstacle> &o)
{
    holonomicObstacles = o;
}

// Sets (overwrites) the unicycle obstacles in the model.
void DynamicGeometricModel::setObstacles(const Vector<UnicycleObstacle> &o)
{
    unicycleObstacles = o;
}

// Clears the geometric model to a blank state.
void DynamicGeometricModel::clear()
{
    holonomicObstacles.clear();
    unicycleObstacles.clear();
}

// Returns true if there are no obstacles in the model.
bool DynamicGeometricModel::isEmpty() const
{
    return (holonomicObstacles.isEmpty() && unicycleObstacles.isEmpty());
}

// Adds a holonomic obstacle to the model.
void DynamicGeometricModel::addObstacle(const HolonomicObstacle &o)
{
    holonomicObstacles << o;
}

// Adds a holonomic obstacle to the model.
void DynamicGeometricModel::addObstacle(const UnicycleObstacle &o)
{
    unicycleObstacles << o;
}

// Grows (or shrinks) the obstacles by delta. Delta is the distance by how much the vertices
// are pushed outwards from the center along the half angle between the two neighbouring
// edges. If delta is negative, the vertices are pulled invards and the polygon is shrunk.
// The polygons can be in an untransformed state.
void DynamicGeometricModel::grow(double delta)
{
    for (int i = 0; i < holonomicObstacles.size(); i++)
        holonomicObstacles[i].grow(delta);
    for (int i = 0; i < unicycleObstacles.size(); i++)
        unicycleObstacles[i].grow(delta);
}

// Erases all polygons that overlap the point p.
void DynamicGeometricModel::eraseObstaclesAt(const Vec2 &p)
{
    for (int i = holonomicObstacles.size()-1; i >= 0; i--)
        if (((Polygon*)(&holonomicObstacles[i]))->intersects(p))
            holonomicObstacles.removeAt(i);
    for (int i = unicycleObstacles.size()-1; i >= 0; i--)
        if (((Polygon*)(&unicycleObstacles[i]))->intersects(p))
            unicycleObstacles.removeAt(i);
}

// Maps all polygons into the frame given by Pose.
// The polygons are assumed to be given in world coordinates and are transformed
// to the local coordinates of the frame given by Pose.
void DynamicGeometricModel::operator-=(const Pose2D &o)
{
    for (uint i = 0; i < holonomicObstacles.size(); i++)
        holonomicObstacles[i] -= o;
    for (uint i = 0; i < unicycleObstacles.size(); i++)
        unicycleObstacles[i] -= o;
}

// Maps the polygons to world coordinates from the frame given by Pose.
// The polygons are assumed to be in local coordinates in the frame given
// by Pose and are transformed to world coordinates.
void DynamicGeometricModel::operator+=(const Pose2D &o)
{
    for (uint i = 0; i < holonomicObstacles.size(); i++)
        holonomicObstacles[i] += o;
    for (uint i = 0; i < unicycleObstacles.size(); i++)
        unicycleObstacles[i] += o;
}

// Maps all polygons into the frame given by Pose.
// The polygons are assumed to be in world coordinates and is transformed
// to the local coordinates in the frame given by Pose.
DynamicGeometricModel DynamicGeometricModel::operator-(const Pose2D &o)
{
    DynamicGeometricModel copy = *this;
    copy -= o;
    return copy;
}

// Maps the polygons to world coordinates from the frame given by Pose.
// The polygons are assumed to be in local in the frame given by Pose and
// are transformed to world coordinates.
DynamicGeometricModel DynamicGeometricModel::operator+(const Pose2D &o)
{
    DynamicGeometricModel copy = *this;
    copy += o;
    return copy;
}

// Consumes the transformation of all obstacles and converts their corners to world coordinates.
void DynamicGeometricModel::transform()
{
    for (uint i = 0; i < holonomicObstacles.size(); i++)
        holonomicObstacles[i].transform();
    for (uint i = 0; i < unicycleObstacles.size(); i++)
        unicycleObstacles[i].transform();
}

// Computes the bounding box for all contained obstacles.
void DynamicGeometricModel::computeBoundingBoxes()
{
    for (uint i = 0; i < holonomicObstacles.size(); i++)
        holonomicObstacles[i].boundingBox();
    for (uint i = 0; i < unicycleObstacles.size(); i++)
        unicycleObstacles[i].boundingBox();
}

// Returns the Euklidean distance between the point v and the closest point in the scene.
double DynamicGeometricModel::distance(const Vec2 &v) const
{
    double minDist = 10000;
    for (uint i = 0; i < holonomicObstacles.size(); i++)
        minDist = min(minDist, holonomicObstacles[i].distance(v));
    for (uint i = 0; i < unicycleObstacles.size(); i++)
        minDist = min(minDist, unicycleObstacles[i].distance(v));
    return minDist;
}

// Returns the normal of the vertex or edge nearest to p.
Vec2 DynamicGeometricModel::closestNormal(const Vec2 &p) const
{
    double minDist = std::numeric_limits<double>::max();
    Vec2 closestNormal;
    for (int i = 0; i < holonomicObstacles.size(); i++)
    {
        double d = holonomicObstacles[i].distance(p);
        if (fabs(d) < fabs(minDist))
        {
            minDist = d;
            closestNormal = holonomicObstacles[i].closestNormal(p);
        }
    }
    for (int i = 0; i < unicycleObstacles.size(); i++)
    {
        double d = unicycleObstacles[i].distance(p);
        if (fabs(d) < fabs(minDist))
        {
            minDist = d;
            closestNormal = unicycleObstacles[i].closestNormal(p);
        }
    }
    return closestNormal;
}

// Returns the normal of the vertex or edge nearest to p.
Vec2 DynamicGeometricModel::closestPoint(const Vec2 &p) const
{
    double minDist = std::numeric_limits<double>::max();
    Vec2 closestPoint;
    for (int i = 0; i < holonomicObstacles.size(); i++)
    {
        Vec2 cp = holonomicObstacles[i].closestPoint(p);
        double d = (p-cp).norm();
        if (fabs(d) < fabs(minDist))
        {
            minDist = d;
            closestPoint = cp;
        }
    }
    for (int i = 0; i < unicycleObstacles.size(); i++)
    {
        Vec2 cp = unicycleObstacles[i].closestPoint(p);
        double d = (p-cp).norm();
        if (fabs(d) < fabs(minDist))
        {
            minDist = d;
            closestPoint = cp;
        }
    }
    return closestPoint;
}

// Computes a collisionless prediction of the future state of the geometric model at
// time dt. This means the obstacles are moved according to their motion models by
// the time dt, but no collisions are resolved.
void DynamicGeometricModel::predict(double dt)
{
    for (uint i = 0; i < holonomicObstacles.size(); i++)
        holonomicObstacles[i].predict(dt);
    for (uint i = 0; i < unicycleObstacles.size(); i++)
        unicycleObstacles[i].predict(dt);
}

// Computes a collisionless prediction of the future state of the geometric model at
// time dt. This means the obstacles are moved according to their motion models by
// the time dt, but no collisions are resolved.
DynamicGeometricModel DynamicGeometricModel::predicted(double dt) const
{
    DynamicGeometricModel wm = *this;
    wm.predict(dt);
    return wm;
}

// Checks if the point p intersects with one of obstacles in their current state.
// If there is a collision, it returns the index of the first found intersected
// obstacle. Otherwise -1 is returned. The getObstacle() method can be used with
// the returned index as argument to retrieve the collided obstacle.
int DynamicGeometricModel::staticPointCollisionCheck(const Vec2 &p) const
{
    for (uint i = 0; i < holonomicObstacles.size(); i++)
        if (((Obstacle*)&holonomicObstacles[i])->intersects(p))
            return i;
    for (uint i = 0; i < unicycleObstacles.size(); i++)
        if (((Obstacle*)&unicycleObstacles[i])->intersects(p))
            return holonomicObstacles.size()+i;
    return -1;
}

// Checks if the point p intersects with one of obstacles at time dt.
// If there is a collision, it returns the index of the first found intersected
// obstacle. Otherwise -1 is returned. The getObstacle() method can be used with
// the returned index as argument to retrieve the collided obstacle.
int DynamicGeometricModel::dynamicPointCollisionCheck(const Vec2 &p, double dt) const
{
    for (uint i = 0; i < holonomicObstacles.size(); i++)
        if (holonomicObstacles[i].dynamicPointIntersection(p, dt))
            return i;
    for (uint i = 0; i < unicycleObstacles.size(); i++)
        if (unicycleObstacles[i].dynamicPointIntersection(p, dt))
            return holonomicObstacles.size()+i;
    return -1;
}

// Checks if the polygon p intersects with one of obstacles at time dt.
// If there is a collision, it returns the index of the first found intersected
// obstacle. Otherwise -1 is returned. The getObstacle() method can be used with
// the returned index as argument to retrieve the collided obstacle.
int DynamicGeometricModel::dynamicPolygonCollisionCheck(const Polygon &p, double dt) const
{
    //qDebug() << "DynamicGeometricModel::dynamicPolygonCollisionCheck(p, dt) p:" << p << "dt:" << dt;
    for (uint i = 0; i < holonomicObstacles.size(); i++)
        if (holonomicObstacles[i].dynamicPolygonIntersection(p, dt))
            return i;
    for (uint i = 0; i < unicycleObstacles.size(); i++)
        if (unicycleObstacles[i].dynamicPolygonIntersection(p, dt))
            return holonomicObstacles.size()+i;
    return -1;
}

// Checks if the line l intersects with one of obstacles in their current state.
// It returns a pointer to the first (not necessarily the nearest) obstacle it finds
// that intersects the line.
// If there is a collision, it returns the index of the first found intersected
// obstacle. Otherwise -1 is returned. The getObstacle() method can be used with
// the returned index as argument to retrieve the collided obstacle.
int DynamicGeometricModel::staticLineCollisionCheck(const Line &l) const
{
    for (uint i = 0; i < holonomicObstacles.size(); i++)
        if (((Obstacle*)&holonomicObstacles[i])->intersects(l))
            return i;
    for (uint i = 0; i < unicycleObstacles.size(); i++)
        if (((Obstacle*)&unicycleObstacles[i])->intersects(l))
            return holonomicObstacles.size()+i;
    return -1;
}

// Checks if the Polygon p intersects with one of the obstacles in their current state.
// If there is a collision, it returns the index of the first found intersected
// obstacle. Otherwise -1 is returned. The getObstacle() method can be used with
// the returned index as argument to retrieve the collided obstacle.
int DynamicGeometricModel::staticPolygonCollisionCheck(const Polygon &p) const
{
    for (uint i = 0; i < holonomicObstacles.size(); i++)
        if (((Obstacle*)&holonomicObstacles[i])->intersects(p))
            return i;
    for (uint i = 0; i < unicycleObstacles.size(); i++)
        if (((Obstacle*)&unicycleObstacles[i])->intersects(p))
            return holonomicObstacles.size()+i;
    return -1;
}

// Returns the point at which a ray from "from" to "to" first intersects the geometric
// scene. If the ray does not intersect with anything, to is returned.
Vec2 DynamicGeometricModel::rayIntersection(const Vec2 &from, const Vec2 &to) const
{
    Vec2 best = to;
    double d = (to-from).length();
    for (uint i = 0; i < holonomicObstacles.size(); i++)
    {
        Vec2 ip = holonomicObstacles[i].rayIntersection(from, to);
        double dd = (ip-from).norm();
        if (dd < d)
        {
            best = ip;
            d = dd;
        }

    }
    for (uint i = 0; i < unicycleObstacles.size(); i++)
    {
        Vec2 ip = unicycleObstacles[i].rayIntersection(from, to);
        double dd = (ip-from).norm();
        if (dd < d)
        {
            best = ip;
            d = dd;
        }

    }
    return best;
}

// Checks if the holonomic trajectory described by the Hpm2D intersects with an obstacle in the future.
// This is a continuous and dynamic collision check that takes the motion of the obstacles into
// account. It returns a collision struct with the collision time dt and a pointer to the intersected
// obstacle. If no collision occurs, a default constructed Collision object is returned with dt=-1.
Collision DynamicGeometricModel::trajectoryCollisionCheck(const Hpm2D &kf) const
{
    Collision col;

    for (uint i = 0; i < holonomicObstacles.size(); i++)
    {
        double ct = holonomicObstacles[i].intersects(kf);
        if (ct >= 0 && (ct < col.dt || col.dt < 0))
        {
            col.dt = ct;
            col.obstacleId = holonomicObstacles[i].getId();
            col.obstacleIdx = i;
        }
    }
    for (uint i = 0; i < unicycleObstacles.size(); i++)
    {
        double ct = unicycleObstacles[i].intersects(kf);
        if (ct >= 0 && (ct < col.dt || col.dt < 0))
        {
            col.dt = ct;
            col.obstacleId = unicycleObstacles[i].getId();
            col.obstacleIdx = holonomicObstacles.size()+i;
        }
    }
    return col;
}

// Checks if the unicycle trajectory described by the Unicycle intersects with an obstacle in the future.
// This is a continuous and dynamic collision check that takes the motion of the obstacles into
// account. It returns a collision struct with the collision time dt and a pointer to the intersected
// obstacle. If no collision occurs, a default constructed Collision object is returned with dt=-1.
Collision DynamicGeometricModel::trajectoryCollisionCheck(const Unicycle &u) const
{
    Collision col;

    for (uint i = 0; i < holonomicObstacles.size(); i++)
    {
        double ct = holonomicObstacles[i].intersects(u);
        if (ct >= 0 && (ct < col.dt || col.dt < 0))
        {
            col.dt = ct;
            col.obstacleId = holonomicObstacles[i].getId();
            col.obstacleIdx = i;
        }
    }
    for (uint i = 0; i < unicycleObstacles.size(); i++)
    {
        HolonomicObstacle hol = unicycleObstacles[i];
        hol.transform();
        double ct = hol.intersects(u);
        if (ct >= 0 && (ct < col.dt || col.dt < 0))
        {
            col.dt = ct;
            col.obstacleId = unicycleObstacles[i].getId();
            col.obstacleIdx = holonomicObstacles.size()+i;
        }
    }
    return col;
}

// Returns the obstacle with the idx obstIdx.
Obstacle DynamicGeometricModel::getObstacle(int obstIdx) const
{
    if (obstIdx < holonomicObstacles.size())
        return holonomicObstacles[obstIdx];
    return unicycleObstacles[obstIdx-holonomicObstacles.size()];
}

// Returns a list of pointers to all obstacles in the world model.
Vector<Obstacle> DynamicGeometricModel::getObstacles() const
{
    Vector<Obstacle> obst;
    for (int i = 0; i < holonomicObstacles.size(); i++)
        obst << holonomicObstacles[i];
    for (int i = 0; i < unicycleObstacles.size(); i++)
        obst << unicycleObstacles[i];
    return obst;
}

// Draws the geometric model on a QPainter.
void DynamicGeometricModel::draw(QPainter *painter, const QPen &pen, const QBrush &brush, double opacity) const
{
    painter->save();

    QFont font;
    font.setFamily("Arial");
    font.setPointSize(1);
    painter->setFont(font);

    // The holonomic obstacles.
    painter->setPen(pen);
    for (uint i = 0; i < holonomicObstacles.size(); i++)
    {
        painter->setOpacity(opacity);
        painter->setBrush(brush);
        holonomicObstacles[i].draw(painter);

        painter->setOpacity(1.0);
        painter->setBrush(Qt::NoBrush);
        holonomicObstacles[i].draw(painter);

        // The obstacle id label.
        if (config.debugLevel > 0)
        {
            painter->save();
            painter->translate(holonomicObstacles[i].centroid());
            painter->rotate((holonomicObstacles[i].vel().angle()+PI2)*RAD_TO_DEG);
            painter->scale(-0.3, 0.3);
            painter->setOpacity(0.5);
            painter->drawText(QPointF(), QString::number(holonomicObstacles[i].getId()));
            painter->restore();
        }
    }

    // The unicycle obstacles.
    painter->setPen(pen);
    for (uint i = 0; i < unicycleObstacles.size(); i++)
    {
        painter->setOpacity(opacity);
        painter->setBrush(brush);
        unicycleObstacles[i].draw(painter);

        painter->setOpacity(1.0);
        painter->setBrush(Qt::NoBrush);
        unicycleObstacles[i].draw(painter);

        // The obstacle id label.
        if (config.debugLevel > 0)
        {
            painter->save();
            painter->translate(unicycleObstacles[i].centroid());
            painter->rotate((unicycleObstacles[i].rotation()+PI2)*RAD_TO_DEG);
            painter->scale(-0.3, 0.3);
            painter->setOpacity(0.5);
            painter->drawText(QPointF(), QString::number(unicycleObstacles[i].getId()));
            painter->restore();
        }
    }

    painter->restore();
}

void DynamicGeometricModel::draw(const QPen &pen, const QBrush &brush, const double opacity) const
{
    for (uint i = 0; i < holonomicObstacles.size(); i++)
        holonomicObstacles[i].draw(brush.color());
    for (uint i = 0; i < unicycleObstacles.size(); i++)
        unicycleObstacles[i].draw(brush.color());
}

// Draws the bounding boxes of all contained objects.
void DynamicGeometricModel::drawBoundingBoxes(QPainter *painter) const
{
    for (uint i = 0; i < holonomicObstacles.size(); i++)
        holonomicObstacles[i].boundingBox().draw(painter);
    for (uint i = 0; i < unicycleObstacles.size(); i++)
        unicycleObstacles[i].boundingBox().draw(painter);
}

// Draws the swept volumes of all contained objects.
void DynamicGeometricModel::drawSweptVolumes(QPainter *painter, double dt) const
{
    painter->save();
    painter->setPen(colorUtil.penThin);
    for (uint i = 0; i < holonomicObstacles.size(); i++)
        holonomicObstacles[i].sweptVolume(dt).draw(painter);
//    for (int i = 0; i < unicycleObstacles.size(); i++)
//        unicycleObstacles[i].sweptVolume(dt).draw(painter);
    painter->restore();
}

QDebug operator<<(QDebug dbg, const DynamicGeometricModel &w)
{
    Vector<Obstacle> obst = w.getObstacles();
    for (int i = 0; i < obst.size(); i++)
        dbg << "   " << obst[i] << obst[i].centroid() << "\n";
    return dbg;
}
