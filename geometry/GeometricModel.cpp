#include "GeometricModel.h"
#include "globals.h"
#include "blackboard/Config.h"
#include "blackboard/State.h"
#include "util/ColorUtil.h"
#include "KeyframePlayer/BangBang2D.h"
#include "clipper.hpp"

// The GeometricModel is a polygonal representation of obstacles. It's a geometric map.
// The GeometricModel offers an interface to add and retrieve the Obstacles, to grow or
// transform them, and most importantly, to compute collision tests for points, lines,
// polygons, and motion trajectories (unicycle and hpm2d).
// Internally, the polygons are organized in a hierarchical structure, which is needed
// in order to be able to represent an all-surrounding free-space polygon. By default,
// only a flat hierarchy is maintained where all polygons are blocked space polygons.
// When the

GeometricModel::GeometricModel()
{
    debug = 0;
}

// Clears the geometric model to a blank state.
void GeometricModel::clear()
{
    obstacles.clear();
    hierarchy.clear();
    visibilityGraph.reset();
}

// Resets the path search of the geometric model.
void GeometricModel::reset()
{
    visibilityGraph.reset();
}

bool GeometricModel::isEmpty() const
{
    return obstacles.isEmpty();
}

// Sets (overwrites) the obstacles with the given ones.
void GeometricModel::setObstacles(const Vector<Obstacle> &obst)
{
    obstacles = obst;
    buildFlatHierarchy();
}

// Adds (appends) the obstacles to the geometric model.
void GeometricModel::addObstacles(const Vector<Obstacle> &obst)
{
    obstacles << obst;
    buildFlatHierarchy();
}

// Adds (appends) the obstacles to the geometric model.
void GeometricModel::addObstacles(const Vector<HolonomicObstacle> &obst)
{
    for (uint i = 0; i < obst.size(); i++)
        obstacles << obst[i];
    buildFlatHierarchy();
}

// Adds a static obstacle to the scene.
void GeometricModel::addObstacle(const Obstacle &o)
{
    obstacles << o;
    buildFlatHierarchy();
}

// Sets (overwrites) the state of the geometric model using an occupancy map (grid).
// The grid is segmented and the contours of the segments are simplified to polygons
// using the Douglas Peucker algorithm. The resulting polygons are non-convex disjunct
// polygons in transformed state which then become static obstacles. This function also
// computes the hierarchy structure that describes how the polygons are nested in each
// other. Sometimes there is a root free space polygon that contains all other polygons.
// Use the prune(p) function to discard unnecessary polygons.
void GeometricModel::setFromGrid(const GridModel &grid)
{
    clear();
    obstacles = grid.extractPolygons(hierarchy); // In this case the hierarchy is built by contour detection.
}

// Grows (or shrinks) the obstacles by delta. Delta is the distance by how much the vertices
// are pushed outwards from the center along the half angle between the two neighbouring
// edges. If delta is negative, the vertexes are pulled invards and the polygon is shrunk.
// The polygons can be in an untransformed state. Note that growing polygons this way can
// have nasty side effects like self-intersections.
void GeometricModel::grow(double delta)
{
    for (int i = 0; i < obstacles.size(); i++)
        obstacles[i].grow(delta);
}

// Scales the whole model by alpha. Multiplies all coordinates with alpha.
void GeometricModel::scale(double alpha)
{
    for (int i = 0; i < obstacles.size(); i++)
        obstacles[i].scale(alpha, alpha);
}

// Scales the whole model by alpha and beta. Multiplies the x coordinates
// with alpha and the y coordinates with beta.
void GeometricModel::scale(double alpha, double beta)
{
    for (int i = 0; i < obstacles.size(); i++)
        obstacles[i].scale(alpha, beta);
}

// Scales the whole model by s.x and s.y. Multiplies the x coordinates
// with s.x and the y coordinates with s.y.
void GeometricModel::scale(const Vec2 &s)
{
    scale(s.x, s.y);
}

// Rotates the polygons by angle a in rad.
void GeometricModel::rotate(double a)
{
    for (int i = 0; i < obstacles.size(); i++)
        obstacles[i].rotate(a);
}

// Translates the polygons by (dx,dy).
void GeometricModel::translate(double dx, double dy)
{
    for (int i = 0; i < obstacles.size(); i++)
        obstacles[i].translate(dx, dy);
}

// Translates the polygons by (v.x, v.y).
void GeometricModel::translate(const Vec2 &v)
{
    for (int i = 0; i < obstacles.size(); i++)
        obstacles[i].translate(v.x, v.y);
}

// Maps all polygons into the frame given by Pose.
// The polygons are assumed to be in world coordinates and is transformed
// to the local coordinates in the frame given by Pose.
void GeometricModel::operator-=(const Pose2D &o)
{
    for (uint i = 0; i < obstacles.size(); i++)
        obstacles[i] -= o;
}

// Maps the polygons to world coordinates from the frame given by Pose.
// The polygons are assumed to be in local in the frame given by Pose and
// are transformed to world coordinates.
void GeometricModel::operator+=(const Pose2D &o)
{
    for (uint i = 0; i < obstacles.size(); i++)
        obstacles[i] += o;
}

// Maps all polygons into the frame given by Pose.
// The polygons are assumed to be in world coordinates and is transformed
// to the local coordinates in the frame given by Pose.
GeometricModel GeometricModel::operator-(const Pose2D &o)
{
    GeometricModel copy = *this;
    copy -= o;
    return copy;
}

// Maps the polygons to world coordinates from the frame given by Pose.
// The polygons are assumed to be in local in the frame given by Pose and
// are transformed to world coordinates.
GeometricModel GeometricModel::operator+(const Pose2D &o)
{
    GeometricModel copy = *this;
    copy += o;
    return copy;
}

// Reverses the vertex order of all contained polygons.
// This can be used to restore a CCW order.
void GeometricModel::reverseOrder()
{
    for (int i = 0; i < obstacles.size(); i++)
    {
        if (obstacles[i].type != Obstacle::FreeSpace)
            obstacles[i].reverseOrder();
    }
}

// Consumes the transformation of all obstacles and converts their corners to world coordinates.
void GeometricModel::transform()
{
    for (int i = 0; i < obstacles.size(); i++)
        obstacles[i].transform();
}

// Sets the bounds for the shortest path search.
void GeometricModel::setBounds()
{
    visibilityGraph.setBounds(config.gridWidth/2,
                              -config.gridHeight/2+config.gridOffset,
                              -config.gridWidth/2,
                              config.gridHeight/2+config.gridOffset);
}

// Returns the bounding box that contains all polygons in the model.
const Box &GeometricModel::getBounds() const
{
    return visibilityGraph.getBounds();
}

// Returns the shortest signed distance between the point p and any polygon
// edge or corner in the scene. The magnitude of the distance is the Euklidean
// distance to the closest edge or corner and the sign of the distance indicates
// whether the point is inside or outside of the polygons.
double GeometricModel::getSignedDistance(const Vec2 &p) const
{
    double minDist = std::numeric_limits<double>::max();
    for (int i = 0; i < obstacles.size(); i++)
    {
        double d = obstacles[i].distance(p);
        if (fabs(d) < fabs(minDist))
            minDist = d;
    }
    return minDist;
}

// If the point p is inside one of the obstacles, it is replaced by
// the closest point (a little bit outside) on the boundary of the obstacle.
Vec2 GeometricModel::moveOutOfObstacles(const Vec2 &p) const
{
    int counter = 0;
    Vec2 v = p;
    int obstIdx = pointCollisionCheck(v);
    while (obstIdx >= 0 && counter < 10)
    {
        tempObstacle = getObstacle(obstIdx);
        tempObstacle.grow(0.01);
        v = tempObstacle.closestPoint(v);
        obstIdx = pointCollisionCheck(v);
        counter++;
    }
    if (debug > 0 && v != p)
        qDebug() << "moveOutOfObstacles():" << p  << "moved to" << v << counter;
    if (debug > 0 && counter >= 10)
        qDebug() << "moveOutOfObstacles() failed to move out of obstacles.";

    return v;
}

// If there is free space root polygon and point p is outside of it, the point is
// moved into the root polygon a little further than the closest point.
Vec2 GeometricModel::moveIntoFreeSpace(const Vec2 &p) const
{
    Vec2 v = p;
    if (!obstacles.isEmpty() && obstacles[0].type == Obstacle::FreeSpace && !obstacles[0].intersects(p))
    {
        tempObstacle = obstacles[0];
        tempObstacle.grow(0.01);
        v = tempObstacle.closestPoint(p);
    }

    if (debug > 0 && v != p)
        qDebug() << "moveIntoFreeSpace(const Vec2 &p):" << p << "moved to:" << v;

    return v;
}

// Erases all polygons that overlap the point p.
void GeometricModel::eraseObstaclesAt(const Vec2 &p)
{
    for (uint i = obstacles.size()-1; i >= 0; i--)
        if (obstacles[i].intersects(p))
            obstacles.removeAt(i);
    buildFlatHierarchy();
}

// Returns the normal of the vertex or edge nearest to p.
Vec2 GeometricModel::closestNormal(const Vec2& p) const
{
    double minDist = std::numeric_limits<double>::max();
    Vec2 closestNormal;
    for (uint i = 0; i < obstacles.size(); i++)
    {
        double d = obstacles[i].distance(p);
        if (fabs(d) < fabs(minDist))
        {
            minDist = d;
            closestNormal = obstacles[i].closestNormal(p);
        }
    }
    return closestNormal;
}

// Returns the point in the geometric scene that is nearest to p.
// This can be a polygon corner or a point on an edge.
Vec2 GeometricModel::closestPoint(const Vec2& p) const
{
    double minDist = std::numeric_limits<double>::max();
    Vec2 closestPoint;
    for (int i = 0; i < obstacles.size(); i++)
    {
        Vec2 cp = obstacles[i].closestPoint(p);
        double d = (p-cp).norm();
        if (fabs(d) < fabs(minDist))
        {
            minDist = d;
            closestPoint = cp;
        }
    }
    return closestPoint;
}

// Searches the polygonal scene for the shortest path from "from" to "to".
// In order to guarantee a path even in situations where "from" or "to" are
// occluded, both from and to are moved out of obstacles (and into free space) to
// the closest point. If there really is no path because the way is blocked, an
// empty path is returned.
const Vector<Vec2> &GeometricModel::computePath(const Vec2 &from, const Vec2 &to)
{
    Vec2 movedStart = moveIntoFreeSpace(from);
    movedStart = moveOutOfObstacles(movedStart);

    if (debug > 0 && movedStart != from)
    {
        qDebug() << "getPathMoveOut(const Vec2 &from, const Vec2 &to):" << from << to << "start moved to:" << movedStart;
        qDebug() << "point collision check:" << pointCollisionCheck(movedStart);
        qDebug() << "line Collision check:" << lineCollisionCheck(Line(from, to));
    }

    // Use the visibility graph to compute the path.
    visibilityGraph.setGeometricModel(*this);
    visibilityGraph.setTarget(to);
    visibilityGraph.setDebug(debug);
    path.clear();
    if (visibilityGraph.minimalConstruct(movedStart))
    {
        const Vector<Node>& ppath = visibilityGraph.getPath();
        for (uint i = 0; i < ppath.size(); i++)
            path << ppath[i];
    }

    return path;
}

// Determine the intersection of the last computed path with the given box.
// The intersection is the first intersection point along the path between the
// path and the box, or the last point of the path if no such intersection
// occurs. The orientation of the path in this point is set as the heading of
// the returned Pose.
Pose2D GeometricModel::pathBoxIntersection(const Box &box) const
{
    Pose2D intersectionPoint;
    const Vector<Node>& staticWorldPath = visibilityGraph.getPath();

    bool targetFound = false;
    uint i = 1;
    while (!targetFound && i < staticWorldPath.size())
    {
        // If the current path node is inside the box...
        if (box.intersects(staticWorldPath[i]))
        {
            intersectionPoint.setPos(staticWorldPath[i]);
            intersectionPoint.setHeading((staticWorldPath[i]-staticWorldPath[i-1]).angle());
        }

        // Otherwise intersect the boundary lines.
        else
        {
            targetFound = true;

            Line l(staticWorldPath[i], staticWorldPath[i-1]);
            intersectionPoint.setPos(box.intersection(l)); // We can be sure there is only one intersection.
            intersectionPoint.setHeading((staticWorldPath[i]-staticWorldPath[i-1]).angle());

            // Now it can happen so that the intersection of the path with the box
            // lies on the edge of a polygon. Let's move the target away by a little
            // to make sure that the target passes tangential and line intersection
            // tests.
            Vec2 nit = intersectionPoint.pos();
            if (staticWorldPath[i].isInLineWith(staticWorldPath[i-1]))
            {
                if (staticWorldPath[i].isRightOf(staticWorldPath[i-1]))
                    nit -= (staticWorldPath[i]-staticWorldPath[i-1]).normal().normalized(0.01);
                else if (staticWorldPath[i].isLeftOf(staticWorldPath[i-1]))
                    nit += (staticWorldPath[i]-staticWorldPath[i-1]).normal().normalized(0.01);
            }

//            if (nit != intermediateTarget.pos())
//                qDebug() << state.frameId << "intermediate target moved from" << intermediateTarget << "to" << nit;
            intersectionPoint.setPos(nit);
        }

        i++;
    }

    return intersectionPoint;
}

// Removes everything from the scene that's not inside the box.
void GeometricModel::clip(const Box &box)
{
    Polygon boxPolygon;
    boxPolygon << box.topLeft() << box.bottomLeft() << box.bottomRight() << box.topRight();

    double clipperFactor = 1000;
    ClipperLib::Clipper clpr;

    ClipperLib::Path clp;
    ListIterator<Vec2> cpit = boxPolygon.vertexIterator();
    while (cpit.hasNext())
    {
        const Vec2& v = cpit.next();
        clp << ClipperLib::IntPoint(v.x*clipperFactor, v.y*clipperFactor);
    }
    clpr.AddPath(clp, ClipperLib::ptClip, true);

    for (uint i = 0; i < obstacles.size(); i++)
    {
        if (obstacles[i].type == Obstacle::FreeSpace)
            continue;

        ClipperLib::Path sub;
        ListIterator<Vec2> vit = obstacles[i].vertexIterator();
        while (vit.hasNext())
        {
            const Vec2& v = vit.next();
            sub << ClipperLib::IntPoint(v.x*clipperFactor, v.y*clipperFactor);
        }
        clpr.AddPath(sub, ClipperLib::ptSubject, true);
    }

    ClipperLib::Paths solution;
    clpr.Execute(ClipperLib::ctIntersection, solution, ClipperLib::pftEvenOdd, ClipperLib::pftEvenOdd);

    Vector<Obstacle> result;
    for (uint i = 0; i < solution.size(); i++)
    {
        Obstacle p;
        for (uint j = 0; j < solution[i].size(); j++)
            p.addVertex((double)solution[i][j].X/clipperFactor, (double)solution[i][j].Y/clipperFactor);
        result << p;
    }

    if (!obstacles.isEmpty() && obstacles[0].type == Obstacle::FreeSpace)
    {

        clpr.Clear();
        clpr.AddPath(clp, ClipperLib::ptSubject, true);

        ClipperLib::Path sub;
        ListIterator<Vec2> vit = obstacles[0].vertexIterator();
        while (vit.hasNext())
        {
            const Vec2& v = vit.next();
            sub << ClipperLib::IntPoint(v.x*clipperFactor, v.y*clipperFactor);
        }
        clpr.AddPath(sub, ClipperLib::ptClip, true);

        ClipperLib::Paths solution;
        clpr.Execute(ClipperLib::ctDifference, solution, ClipperLib::pftEvenOdd, ClipperLib::pftEvenOdd);
        for (uint i = 0; i < solution.size(); i++)
        {
            Obstacle p;
            for (uint j = 0; j < solution[i].size(); j++)
                p.addVertex((double)solution[i][j].X/clipperFactor, (double)solution[i][j].Y/clipperFactor);
            result << p;
        }
    }

    obstacles = result;
}

// Returns the last computed path.
const Vector<Vec2> &GeometricModel::getLastPath()
{
    return path;
}

// Checks if the point p intersects with an obstacle.
// If there is a collision, it returns the index of the first found intersected
// obstacle. Otherwise -1 is returned. The getObstacle() method can be used with
// the returned index as argument to retrieve the collided obstacle. The root
// polygon, if there is one, is excluded from this check.
int GeometricModel::pointCollisionCheck(const Vec2 &p) const
{
    for (int i = 0; i < obstacles.size(); i++)
        if (obstacles[i].type == Obstacle::BlockedSpace && obstacles[i].intersects(p))
            return i;
    return -1;
}

// Checks if the line l intersects with an obstacle in the geometric model.
// If there is a collision, it returns the index of the first found intersected
// obstacle. Otherwise -1 is returned. The getObstacle() method can be used with
// the returned index as argument to retrieve the collided obstacle.
int GeometricModel::lineCollisionCheck(const Line &l) const
{
    for (int i = 0; i < obstacles.size(); i++)
        if (obstacles[i].intersects(l))
            return i;
    return -1;
}

// Returns the point at which a ray from from to to first intersects the geometric
// scene. If the ray does not intersect with anything, to is returned.
Vec2 GeometricModel::rayIntersection(const Vec2 &from, const Vec2 &to) const
{
    Vec2 best = to;
    Line l(from, to);
    double d = l.length();
    for (int i = 0; i < obstacles.size(); i++)
    {
        Vec2 ip = obstacles[i].rayIntersection(from, to);
        double dd = (ip-from).norm();
        if (dd < d)
        {
            best = ip;
            d = dd;
        }
    }
    return best;
}

// Checks if the Polygon p intersects with an obstacle.
// It returns the first found intersected obstacle idx (the index, not the id)
// if there is a collision. Otherwise -1 is returned. The Polygon
// and the obstacles must be in a transformed state for this to work.
int GeometricModel::polygonCollisionCheck(const Polygon &p) const
{
    for (int i = 0; i < obstacles.size(); i++)
        if (obstacles[i].intersects(p))
            return i;
    return -1;
}

// Checks if the Polygon p intersects with any convex obstacle. Nonconvex
// obstacles are ignored. It returns the first found intersected obstacle
// idx (the index, not the id) if there is a collision. Otherwise -1 is
// returned. The Polygon and the obstacles must be in a transformed state
// for this to work.
int GeometricModel::convexPolygonCollisionCheck(const Polygon &p) const
{
    for (int i = 0; i < obstacles.size(); i++)
        if (obstacles[i].isConvex() && obstacles[i].intersects(p))
            return i;
    return -1;
}

// Checks if the holonomic trajectory described by the Hpm2D intersects with an obstacle in the future.
// This is a continuous and dynamic collision check that takes the motion of the obstacles into
// account. It returns a collision struct with the collision time dt and a pointer to the intersected
// obstacle. If no collision occurs, a default constructed Collision object is returned with dt=-1.
// The trajectory and the obstacles must be in a transformed state for this to work.
Collision GeometricModel::trajectoryCollisionCheck(const Hpm2D &kf) const
{
    Collision col;

    // Check every obstacle.
    for (int i = 0; i < obstacles.size(); i++)
    {
        // Call the intersect algorithm of the specific obstacle type
        // (static, holonomic, unicycle). The type is resolved by polymorphy.
        double ct = obstacles[i].intersects(kf);

        // We have a collision.
        // Is it earlier than the earliest we found so far?
        if (ct >= 0 && (ct < col.dt || col.dt < 0))
        {
            col.dt = ct;
            col.obstacleId = obstacles[i].getId();
            col.obstacleIdx = i;
        }
    }

    return col;
}

// Checks if the unicycle bang described by u intersects with an obstacle in the future.
// This is a continuous and dynamic collision check that takes the motion of the obstacles into
// account. It returns a collision struct with the collision time dt and a pointer to the intersected
// obstacle. If no collision occurs, a default constructed Collision object is returned with dt=-1.
// The trajectory and the obstacles must be in a transformed state for this to work.
Collision GeometricModel::trajectoryCollisionCheck(const Unicycle &u) const
{
    Collision col;

    // Check every obstacle.
    for (int i = 0; i < obstacles.size(); i++)
    {
        // Call the intersect algorithm of the specific obstacle type
        // (static, holonomic, unicycle). The type is resolved by polymorphy.
        double ct = obstacles[i].intersects(u);

        // We have a collision.
        // Is it earlier than the earliest we found so far?
        if (ct >= 0 && (ct < col.dt || col.dt < 0))
        {
            col.dt = ct;
            col.obstacleId = obstacles[i].getId();
            col.obstacleIdx = i;
        }
    }

    return col;
}

void GeometricModel::setDebug(uint d)
{
    debug = d;
}

// Returns a reference to the obstacle with the idx obstIdx.
Obstacle& GeometricModel::getObstacle(int obstIdx)
{
    return obstacles[obstIdx];
}

// Returns a reference to the obstacle with the idx obstIdx.
const Obstacle& GeometricModel::getObstacle(int obstIdx) const
{
    return obstacles[obstIdx];
}

// Returns all obstacles in the geometric model.
const Vector<Obstacle> &GeometricModel::getObstacles() const
{
    return obstacles;
}

// Returns the number of obstascles in the geometric model.
uint GeometricModel::size() const
{
    return obstacles.size();
}

// Returns the number of vertices in the model.
int GeometricModel::getVertexCount() const
{
    uint vertexCount = 0;
    for (uint i = 0; i < obstacles.size(); i++)
        vertexCount += obstacles[i].size();
    return vertexCount;
}

// Prunes the geometric model by locating the deepest polygon
// in the hierarchy that contains point p, making it the root
// polygon and discarding everything above. The root polygon
// becomes a free space polygon where the inside is considered
// to be free space. If no polygon contains p, there is no root
// polygon and the hierarchy will be only one level deep. Then,
// the polygons that are contained by the root polygon (if there
// is one) are added as blocked space polygons. Polygons that
// are inside blocked space polygons are discarded. In the end,
// we either have a two level hierarchy where one root polygon
// is a free space polygon that contains all other polygons on
// level 2, or we have a one level hierarchy basically a list of
// blocked space polygons.
void GeometricModel::prune(Vec2 p)
{
    if (obstacles.isEmpty())
        return;

    // Locate p in the polygon hierarchy.
    int currentIndex = 0;
    int polygonIndex = -1;
    do
    {
        if (obstacles[currentIndex].intersects(p))
        {
            polygonIndex = currentIndex;
            currentIndex = hierarchy[currentIndex][2]; // decend in the hierarchy
            //qDebug() << "Prune decend" << currentIndex;
        }
        else
        {
            currentIndex = hierarchy[currentIndex][0]; // next polygon on the same level
            //qDebug() << "Prune next" << currentIndex;
        }
    } while (currentIndex > 0);

    //qDebug() << "located in polygon" << polygonIndex;

    // Prune and rewrite the obstacles and the hierarchy.
    int parentIdx = -1;
    Vector<Obstacle> prunedObstacles;
    Vector<VecN<4> > prunedHierarchy; // Polygon hierarchy information according to opencv.
    if (polygonIndex > -1)
    {
        prunedObstacles << obstacles[polygonIndex];
        prunedObstacles.last().type = Obstacle::FreeSpace;
        if (hierarchy[polygonIndex][2] > -1) // has a child
            prunedHierarchy << VecN<4>(-1, -1, 1, -1);
        else
            prunedHierarchy << VecN<4>(-1, -1, -1, -1);
        currentIndex = hierarchy[polygonIndex][2]; // child
    }
    else
    {
        currentIndex = 0;
    }

    while (currentIndex > -1)
    {
        prunedObstacles << obstacles[currentIndex];
        currentIndex = hierarchy[currentIndex][0]; // next polygon on the same level
        int prevIdx = (prunedObstacles.size()-2 == 0) ? -1 : (prunedObstacles.size()-2);
        if (currentIndex > -1)
            prunedHierarchy << VecN<4>(prunedObstacles.size(), prevIdx, -1, parentIdx);
        else
            prunedHierarchy << VecN<4>(-1, prunedObstacles.size()-2, -1, parentIdx);
        //qDebug() << "pol:" << polygonIndex << "cur" << currentIndex;
    }

    obstacles = prunedObstacles;
    hierarchy = prunedHierarchy;
    //qDebug() << "hierarchy:" << hierarchy;
}

// Returns the axis aligned bounding box that contains all
// obstacles in the geometric scene.
Box GeometricModel::getBoundingBox() const
{
    if (obstacles.isEmpty())
        return Box();

    Box box = obstacles[0].boundingBox();
    for (uint i = 1; i < obstacles.size(); i++)
    {
        const Box& bb = obstacles[i].boundingBox();
        if (bb.left() < box.left())
            box.setLeft(bb.left());
        if (bb.right() > box.right())
            box.setRight(bb.right());
        if (bb.top() > box.top())
            box.setTop(bb.top());
        if (bb.bottom() < box.bottom())
            box.setBottom(bb.bottom());
    }

    return box;
}

// Draws the geometric model on a QPainter.
void GeometricModel::draw(QPainter *painter, const QPen &pen, const QBrush &brush, double opacity) const
{
    painter->save();

    QFont font;
    font.setFamily("Arial");
    font.setPointSize(1);
    painter->setFont(font);

    // The obstacles.
    painter->setPen(pen);
    for (int i = 0; i < obstacles.size(); i++)
    {
        painter->setOpacity(opacity);
        painter->setBrush(brush);
        if (obstacles[i].type == Obstacle::FreeSpace)
        {
            painter->setBrush(colorUtil.brushWhite);
            painter->setOpacity(0.8);
        }
        obstacles[i].draw(painter);

        painter->setOpacity(1.0);
        painter->setBrush(Qt::NoBrush);
        obstacles[i].draw(painter);

        // The obstacle id label.
        if (config.debugLevel > 4)
        {
            painter->save();
            painter->translate(obstacles[i].centroid());
            painter->scale(0.5, -0.5);
            painter->setOpacity(0.8);
            painter->drawText(QPointF(), QString::number(obstacles[i].getId()));
            painter->restore();
        }
    }

    painter->restore();
}

// Draws the polygons in OpenGL context.
void GeometricModel::draw(const QPen &pen, const QBrush &brush, double opacity) const
{
    glPushMatrix();
    for (uint i = 0; i < obstacles.size(); i++)
    {
        if (obstacles[i].type == Obstacle::FreeSpace)
        {
            QColor mc = colorUtil.brushIvory.color();
            mc.setAlphaF(0.8);
            obstacles[i].draw(mc);
            glTranslated(0, 0, 0.001);
        }
        else
        {
            QColor mc = brush.color();
            mc.setAlphaF(opacity);
            obstacles[i].draw(mc);
        }
    }
    glPopMatrix();
}

// Draws the bounding boxes of all contained objects.
void GeometricModel::drawBoundingBoxes(QPainter *painter) const
{
    for (int i = 0; i < obstacles.size(); i++)
        obstacles[i].boundingBox().draw(painter);
}

// Draws the bounding boxes of all contained objects.
void GeometricModel::drawVisibilityGraph(QPainter *painter) const
{
    visibilityGraph.draw(painter);
}


// Writes the GeometricModel into a data stream.
void GeometricModel::streamOut(QDataStream &out) const
{
    out << obstacles;
}

// Reads the GeometricModel from a data stream.
void GeometricModel::streamIn(QDataStream &in)
{
    in >> obstacles;
}

// Builds a flat hierarchy structure where every polygon is a root.
void GeometricModel::buildFlatHierarchy()
{
    hierarchy.resize(obstacles.size());
    for (int i = 0; i < hierarchy.size(); i++)
    {
        hierarchy[i][0] = i < hierarchy.size()-1 ? i+1 : -1; // next
        hierarchy[i][1] = i-1; // previous
        hierarchy[i][2] = -1; // child
        hierarchy[i][3] = -1; // parent
    }
}

void GeometricModel::rewriteIds()
{
    uint baseId = 0;
    for (uint i = 0; i < obstacles.size(); i++)
    {
        obstacles[i].setBaseLineId(baseId);
        baseId+= obstacles[i].size();
    }
}

QDataStream& operator<<(QDataStream& out, const GeometricModel &o)
{
    o.streamOut(out);
    return out;
}

QDataStream& operator>>(QDataStream& in, GeometricModel &o)
{
    o.streamIn(in);
    return in;
}

QDebug operator<<(QDebug dbg, const GeometricModel &w)
{
    dbg << w.getObstacles();
    return dbg;
}
