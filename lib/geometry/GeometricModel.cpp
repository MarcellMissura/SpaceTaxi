#include "GeometricModel.h"
#include "blackboard/Config.h"
#include "blackboard/State.h"
#include "blackboard/Command.h"
#include "lib/util/DrawUtil.h"
#include "lib/util/GLlib.h"

// The GeometricModel is a representation of a polygonal scene.
// The GeometricModel is a feature-rich class that offers an interface to add and retrieve
// polygons and unicycle obstacles as static and dynamic obstacles, respectively.
// The polygons in the GeometricModel are organized in separate structures for root
// polygons, static polygons, dilated static polygons and moving obstacles. Root polygons
// are large, all surrounding polygons with CCW winding that demark free space. The static
// polygons have CW winding and demark blocked space holes in the root polygons. The dilated
// static polygons are also CW and are basically a dilated version of the blocked space polygons,
// even though there does not need to be a one to one correspondence. The moving obstacles are
// also CW wound and represent dynamic obstacles according to the unicycle model.

// One of the main features of the class is the predict() function group that compute
// estimates of the future state of the geometric world modeled by the GeometricModel.
// These predictions would move the dynamic obstacles according to either a weak, constant
// velocity assumption model, or a stronger "follow the shortest path to your target"
// prediction model. For this purpose and for general path planning in the model, a visibility
// graph is integrated into the GeometricModel along with the MinimalConstruct algorithm for
// efficient shortest path queries.

// Apart from predictions, the GeometricModel offers a group of functions for collision
// checking a few different geometric primitives and also holonomic and unicycle motion
// trajectories against the model in static and dynamic ways. Due to its prediction and
// collision checking capabilities, the GeometricModel combines ideally with A* planning.

// Additionally, the polygonal objects in the GeometricModel can be manipulated in a number
// of ways, such as grown(), scale(), unite(), offset(), simplify() and different functions
// for the transformation of the entire scene.

// The GeometricModel harbors a VisibilityGraph inside and offers a computePath() function
// for shortest path queries with the help of the embedded VisibilityGraph.

// And then there is draw()-ing, of course. The Geometric model can be drawn in in OpenGL
// and on QPainter.

GeometricModel::GeometricModel()
{
    visibilityGraph.setGeometricModel(this);
    pathSearchMode = command.Dynamic;
}

// Copy constructor.
GeometricModel::GeometricModel(const GeometricModel &o)
{
    *this = o;
}

// Assignment operator.
GeometricModel& GeometricModel::operator=(const GeometricModel &o)
{
    rootPolygons = o.rootPolygons;
    polygons = o.polygons;
    dilatedPolygons = o.dilatedPolygons;
    unicycleObstacles = o.unicycleObstacles;
    visibilityGraph = o.visibilityGraph;
    visibilityGraph.setGeometricModel(this);
    pathSearchMode = o.pathSearchMode;

    return *this;
}

// Clears the geometric model to a blank state.
void GeometricModel::clear()
{
    rootPolygons.clear();
    polygons.clear();
    dilatedPolygons.clear();
    unicycleObstacles.clear();
    visibilityGraph.reset();
}

// Resets the visibility graph to an initial state.
void GeometricModel::resetSearch()
{
    visibilityGraph.reset();
}

// Returns true when there are no polygons and no obstacles in the model.
bool GeometricModel::isEmpty() const
{
    return (rootPolygons.isEmpty() && polygons.isEmpty() && dilatedPolygons.isEmpty() && unicycleObstacles.isEmpty());
}

// Returns all polygons in the geometric model.
const LinkedList<Polygon> &GeometricModel::getPolygons() const
{
    return polygons;
}

// Returns all root polygons in the geometric model.
const LinkedList<Polygon> &GeometricModel::getRootPolygons() const
{
    return rootPolygons;
}

// Returns all dilated polygons in the geometric model.
const LinkedList<Polygon> &GeometricModel::getDilatedPolygons() const
{
    return dilatedPolygons;
}

// Sets (overwrites) the polygons with the given ones.
void GeometricModel::setPolygons(const LinkedList<Polygon> &pols)
{
    polygons = pols;
}

// Sets (overwrites) the dilated polygons with the given ones.
void GeometricModel::setDilatedPolygons(const LinkedList<Polygon> &obst)
{
    dilatedPolygons = obst;
}

// Adds a polygon to the scene.
void GeometricModel::addPolygon(const Polygon &o)
{
    polygons << o;
}

// Adds (appends) the polygons to the geometric model.
void GeometricModel::addPolygons(const LinkedList<Polygon> &o)
{
    polygons << o;
}

// Returns all obstacles in the geometric model.
const LinkedList<UnicycleObstacle> &GeometricModel::getObstacles() const
{
    return unicycleObstacles;
}

// Sets (overwrites) the unicycle obstacles in the model.
void GeometricModel::setObstacles(const Vector<UnicycleObstacle> &o)
{
    unicycleObstacles = o;
}

// Sets (overwrites) the unicycle obstacles in the model.
void GeometricModel::setObstacles(const LinkedList<UnicycleObstacle> &o)
{
    unicycleObstacles = o;
}

// Adds a unicycle obstacle to the model.
void GeometricModel::addObstacle(const UnicycleObstacle &o)
{
    unicycleObstacles << o;
}

// Adds a list of unicycle obstacles to the model.
void GeometricModel::addObstacles(const LinkedList<UnicycleObstacle> &o)
{
    unicycleObstacles << o;
}

// Removes all dynamic obstacles from the scene.
void GeometricModel::eraseObstacles()
{
    unicycleObstacles.clear();
}

// Adds all objects of GeometricModel o to this one.
void GeometricModel::operator+=(const GeometricModel &o)
{
    rootPolygons << o.rootPolygons;
    polygons << o.polygons;
    dilatedPolygons << o.dilatedPolygons;
    unicycleObstacles << o.unicycleObstacles;
}

// Returns a sum of the polygons and the obstacles of GeometricModel o.
GeometricModel GeometricModel::operator+(const GeometricModel &o)
{
    GeometricModel copy = *this;
    copy += o;
    return copy;
}

// Returns the number of all objects in this geometric model.
uint GeometricModel::getObjectCount() const
{
    return rootPolygons.size() + polygons.size() + dilatedPolygons.size() + unicycleObstacles.size();
}

// Returns the number of all vertices in this model.
int GeometricModel::getVertexCount() const
{
    uint vertexCount = 0;
    ListIterator<Polygon> rootPolyIt = rootPolygons.begin();
    while (rootPolyIt.hasNext())
        vertexCount += rootPolyIt.next().size();
    ListIterator<Polygon> polyIt = polygons.begin();
    while (polyIt.hasNext())
        vertexCount += polyIt.next().size();
    ListIterator<Polygon> dilatedPolyIt = dilatedPolygons.begin();
    while (dilatedPolyIt.hasNext())
        vertexCount += dilatedPolyIt.next().size();
    ListIterator<UnicycleObstacle> obstIt = unicycleObstacles.begin();
    while (obstIt.hasNext())
        vertexCount += obstIt.next().size();
    return vertexCount;
}

// Returns the axis aligned bounding box that contains all
// objects in this geometric scene.
Box GeometricModel::getBoundingBox() const
{
    Box box;
    if (!rootPolygons.isEmpty())
        box = rootPolygons.first().boundingBox();
    else if (!polygons.isEmpty())
        box = polygons.first().boundingBox();
    else if (!dilatedPolygons.isEmpty())
        box = dilatedPolygons.first().boundingBox();
    else if (!unicycleObstacles.isEmpty())
        box = unicycleObstacles.first().boundingBox();
    else
        return box;

    ListIterator<Polygon> rootPolyIt = rootPolygons.begin();
    while (rootPolyIt.hasNext())
    {
        const Box& bb = rootPolyIt.next().boundingBox();
        if (bb.left() < box.left())
            box.setLeft(bb.left());
        if (bb.right() > box.right())
            box.setRight(bb.right());
        if (bb.top() > box.top())
            box.setTop(bb.top());
        if (bb.bottom() < box.bottom())
            box.setBottom(bb.bottom());
    }

    ListIterator<Polygon> polyIt = polygons.begin();
    while (polyIt.hasNext())
    {
        const Box& bb = polyIt.next().boundingBox();
        if (bb.left() < box.left())
            box.setLeft(bb.left());
        if (bb.right() > box.right())
            box.setRight(bb.right());
        if (bb.top() > box.top())
            box.setTop(bb.top());
        if (bb.bottom() < box.bottom())
            box.setBottom(bb.bottom());
    }

    ListIterator<Polygon> dilatedPolyIt = dilatedPolygons.begin();
    while (dilatedPolyIt.hasNext())
    {
        const Box& bb = dilatedPolyIt.next().boundingBox();
        if (bb.left() < box.left())
            box.setLeft(bb.left());
        if (bb.right() > box.right())
            box.setRight(bb.right());
        if (bb.top() > box.top())
            box.setTop(bb.top());
        if (bb.bottom() < box.bottom())
            box.setBottom(bb.bottom());
    }

    ListIterator<UnicycleObstacle> obstIt = unicycleObstacles.begin();
    while (obstIt.hasNext())
    {
        const Box& bb = obstIt.next().boundingBox();
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

// Returns the shortest Euklidean distance between the point v
// and the closest point between the polygons and the obstacles
// in the scene (acting only on the real objects).
double GeometricModel::distance(const Vec2 &v) const
{
    double minDist = std::numeric_limits<double>::max();

    ListIterator<Polygon> polyIt = polygons.begin();
    while (polyIt.hasNext())
    {
        double d = polyIt.next().distance(v);
        if (fabs(d) < fabs(minDist))
            minDist = d;
    }

    ListIterator<UnicycleObstacle> obstIt = unicycleObstacles.begin();
    while (obstIt.hasNext())
    {
        double d = obstIt.next().distance(v);
        if (fabs(d) < fabs(minDist))
            minDist = d;
    }

    return minDist;
}

// Returns the shortest Euklidean distance between the point v
// and the closest point between the obstacles in the scene.
double GeometricModel::dynamicDistance(const Vec2 &v) const
{
    double minDist = std::numeric_limits<double>::max();

    ListIterator<UnicycleObstacle> obstIt = unicycleObstacles.begin();
    while (obstIt.hasNext())
    {
        double d = obstIt.next().distance(v);
        if (fabs(d) < fabs(minDist))
            minDist = d;
    }

    return minDist;
}

// Returns the normal of the vertex or the edge nearest to p.
Vec2 GeometricModel::closestNormal(const Vec2& p) const
{
    double minDist = std::numeric_limits<double>::max();
    Vec2 closestNormal;
    ListIterator<Polygon> polyIt = polygons.begin();
    while (polyIt.hasNext())
    {
        double d = polyIt.cur().distance(p);
        if (fabs(d) < fabs(minDist))
        {
            minDist = d;
            closestNormal = polyIt.cur().closestNormal(p);
        }
        polyIt.next();
    }

    ListIterator<UnicycleObstacle> obstIt = unicycleObstacles.begin();
    while (obstIt.hasNext())
    {
        double d = obstIt.cur().distance(p);
        if (fabs(d) < fabs(minDist))
        {
            minDist = d;
            minDist = d;
            closestNormal = obstIt.cur().closestNormal(p);
        }
        obstIt.next();
    }

    return closestNormal;
}

// Returns the closest point to the point p between the real objects
// (polygons and obstacles) in the scene.
Vec2 GeometricModel::closestPoint(const Vec2& p) const
{
    //qDebug() << "GeometricModel::closestPoint(const Vec2& p):" << p;

    double minDist = std::numeric_limits<double>::max();
    Vec2 closestPoint;
    ListIterator<Polygon> polyIt = polygons.begin();
    while (polyIt.hasNext())
    {
        Vec2 cp = polyIt.cur().closestPoint(p);
        double d = (p-cp).norm();
        if (fabs(d) < fabs(minDist))
        {
            minDist = d;
            closestPoint = cp;
        }
        polyIt.next();
    }

    ListIterator<UnicycleObstacle> obstIt = unicycleObstacles.begin();
    while (obstIt.hasNext())
    {
        Vec2 cp = obstIt.cur().closestPoint(p);
        double d = (p-cp).norm();
        if (fabs(d) < fabs(minDist))
        {
            minDist = d;
            closestPoint = cp;
        }
        obstIt.next();
    }

    return closestPoint;
}

// Returns the closest point and the normal of the closest point in the scene
// to the given point p acting only on the real objects (polygons and obstacles).
// p is given in world coordinates and the returned vectors are also given in
// world coordinates. The normal always points to the left of the nearest polygon
// edge. The winding of the closest polygon determines whether the left is the
// outside or the inside. For the inner CW polygons left means the normal points
// to the outside away from the core. For the CCW root polygon left means the
// normal points to the inside into the polygon core. Edge normals are
// perpendicular to their edge and vertex normals point along the half angle
// between the adjacent edges.
void GeometricModel::closestPointNormal(const Vec2& p, Vec2& closestPoint, Vec2& closestNormal) const
{
    double minDist = std::numeric_limits<double>::max();
    Vec2 point, normal;
    ListIterator<Polygon> polyIt = polygons.begin();
    while (polyIt.hasNext())
    {
        polyIt.cur().closestPointNormal(p, point, normal);
        double d = (p-point).norm();
        if (fabs(d) < fabs(minDist))
        {
            minDist = d;
            closestPoint = point;
            closestNormal = normal;
        }
        polyIt.next();
    }

    ListIterator<UnicycleObstacle> obstIt = unicycleObstacles.begin();
    while (obstIt.hasNext())
    {
        obstIt.cur().closestPointNormal(p, point, normal);
        double d = (p-point).norm();
        if (fabs(d) < fabs(minDist))
        {
            minDist = d;
            closestPoint = point;
            closestNormal = normal;
        }
        obstIt.next();
    }

    return;
}

// Imports the raw map polygons from the world and computes the
// world map from it.
void GeometricModel::worldImport(const LinkedList<Polygon> &pols)
{
    clear();

    // First, create a union of the convex polygons that the simulated
    // world maps are made of. They are set up in a way that there is
    // always a "fence" (a boundary) around the map made of obstacles.
    // Because of this fence, the union of the convex polygons will
    // always have the same result: 1. one large CCW polygon surrounding
    // everything. This is the outer border of the fence. 2. Another
    // large CW polygon surrounding everything. This is the inner border
    // of the fence. 3. A bunch of smaller CCW polygons marking the
    // blocked space.
    polygons = Polygon::unify(pols);


    // Now prune the result of the union to get rid of the outermost
    // polygon and to bring the other polygons into proper winding.
    // The root polygon becomes a CCW polygon where the inside
    // is considered to be free space. Then, the polygons that are
    // contained by the root polygon are added as blocked space
    // polygons with CW winding.

    // Kill the outermost polygon.
    polygons.pop_front();

    // polyIt should be the root now.
    ListIterator<Polygon> polyIt = polygons.begin();

    // Flip the root to CCW.
    bool isCCW = polyIt.cur().isCCW();
    polyIt.cur().ensureCCW();
    //qDebug() << "polyit is now:" << &polyIt.cur() << "CCW:" << isCCW;
    polyIt.next();

    // Flip the next stack to CW.
    while (polyIt.hasNext() && polyIt.cur().isCCW() != isCCW)
    {
        //qDebug() << "flipping" << &polyIt.cur();
        polyIt.next().ensureCW();
    }

    // Remove everything else.
    while (polyIt.hasNext())
    {
        //qDebug() << "clearing" << &polyIt.cur();
        polygons.remove(polyIt);
    }


    // Contract.
    // Contract the polygons with a negative offset in order to create the
    // actual map as the agent would build it.
    polygons = Polygon::offset(polygons, -config.gmDilationRadius, config.gmDouglasPeuckerEpsilon);


    // Classify the polygons in the polygons list into CCW polygons that are
    // moved to the rootPolygons list and into CW polygons that are moved to
    // dilated polygons. This is a silly function only needed to post process
    // the world map.
    polyIt = polygons.begin();
    while (polyIt.hasNext())
    {
        if (polyIt.cur().isCCW())
            rootPolygons << polyIt.cur();
        else
            dilatedPolygons << polyIt.cur();
        polyIt.next();
    }
    polygons.clear();


    // Renumber everything.
    renumber();

    return;
}

// Overwrites the ids of all obstacles with the given one.
// This feature is used for the visualization of a sequence of predictions.
void GeometricModel::setObstacleIds(int id)
{
    ListIterator<UnicycleObstacle> obstIt = unicycleObstacles.begin();
    while (obstIt.hasNext())
        obstIt.next().setId(id);
}

// Rewrites the ids of all polygons and edges in the scene.
void GeometricModel::renumber()
{
    uint polygonCounter = 0;
    uint edgeCounter = 0;

    ListIterator<Polygon> rootPolyIt = rootPolygons.begin();
    while (rootPolyIt.hasNext())
    {
        Polygon& pol = rootPolyIt.next();
        pol.setId(polygonCounter++);
        ListIterator<Line> edges = pol.edgeIterator();
        while (edges.hasNext())
            edges.next().id = edgeCounter++;
    }

    ListIterator<Polygon> polyIt = polygons.begin();
    while (polyIt.hasNext())
    {
        Polygon& pol = polyIt.next();
        pol.setId(polygonCounter++);
        ListIterator<Line> edges = pol.edgeIterator();
        while (edges.hasNext())
            edges.next().id = edgeCounter++;
    }

    ListIterator<Polygon> dilatedPolyIt = dilatedPolygons.begin();
    while (dilatedPolyIt.hasNext())
    {
        Polygon& pol = dilatedPolyIt.next();
        pol.setId(polygonCounter++);
        ListIterator<Line> edges = pol.edgeIterator();
        while (edges.hasNext())
            edges.next().id = edgeCounter++;
    }

    ListIterator<UnicycleObstacle> obstIt = unicycleObstacles.begin();
    while (obstIt.hasNext())
    {
        UnicycleObstacle& obst = obstIt.next();
        obst.setId(polygonCounter++);
        ListIterator<Line> edges = obst.edgeIterator();
        while (edges.hasNext())
            edges.next().id = edgeCounter++;

        obst.hullPolygon.setId(polygonCounter++);
        edges = obst.hullPolygon.edgeIterator();
        while (edges.hasNext())
            edges.next().id = edgeCounter++;
    }
}

// Reverses the order of all polygons and obstacles from CCW to CW or vice versa.
void GeometricModel::reverseOrder()
{
    ListIterator<Polygon> polyIt = polygons.begin();
    while (polyIt.hasNext())
        polyIt.next().reverseOrder();
    ListIterator<UnicycleObstacle> obstIt = unicycleObstacles.begin();
    while (obstIt.hasNext())
        obstIt.next().reverseOrder();
}

// Simplifies all polygons with the Dougals Peucker algorithm.
// The obstacles are usually already simple so they remain untouched.
void GeometricModel::simplify(double epsilon)
{
    ListIterator<Polygon> rootPolyIt = rootPolygons.begin();
    while (rootPolyIt.hasNext())
        rootPolyIt.next().simplify(epsilon);
    ListIterator<Polygon> polyIt = polygons.begin();
    while (polyIt.hasNext())
        polyIt.next().simplify(epsilon);
    ListIterator<Polygon> dilatedPolyIt = dilatedPolygons.begin();
    while (dilatedPolyIt.hasNext())
        dilatedPolyIt.next().simplify(epsilon);
    visibilityGraph.reset();
}

// Scales the whole model by alpha. Multiplies all coordinates with alpha.
void GeometricModel::scale(double alpha)
{
    scale(alpha, alpha);
}

// Scales the whole model by s.x and s.y. Multiplies the x coordinates
// with s.x and the y coordinates with s.y.
void GeometricModel::scale(const Vec2 &s)
{
    scale(s.x, s.y);
}

// Scales the whole model by alpha and beta. Multiplies the x coordinates
// with alpha and the y coordinates with beta.
void GeometricModel::scale(double alpha, double beta)
{
    ListIterator<Polygon> rootPolyIt = rootPolygons.begin();
    while (rootPolyIt.hasNext())
        rootPolyIt.next().scale(alpha, beta);
    ListIterator<Polygon> polyIt = polygons.begin();
    while (polyIt.hasNext())
        polyIt.next().scale(alpha, beta);
    ListIterator<Polygon> dilatedPolyIt = dilatedPolygons.begin();
    while (dilatedPolyIt.hasNext())
        dilatedPolyIt.next().scale(alpha, beta);
    ListIterator<UnicycleObstacle> obstIt = unicycleObstacles.begin();
    while (obstIt.hasNext())
        obstIt.next().scale(alpha, beta);
    visibilityGraph.reset();
}

// Rotates the scene counter clockwise by the angle "a" given in radians
// around the origin.
void GeometricModel::rotate(double a)
{
    ListIterator<Polygon> rootPolyIt = rootPolygons.begin();
    while (rootPolyIt.hasNext())
        rootPolyIt.next().rotate(a);
    ListIterator<Polygon> polyIt = polygons.begin();
    while (polyIt.hasNext())
        polyIt.next().rotate(a);
    ListIterator<Polygon> dilatedPolyIt = dilatedPolygons.begin();
    while (dilatedPolyIt.hasNext())
        dilatedPolyIt.next().rotate(a);
    ListIterator<UnicycleObstacle> obstIt = unicycleObstacles.begin();
    while (obstIt.hasNext())
        obstIt.next().rotate(a);
    visibilityGraph.reset();
}

// Translates the scene by (dx,dy).
void GeometricModel::translate(double dx, double dy)
{
    ListIterator<Polygon> rootPolyIt = rootPolygons.begin();
    while (rootPolyIt.hasNext())
        rootPolyIt.next().translate(dx, dy);
    ListIterator<Polygon> polyIt = polygons.begin();
    while (polyIt.hasNext())
        polyIt.next().translate(dx, dy);
    ListIterator<Polygon> dilatedPolyIt = dilatedPolygons.begin();
    while (dilatedPolyIt.hasNext())
        dilatedPolyIt.next().translate(dx, dy);
    ListIterator<UnicycleObstacle> obstIt = unicycleObstacles.begin();
    while (obstIt.hasNext())
        obstIt.next().translate(dx, dy);
    visibilityGraph.reset();
}

// Translates the scene by (v.x, v.y).
void GeometricModel::translate(const Vec2 &v)
{
    translate(v.x, v.y);
}

// Consumes the transformation of static polygons (root, polygons and dilated).
// The obstacles are not transformed.
void GeometricModel::transform()
{
    ListIterator<Polygon> rootPolyIt = rootPolygons.begin();
    while (rootPolyIt.hasNext())
        rootPolyIt.next().transform();
    ListIterator<Polygon> polyIt = polygons.begin();
    while (polyIt.hasNext())
        polyIt.next().transform();
    ListIterator<Polygon> dilatedPolyIt = dilatedPolygons.begin();
    while (dilatedPolyIt.hasNext())
        dilatedPolyIt.next().transform();
    ListIterator<UnicycleObstacle> obstIt = unicycleObstacles.begin();
    while (obstIt.hasNext())
        obstIt.next().transform();

    visibilityGraph.reset();
}

// Maps the entire scene into the frame given by Pose2D.
void GeometricModel::operator-=(const Pose2D &o)
{
    ListIterator<Polygon> rootPolyIt = rootPolygons.begin();
    while (rootPolyIt.hasNext())
        rootPolyIt.next() -= o;
    ListIterator<Polygon> polyIt = polygons.begin();
    while (polyIt.hasNext())
        polyIt.next() -= o;
    ListIterator<Polygon> dilatedPolyIt = dilatedPolygons.begin();
    while (dilatedPolyIt.hasNext())
        dilatedPolyIt.next() -= o;
    ListIterator<UnicycleObstacle> obstIt = unicycleObstacles.begin();
    while (obstIt.hasNext())
        obstIt.next() -= o;
    transform();
}

// Maps the scene to world coordinates from the frame given by Pose2D.
void GeometricModel::operator+=(const Pose2D &o)
{
    ListIterator<Polygon> rootPolyIt = rootPolygons.begin();
    while (rootPolyIt.hasNext())
        rootPolyIt.next() += o;
    ListIterator<Polygon> polyIt = polygons.begin();
    while (polyIt.hasNext())
        polyIt.next() += o;
    ListIterator<Polygon> dilatedPolyIt = dilatedPolygons.begin();
    while (dilatedPolyIt.hasNext())
        dilatedPolyIt.next() += o;
    ListIterator<UnicycleObstacle> obstIt = unicycleObstacles.begin();
    while (obstIt.hasNext())
        obstIt.next() += o;
    transform();
}

// Returns a copy of the scene mapped into the frame given by Pose2D.
GeometricModel GeometricModel::operator-(const Pose2D &o)
{
    GeometricModel copy = *this;
    copy -= o;
    return copy;
}

// Returns a copy of the scene mapped to world coordinates from the frame
// given by Pose2D.
GeometricModel GeometricModel::operator+(const Pose2D &o)
{
    GeometricModel copy = *this;
    copy += o;
    return copy;
}

// Unites the polygons in this geometric scene with the given polygon.
// Obstacles are ignored. The winding of the polygon determines the
// result of the union. If the winding is CCW, the polygon is taken as
// a member of the freespace layer and united with the freespace polygons
// where the child polygons on the blocked space layer can become holes
// of pol. If the winding is CW, the polygon is taken as a member of the
// deeper blocked space layer and united with the children that are
// holes of the freespace polygons in the higher layer.
// This function is currently used by GeometricMap for uniting one
// visibility polygon with the observed neighborhood of a pose node.
void GeometricModel::unite(const Polygon &pol)
{
    polygons << pol;
    polygons << rootPolygons;
    polygons = Polygon::unify(polygons);
    rootPolygons.clear();
    ListIterator<Polygon> polyIt = polygons.begin();
    while (polyIt.hasNext())
    {
        if (polyIt.cur().isCCW())
        {
            rootPolygons << polyIt.cur();
            polygons.remove(polyIt);
        }
        else
        {
            polyIt.next();
        }
    }
    return;
}

// Unites this geometric scene with the given GeometricModel.
// A union operation is performed on the polygons of this model
// combined with the polygons of gm. The winding of each polygon
// determines the result of the union. After the union, CCW polygons
// are sorted into the root polygons and CW polygons become blocked
// space polygons. This function is used by the GeometricMap to unite
// an observed neighborhood of a pose graph node with the entire
// polygon map.
void GeometricModel::unite(const GeometricModel &gm)
{
    polygons << rootPolygons;
    polygons << gm.getPolygons();
    polygons << gm.getRootPolygons();
    polygons = Polygon::unify(polygons);
    rootPolygons.clear();
    ListIterator<Polygon> polyIt = polygons.begin();
    while (polyIt.hasNext())
    {
        if (polyIt.cur().isCCW())
        {
            rootPolygons << polyIt.cur();
            polygons.remove(polyIt);
        }
        else
        {
            polyIt.next();
        }
    }
    unicycleObstacles << gm.unicycleObstacles;
    return;
}

// Applies the offset operator on the polygons and overwrites
// the dilatedPolygons with the result. The polygons and the
// root polygons and the obstacles remain untouched.
void GeometricModel::dilate(double delta)
{
    if (polygons.isEmpty())
        return;

    dilatedPolygons = Polygon::offset(polygons, delta, config.gmDouglasPeuckerEpsilon);

    return;
}

// Removes everything from the scene that's not inside the box.
// Polygons are clipped and obstacles are removed. This function
// is used, for example, to create the local map from the world map.
void GeometricModel::clip(const Box &box)
{
    ListIterator<Polygon> rootPolyIt = rootPolygons.begin();
    while (rootPolyIt.hasNext())
    {
        Polygon& pol = rootPolyIt.cur();
        //qDebug() << "Clipping pol" << &pol;
        pol.clipBox(box);
        pol.prune();
        if (pol.isEmpty())
            rootPolygons.remove(rootPolyIt);
        else
            rootPolyIt.next();
    }

    ListIterator<Polygon> polyIt = polygons.begin();
    while (polyIt.hasNext())
    {
        Polygon& pol = polyIt.cur();
        //qDebug() << "Clipping pol" << &pol;
        pol.clipBox(box);
        pol.prune();
        if (pol.isEmpty())
            polygons.remove(polyIt);
        else
            polyIt.next();
    }

    ListIterator<Polygon> dilatedPolyIt = dilatedPolygons.begin();
    while (dilatedPolyIt.hasNext())
    {
        Polygon& pol = dilatedPolyIt.cur();
        //qDebug() << "Clipping pol" << &pol;
        pol.clipBox(box);
        pol.prune();
        if (pol.isEmpty())
            dilatedPolygons.remove(dilatedPolyIt);
        else
            dilatedPolyIt.next();
    }

    ListIterator<UnicycleObstacle> obstIt = unicycleObstacles.begin();
    while (obstIt.hasNext())
    {
        if (!box.intersects(obstIt.cur().pos()))
            unicycleObstacles.remove(obstIt);
        else
            obstIt.next();
    }

    return;
}

// Returns true if the point p is inside one of the root polygons
// and does not intersect any blocked space polgons, dilated polygons
// or the hull polygon of an obstacle.
bool GeometricModel::isInFreeSpace(const Vec2 &p, bool debug) const
{
    bool iifs = false;
    ListIterator<Polygon> rootPolyIt = rootPolygons.begin();
    while (rootPolyIt.hasNext() && !iifs)
        if (rootPolyIt.next().intersects(p))
            iifs = true;
    if (!iifs)
        return false;

    ListIterator<Polygon> polyIt = polygons.begin();
    while (polyIt.hasNext())
        if (polyIt.next().intersects(p))
            return false;

    ListIterator<Polygon> dilatedPolyIt = dilatedPolygons.begin();
    while (dilatedPolyIt.hasNext())
        if (dilatedPolyIt.next().intersects(p))
            return false;

    ListIterator<UnicycleObstacle> obstIt = unicycleObstacles.begin();
    while (obstIt.hasNext())
    {
        if (obstIt.cur().hullIntersects(p)) // hullIntersects transforms to local
            return false;
        obstIt.next();
    }

    return true;
}

// Computes a collisionless prediction of the future state of the geometric model at
// time dt. This means the obstacles are moved according to their motion model by
// the time dt, but no collisions are resolved.
void GeometricModel::predict(double dt)
{
    if (unicycleObstacles.isEmpty())
        return;
    ListIterator<UnicycleObstacle> obstIt = unicycleObstacles.begin();
    while (obstIt.hasNext())
        obstIt.next().predict(dt);
    visibilityGraph.reset();
    return;
}

// Computes a collisionless prediction of the future state of the geometric model at
// time dt. This means the obstacles are moved according to their motion model by
// the time dt, but no collisions are resolved.
GeometricModel GeometricModel::predicted(double dt) const
{
    GeometricModel wm = *this;
    wm.predict(dt);
    return wm;
}

// Computes a collisionless prediction of the future state of the geometric model.
// The time of prediction is determined for each obstacle individually depending on
// their distance to the observer. The obstacles are moved according to their motion
// model by their prediction time, but no collisions are resolved.
void GeometricModel::autoPredict(bool debug)
{
    if (unicycleObstacles.isEmpty())
        return;

    ListIterator<UnicycleObstacle> obstIt = unicycleObstacles.begin();
    while (obstIt.hasNext())
    {
        double T = obstIt.cur().pos().norm() / config.agentLinearVelocityLimitForward;
        if (T > config.predictIgnoreHorizon)
        {
            unicycleObstacles.remove(obstIt);
        }
        else
        {
            double dt = min(config.predictFactor*T, config.predictMaxPredictTime);
            if (debug)
                qDebug() << obstIt.cur().getId() << "T:" << T << "dt:" << dt;
            obstIt.cur().predict(dt);
            obstIt.next();
        }
    }

    visibilityGraph.reset();
    return;
}

// Returns the point at which a ray from from to to first intersects any real
// object (polygons and obstacles) in the geometric scene. If the ray does not
// intersect with anything, to is returned.
Vec2 GeometricModel::rayIntersection(const Vec2 &from, const Vec2 &to, bool debug) const
{
    if (debug)
        qDebug() << "rayIntersection(const Vec2 &from, const Vec2 &to)" << from << to;

    Vec2 best = to;
    Line l(from, to);
    double d = l.length();
    ListIterator<Polygon> polyIt = polygons.begin();
    while (polyIt.hasNext())
    {
        const Polygon& pol = polyIt.next();
        Vec2 ip = pol.rayIntersection(l);
        if (debug)
            qDebug() << "pol" << pol.getId() << "ray intersect" << ip;
        double dd = (ip-from).norm();
        if (dd < d)
        {
            best = ip;
            d = dd;
        }
    }

    ListIterator<UnicycleObstacle> obstIt = unicycleObstacles.begin();
    while (obstIt.hasNext())
    {
        const UnicycleObstacle& obst = obstIt.next();
        Vec2 ip = obst.rayIntersection(l);
        double dd = (ip-from).norm();
        if (dd < d)
        {
            best = ip;
            d = dd;
        }
    }

    return best;
}

// Checks if the point p intersects with one of the root polygons.
// If there is a collision, it returns the involved polygon.
const Polygon& GeometricModel::rootPointCollisionCheck(const Vec2 &p, bool debug) const
{
    ListIterator<Polygon> rootPolyIt = rootPolygons.begin();
    while (rootPolyIt.hasNext())
    {
        if (rootPolyIt.cur().intersects(p, false)) // intersection test without the polygon boundary
            return rootPolyIt.cur();
        rootPolyIt.next();
    }

    return sinkPolygon;
}

// Checks if the point p intersects with one of the dilated polygons or the hullPolygon
// of one of the obstacles. If there is a collision, it returns the involved polygon.
// Otherwise it returns an empty polygon.
const Polygon& GeometricModel::dilatedPointCollisionCheck(const Vec2 &p, bool debug) const
{
    ListIterator<Polygon> dilatedPolyIt = dilatedPolygons.begin();
    while (dilatedPolyIt.hasNext())
    {
        if (dilatedPolyIt.cur().intersects(p))
            return dilatedPolyIt.cur();
        dilatedPolyIt.next();
    }

    if (pathSearchMode == command.Dynamic)
    {
        ListIterator<UnicycleObstacle> obstIt = unicycleObstacles.begin();
        while (obstIt.hasNext())
        {
            if (!obstIt.cur().isIgnored() && obstIt.cur().hullIntersects(p)) // hullIntersects transforms to local
                return obstIt.cur().getHullPolygon();
            obstIt.next();
        }
    }

    return sinkPolygon;
}

// Checks if the line l intersects with a root polygon, a dilated polygon or the hull
// polygon of an obstacle in the geometric model. If there is a collision, it returns
// the involved polygon. Otherwise it returns an empty polygon. This function is mostly
// used by Minimal Construct for path finding.
const Polygon& GeometricModel::dilatedLineCollisionCheck(const Line &l, bool debug) const
{
    if (debug)
        qDebug() << "   GeometricModel::dilatedLineCollisionCheck(const Line &l):" << l;

    if (pathSearchMode == command.Dynamic)
    {
        ListIterator<UnicycleObstacle> obstIt = unicycleObstacles.begin();
        while (obstIt.hasNext())
        {
            if (!obstIt.cur().isIgnored() && obstIt.cur().hullIntersects(l, debug))
                return obstIt.cur().getHullPolygon();
            obstIt.next();
        }
    }

    ListIterator<Polygon> polyIt = dilatedPolygons.begin();
    while (polyIt.hasNext())
    {
        if (polyIt.cur().intersects(l))
            return polyIt.cur();
        polyIt.next();
    }

    ListIterator<Polygon> rootPolyIt = rootPolygons.begin();
    while (rootPolyIt.hasNext())
    {
        if (rootPolyIt.cur().intersects(l))
            return rootPolyIt.cur();
        rootPolyIt.next();
    }

    return sinkPolygon;
}

// Checks if the Polygon p intersects with any real object, that is a polygon
// or an obstacle. It returns the polygon of the first found touched object or
// an empty polygon if there is no collision.
const Polygon& GeometricModel::polygonCollisionCheck(const Polygon &p, bool debug) const
{
    ListIterator<UnicycleObstacle> obstIt = unicycleObstacles.begin();
    while (obstIt.hasNext())
    {
        if (debug)
            qDebug() << "   Intersecting obst" << &obstIt.cur() << "with" << &p;
        if (obstIt.cur().intersects(p)) // critical
            return obstIt.cur();
        obstIt.next();
    }

    ListIterator<Polygon> polyIt = polygons.begin();
    while (polyIt.hasNext())
    {
        if (debug)
            qDebug() << "   Intersecting pol" << &polyIt.cur() << "with" << &p;
        if (polyIt.cur().intersects(p))
            return polyIt.cur();
        polyIt.next();
    }

    return sinkPolygon;
}

// Checks if the holonomic trajectory described by the Hpm2D intersects with an obstacle in the future.
// This is a continuous and dynamic collision check that takes the motion of the obstacles into
// account. It returns a collision struct with the collision time dt and a pointer to the intersected
// obstacle. If no collision occurs, a default constructed Collision object is returned with dt=-1.
// The trajectory and the obstacles must be in a transformed state for this to work.
Collision GeometricModel::trajectoryCollisionCheck(const Hpm2D &kf) const
{
    Collision col;

    // Check every polygon.
    ListIterator<Polygon> polyIt = polygons.begin();
    while (polyIt.hasNext())
    {
        const Polygon& pol = polyIt.next();

        // Call the intersect algorithm of the specific obstacle type
        // (static, holonomic, unicycle). The type is resolved by polymorphy.
        double ct = pol.intersects(kf);

        // We have a collision.
        // Is it earlier than the earliest we found so far?
        if (ct >= 0 && (ct < col.dt || col.dt < 0))
        {
            col.dt = ct;
            col.obstacleId = pol.getId();
        }
    }

    // Check the dynamic obstacles.
    ListIterator<UnicycleObstacle> obstIt = unicycleObstacles.begin();
    while (polyIt.hasNext())
    {
        const UnicycleObstacle& obst = obstIt.next();
        double ct = obst.intersects(kf);
        if (ct >= 0 && (ct < col.dt || col.dt < 0))
        {
            col.dt = ct;
            col.obstacleId = obst.getId();
        }
    }

    return col;
}

// Checks if the unicycle bang described by u intersects with an obstacle in the future.
// This is a continuous and dynamic collision check that takes the motion of the obstacles into
// account. It returns a collision struct with the collision time dt and a pointer to the intersected
// obstacle. If no collision occurs, a default constructed Collision object is returned with dt=-1.
// The trajectory and the obstacles must be in a transformed state for this to work.
Collision GeometricModel::trajectoryCollisionCheck(const Unicycle &u, bool debug) const
{
    if (debug)
        qDebug() << "GeometricModel::trajectoryCollisionCheck(const Unicycle &u)" << u;

    Collision col;

    // Check every obstacle.
    ListIterator<Polygon> polyIt = polygons.begin();
    while (polyIt.hasNext())
    {
        const Polygon& pol = polyIt.next();

        // Call the intersect algorithm of the specific obstacle type
        // (static, holonomic, unicycle). The type is resolved by polymorphy.
        double ct = pol.intersects(u);

        // We have a collision.
        // Is it earlier than the earliest we found so far?
        if (ct >= 0 && (ct < col.dt || col.dt < 0))
        {
            col.dt = ct;
            col.obstacleId = pol.getId();
            if (debug)
                qDebug() << "   Collision with pol" << pol.getId() << "ct" << ct;
        }
    }

    // Check the dynamic obstacles.
    ListIterator<UnicycleObstacle> obstIt = unicycleObstacles.begin();
    while (obstIt.hasNext())
    {
        const UnicycleObstacle& obst = obstIt.next();
        if (debug)
            qDebug() << "   Checking obst" << obst;
        double ct = obst.intersects(u, debug);
        if (ct >= 0 && (ct < col.dt || col.dt < 0))
        {
            col.dt = ct;
            col.obstacleId = obst.getId();
            if (debug)
                qDebug() << "   Collision with obst" << obst.getId() << "ct" << ct;
        }
    }

    return col;
}

// Point p is moved into free space in case it is outside the root polygon and
// intersects with blocked space.
Vec2 GeometricModel::moveIntoFreeSpace(const Vec2 &p, bool debug) const
{
    if (debug)
        qDebug() << state.frameId << "GeometricModel::moveIntoFreeSpace(const Vec2 &p)" << p;

    Vec2 v = p;
    int counter = 0;
    bool ok = false;
    while (counter < 10 && !ok)
    {
        ok = true;
        counter++;
        const Polygon& rpol = rootPointCollisionCheck(v, debug);
        if (!rootPolygons.isEmpty() && rpol.isEmpty())
        {
            Vec2 point, normal;
            rootPolygons.first().closestPointNormal(v, point, normal); // BUG: Which root polygon??
            v = point + 0.01*normal;
            ok = false;

            if (debug)
                qDebug() << v << "is not inside a root polygon. moving along point normal:" << point << normal << "to" << v
                         << "test:" << rootPolygons.first().intersects(v);
        }

        const Polygon& pol = dilatedPointCollisionCheck(v, debug);
        if (!pol.isEmpty())
        {
            Vec2 point, normal;
            pol.closestPointNormal(v, point, normal);
            v = point + 0.01*normal;
            ok = false;
            if (debug)
                qDebug() << v << "is inside a blocking polygon. moving along point normal:" << point << normal << "to" << v
                         << "test:" << !pol.intersects(v) << "(true is good)";
        }
    }

    if (debug && v != p)
        qDebug() << "moveIntoFreeSpace():" << p << "moved to" << v << "test:" << isInFreeSpace(v);
    if (debug && counter >= 10)
        qDebug() << "moveIntoFreeSpace() failed.";
    if (debug)
        qDebug() << "MoveIntoFreeSpace() finished.";

    return v;
}

// Sets the scene bounding box. This box is used to confine the search for
// the shortest path within the rectangular area of a local map.
void GeometricModel::setBounds(double t, double l, double b, double r)
{
    visibilityGraph.setBounds(t, l, b, r);
}

// Sets the scene bounding box. This box is used to confine the search for
// the shortest path within the rectangular area of a local map.
void GeometricModel::setBounds(const Box &bb)
{
    visibilityGraph.setBounds(bb);
}

// Returns the bounding box.
const Box &GeometricModel::getBounds() const
{
    return visibilityGraph.getBounds();
}

// Searches the polygonal scene for the shortest path from "from" to "to"
// using only the static polygons. In order to guarantee a path even in situations
// where "from" or "to" are occluded, both from and to are moved into free space.
// The starting point and the target of the search are provided as an argument to
// this function. The returned boolean indicates whether the path was successfully
// found. A path search can be unsuccessful when there really is no way from
// start to target in the map. After a successful search, you can retrieve the
// found path using the getPath() function.
bool GeometricModel::computeStaticPath(const Vec2 &from, const Vec2 &to, int debug)
{
    pathSearchMode = command.Static;
    return visibilityGraph.computePath(from, to, debug);
}

// Searches the polygonal scene for the shortest path from "from" to "to"
// regarding also the moving obstacles. In order to guarantee a path even in
// situations where "from" or "to" are occluded, both from and to are moved into
// free space. The starting point and the target of the search are provided as an
// argument to this function. The returned boolean indicates whether the path was
// successfully found. A path search can be unsuccessful when there really is no
// way from start to target in the map. When the dynamic obstacles are taken into
// account, it is not unlikely that they would block a doorway or a narrow passage
// and no path can be found. In this case, fall back to the static path. After a
// successful search, you can retrieve the found path using the getPath() function.
bool GeometricModel::computeDynamicPath(const Vec2 &from, const Vec2 &to, int debug)
{
    // Ignore all obstacles that block the target.
    ListIterator<UnicycleObstacle> obstIt = unicycleObstacles.begin();
    while (obstIt.hasNext())
    {
        if (obstIt.cur().hullIntersects(to))
            obstIt.cur().ignore();
        obstIt.next();
    }

    pathSearchMode = command.Dynamic; // Determines if obstacles are regarded or not.
    bool ret = visibilityGraph.computePath(from, to, debug);

    // Unignore all obstacles.
    obstIt = unicycleObstacles.begin();
    while (obstIt.hasNext())
        obstIt.next().unignore();

    return ret;
}

// Returns the last computed path.
const Vector<Vec2> &GeometricModel::getPath() const
{
    return visibilityGraph.getPath();
}

// Draws the geometric model on a QPainter.
void GeometricModel::draw(QPainter *painter, const QPen &pen, const QBrush &polygonBrush, const QBrush &dilatedPolygonBrush) const
{
    painter->save();

    QFont font;
    font.setFamily("Arial");
    font.setPointSize(1);
    painter->setFont(font);

    // The root polygons.
    painter->setPen(pen);
    ListIterator<Polygon> rootPolyIt = rootPolygons.begin();
    while (rootPolyIt.hasNext())
    {
        const Polygon& pol = rootPolyIt.next();
        pol.draw(painter, pen, drawUtil.brushWhite, 0.8); // always white
        pol.draw(painter, pen, Qt::NoBrush, 1.0);
        if (config.debugLevel > 5 && command.showLabels)
            pol.drawLabel(painter);
    }

    // The dilated polygons.
    painter->setPen(pen);
    ListIterator<Polygon> dilatedPolyIt = dilatedPolygons.begin();
    while (dilatedPolyIt.hasNext())
    {
        const Polygon& pol = dilatedPolyIt.next();
        pol.draw(painter, pen, dilatedPolygonBrush, 0.5); // dilated brush
        pol.draw(painter, pen, Qt::NoBrush, 1.0);
        if (config.debugLevel > 5  && command.showLabels)
            pol.drawLabel(painter);
    }

    // The polygons.
    painter->setPen(pen);
    ListIterator<Polygon> polyIt = polygons.begin();
    while (polyIt.hasNext())
    {
        const Polygon& pol = polyIt.next();
        pol.draw(painter, pen, polygonBrush, 0.5); // polygon brush
        pol.draw(painter, pen, Qt::NoBrush, 1.0);
        if (config.debugLevel > 5 && command.showLabels)
            pol.drawLabel(painter);
    }

    // The unicycle obstacles.
    painter->setPen(pen);
    ListIterator<UnicycleObstacle> obstIt = unicycleObstacles.begin();
    while (obstIt.hasNext())
    {
        const UnicycleObstacle& obst = obstIt.next();
        obst.drawHull(painter, pen, drawUtil.brushBlue, 0.1);
        obst.draw(painter, pen, drawUtil.brushRed, 0.1);
        if (config.debugLevel > 5 && command.showLabels)
            obst.drawLabel(painter);
    }

    painter->restore();
}

// Draws the polygons in OpenGL context.
void GeometricModel::draw(const QPen &pen, const QBrush &brush, double opacity) const
{
    glPushMatrix();

    ListIterator<Polygon> rootPolyIt = rootPolygons.begin();
    while (rootPolyIt.hasNext())
    {
        const Polygon& pol = rootPolyIt.next();
        pol.draw(pen, brush, opacity);
        glTranslated(0, 0, 0.001);
    }

    ListIterator<Polygon> dilatedPolyIt = dilatedPolygons.begin();
    while (dilatedPolyIt.hasNext())
    {
        const Polygon& pol = dilatedPolyIt.next();
        pol.draw(pen, drawUtil.white, 1.0);
        glTranslated(0, 0, 0.001);
    }

    ListIterator<Polygon> polyIt = polygons.begin();
    while (polyIt.hasNext())
    {
        const Polygon& pol = polyIt.next();
        pol.draw(pen, drawUtil.lightGray, 1.0);
        glTranslated(0, 0, 0.001);
    }

    ListIterator<UnicycleObstacle> obstIt = unicycleObstacles.begin();
    while (obstIt.hasNext())
    {
        const UnicycleObstacle& obst = obstIt.next();
        obst.draw(pen, brush, opacity);
        glTranslated(0, 0, 0.001);
    }

    glPopMatrix();
}

// Draws the bounding boxes of all contained objects.
void GeometricModel::drawBoundingBoxes(QPainter *painter) const
{
    ListIterator<Polygon> polyIt = polygons.begin();
    while (polyIt.hasNext())
        polyIt.next().boundingBox().draw(painter);
    ListIterator<UnicycleObstacle> obstIt = unicycleObstacles.begin();
    while (obstIt.hasNext())
        obstIt.next().boundingBox().draw(painter);
}

// Draws the visibility graph on a QPainter.
void GeometricModel::drawVisibilityGraph(QPainter *painter) const
{
    visibilityGraph.draw(painter);
}

// Draws the visibility graph in an OpenGL context.
void GeometricModel::drawVisibilityGraph() const
{
    visibilityGraph.draw();
}

// Writes the GeometricModel into a data stream.
void GeometricModel::streamOut(QDataStream &out) const
{
    out << rootPolygons;
    out << polygons;
    out << dilatedPolygons;
    out << unicycleObstacles;
}

// Reads the GeometricModel from a data stream.
void GeometricModel::streamIn(QDataStream &in)
{
    in >> rootPolygons;
    in >> polygons;
    in >> dilatedPolygons;
    in >> unicycleObstacles;
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
    dbg << "Root polygons:";
    dbg << w.getRootPolygons();
    dbg << "Polygons:";
    dbg << w.getPolygons();
    dbg << "Dilated polygons:";
    dbg << w.getDilatedPolygons();
    dbg << "Obstacles:";
    dbg << w.getObstacles();
    return dbg;
}

QDebug operator<<(QDebug dbg, const GeometricModel *w)
{
    dbg << "Root polygons:";
    dbg << &w->getRootPolygons();
    dbg << "Polygons:";
    dbg << &w->getPolygons();
    dbg << "Dilated polygons:";
    dbg << &w->getDilatedPolygons();
    dbg << "Obstacles:";
    dbg << &w->getObstacles();
    return dbg;
}
