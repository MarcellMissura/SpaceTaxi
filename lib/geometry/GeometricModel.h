#ifndef GEOMETRICMODEL_H_
#define GEOMETRICMODEL_H_
#include "robotcontrol/agents/UnicycleObstacle.h"
#include "lib/util/Vector.h"
#include "lib/util/Vec2.h"
#include "lib/geometry/Box.h"
#include "lib/geometry/VisibilityGraph.h"
#include "Collision.h"
#include <QPainter>
#include <QDebug>

class GeometricModel
{
    LinkedList<Polygon> rootPolygons; // A LinkedList of all surrounding CCW free space root Polygons.
    LinkedList<Polygon> polygons; // A LinkedList of all static CW blocked space Polygons.
    LinkedList<Polygon> dilatedPolygons; // A LinkedList of all dilated static CW blocked space Polygons.
    LinkedList<UnicycleObstacle> unicycleObstacles; // Obstacles that move like the unicycle model.
    VisibilityGraph visibilityGraph; // For path queries.
    int pathSearchMode; // Static or dynamic path search.
    Polygon sinkPolygon; // To return a reference to sometimes.

    enum
    {
        Static,
        Dynamic
    };

public:
    GeometricModel();
    ~GeometricModel() {}

    GeometricModel(const GeometricModel& o);
    GeometricModel& operator=(const GeometricModel& v);

    void clear();
    void resetSearch();
    bool isEmpty() const;

    // Scene population functions.
    void worldImport(const LinkedList<Polygon> &pols);
    void prune(const Vec2& p);

    const LinkedList<Polygon>& getRootPolygons() const;

    const LinkedList<Polygon>& getDilatedPolygons() const;
    void setDilatedPolygons(const LinkedList<Polygon>& obst);

    const LinkedList<Polygon>& getPolygons() const;
    void setPolygons(const LinkedList<Polygon>& obst);
    void addPolygons(const LinkedList<Polygon>& obst);
    void addPolygon(const Polygon& o);

    const LinkedList<UnicycleObstacle>& getObstacles() const;
    void setObstacles(const Vector<UnicycleObstacle>& o);
    void setObstacles(const LinkedList<UnicycleObstacle>& o);
    void addObstacle(const UnicycleObstacle& o);
    void addObstacles(const LinkedList<UnicycleObstacle>& obst);
    void eraseObstacles();

    void operator+=(const GeometricModel& o);
    GeometricModel operator+(const GeometricModel& o);

    // Various geometric queries.
    uint getObjectCount() const;
    int getVertexCount() const;
    Box getBoundingBox() const;
    Vec2 getClosestPoint(const Vec2& v) const;
    double distance(const Vec2& v) const;
    double dynamicDistance(const Vec2& v) const;
    Vec2 closestNormal(const Vec2& p) const;
    Vec2 closestPoint(const Vec2& p) const;
    void closestPointNormal(const Vec2& p, Vec2& closestPoint, Vec2& closestNormal) const;

    // Order maintenance functions.
    void setObstacleIds(int id);
    void renumber();
    void reverseOrder();
    void simplify(double epsilon=0.05);
    void pruneOut(double epsilon=0.05);
    void pruneIn(double epsilon=0.05);

    // Transformation functions.
    void scale(double alpha);
    void scale(double alpha, double beta);
    void scale(const Vec2& s);
    void rotate(double a);
    void translate(double dx, double dy);
    void translate(const Vec2& v);
    void transform();
    void operator-=(const Pose2D& o);
    void operator+=(const Pose2D& o);
    GeometricModel operator-(const Pose2D& o);
    GeometricModel operator+(const Pose2D& o);

    // Clipping, offsetting, and boolean operations.
    void unite(const GeometricModel& gm);
    void unite(const Vector<Polygon>& rootPols, const Vector<Polygon>& pols);
    void unite(const Polygon& pol);
    void clip(const Box& box);
    void dilate(double delta);
    void observationUpdate(const Vector<Polygon>& rootPolygons, const Vector<Polygon>& pols);
    void clipRoot(const Vector<Polygon>& pol);
    void clipConvex(const Polygon& pol);
    void clipPolygons(const Vector<Polygon>& pols);
    void clipPolygons(const Polygon& pol);

    // Collision detection methods.
    bool isInFreeSpace(const Vec2& p, bool debug=false) const;
    Vec2 rayIntersection(const Vec2& from, const Vec2& to, bool debug=false) const; // Ray casting.
    const Polygon& rootPointCollisionCheck(const Vec2& p, bool debug=false) const;
    const Polygon& dilatedPointCollisionCheck(const Vec2& p, bool debug=false) const;
    const Polygon& dilatedLineCollisionCheck(const Line &l, bool debug=false) const;
    const Polygon& polygonCollisionCheck(const Polygon &p, bool debug=false) const;
    Collision trajectoryCollisionCheck(const Hpm2D &kf) const;
    Collision trajectoryCollisionCheck(const Unicycle &u, bool debug=false) const;

    // Prediction methods.
    void predict(double dt);
    GeometricModel predicted(double dt) const;
    void autoPredict(bool debug=false);

    // Path planning.
    Vec2 moveIntoFreeSpace(const Vec2& p, bool debug=false) const;
    void setBounds(double t, double l, double b, double r);
    void setBounds(const Box& bb);
    const Box& getBounds() const;
    bool computeStaticPath(const Vec2 &from, const Vec2 &to, int debug=0);
    bool computeDynamicPath(const Vec2 &from, const Vec2 &to, int debug=0);
    const Path &getPath() const;

    // Drawing methods.
    void draw(const QPen &pen, const QBrush &brush, double opacity=0.5) const;
    void draw(QPainter* painter, const QPen &pen, const QBrush &polygonBrush, const QBrush &dilatedPolygonBrush) const;
    void drawBoundingBoxes(QPainter* painter) const;
    void drawVisibilityGraph(QPainter* painter) const;
    void drawVisibilityGraph() const;

    // Streaming.
    void streamOut(QDataStream &out) const;
    void streamIn(QDataStream &in);
};

QDebug operator<<(QDebug dbg, const GeometricModel &w);
QDebug operator<<(QDebug dbg, const GeometricModel *w);
QDataStream& operator<<(QDataStream& out, const GeometricModel &o);
QDataStream& operator>>(QDataStream& in, GeometricModel &o);

#endif
