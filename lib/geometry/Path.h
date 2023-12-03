#ifndef PATH_H_
#define PATH_H_
#include "lib/util/Vec2.h"
#include "lib/util/Vector.h"
#include "lib/util/Pose2D.h"
#include <QPainter>

class Path
{
    Vector<Vec2> vertices; // The edges of the polygon and with that also the vertices in p1.

public:

    Path();
    ~Path(){}

    void clear();

    uint size() const;
    bool isEmpty() const;

    void set(const Vector<Vec2>& v);
    Path& operator<<(const Vec2 &p);
    Path& operator<<(const Vector<Vec2> &lp);
    void reverse();

    const Vector<Vec2>& getVertices() const;

    double length() const;
    double distance(const Vec2 &p) const;
    Vec2 closestPoint(const Vec2 &p) const;

    void draw(QPainter* painter, const QPen &pen, double opacity=1.0) const;
    void draw(const QPen &pen, double alpha=1.0) const;

    void streamOut(QDataStream& out) const;
    void streamIn(QDataStream& in);

    void operator+=(const Pose2D& p);
    void operator-=(const Pose2D& p);

    Vec2& operator[](uint i) {return vertices[i];}
    const Vec2& operator[](uint i) const {return vertices[i];}

    Vec2& last();
    const Vec2& last() const;
};

extern Path operator+(const Path& l, const Pose2D& p);
extern Path operator-(const Path& l, const Pose2D& p);

QDebug operator<<(QDebug dbg, const Path &o);
QDebug operator<<(QDebug dbg, const Path* o);
QDataStream& operator<<(QDataStream& out, const Path &o);
QDataStream& operator>>(QDataStream& in, Path &o);

#endif
