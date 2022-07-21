#ifndef BOX_H_
#define BOX_H_
#include "util/Vec2.h"
#include "geometry/Line.h"

// This is a lightweight axis-aligned box class used for the purpose of bounding box tests.
// The box is defined by its center point x,y and its half width and height h,w.
// The box offers an interface for fast intersection with geometric primitives.


class Box
{
protected:

    double x,y,t,l,b,r;

public:

    Box();
    Box(double t, double l, double b, double r);
    Box(double x, double y, double t, double l, double b, double r);
    ~Box(){}

    void set(double t, double l, double b, double r);
    void set(double x, double y, double t, double l, double b, double r);

    double height() const;
    double width() const;

    double left() const;
    double right() const;
    double top() const;
    double bottom() const;    
    void setLeft(double ll);
    void setRight(double rr);
    void setTop(double tt);
    void setBottom(double bb);
    Vec2 topLeft() const;
    Vec2 topRight() const;
    Vec2 bottomLeft() const;
    Vec2 bottomRight() const;

    Vec2 pos() const;
    void setPos(const Vec2& p);
    void setPos(double x, double y);
    void translate(double dx, double dy);
    void translate(const Vec2& d);

    void grow(double s);
    bool isEmpty() const;

    void operator+=(const Box& b);
    Box operator+(const Box& b) const;

    bool intersects(const Box &o) const;
    bool intersects(const Line &l) const;
    bool intersects(const Vec2 &p) const;
    bool intersects(const Vec2 &p, double radius) const;
    bool isOnBounds(const Vec2 &p) const;

    Vec2 intersection(const Line& l) const;

    void draw(QPainter* painter) const;
    void draw() const;

    void streamOut(QDataStream& out) const;
    void streamIn(QDataStream& in);
};

QDebug operator<<(QDebug dbg, const Box &o);

#endif // Box_H
