#ifndef LINE_H_
#define LINE_H_
#include <QPainter>
#include "lib/util/Vec2.h"

class Line
{
    Vec2 vp1; // p1
    Vec2 vp2; // p2
    mutable double ang;

public:
    int id;
    int type; // SightLine or BlockingLine

    // Line types: sight line and blocking line.
    enum {SightLine, BlockingLine};
    bool isBlockingLine() const;
    bool isSightLine() const;
    void setBlockingLine();
    void setSightLine();
    void setType(int t);
    int getType() const;

    Line();
    Line(int id, const Vec2& p1, const Vec2& p2, int t=BlockingLine);
    Line(const Vec2& p1, const Vec2& p2, int t=BlockingLine);
    Line(double x1, double y1, double x2, double y2, int t=BlockingLine);

    bool operator==(const Line& other) const;
    bool operator!=(const Line& other) const;

    void set(double x1, double y1, double x2, double y2, int t=BlockingLine);
    void set(const Vec2& p1, const Vec2& p2, int t=BlockingLine);

    double& x1();
    double& x2();
    double& y1();
    double& y2();

    const double& x1() const;
    const double& x2() const;
    const double& y1() const;
    const double& y2() const;

    const Vec2& p1() const;
    const Vec2& p2() const;
    Vec2& p1();
    Vec2& p2();
    void setP1(const Vec2& p);
    void setP2(const Vec2& p);
    void flip();
    void sort();
    Line sorted() const;

    Vec2 center() const;
    double a() const;
    double b() const;

    double left() const;
    double right() const;
    double top() const;
    double bottom() const;

    Vec2 leftVertex() const;
    Vec2 rightVertex() const;
    Vec2 topVertex() const;
    Vec2 bottomVertex() const;

    bool isVertical() const;
    bool isHorizontal() const;
    double length() const;
    double length2() const;
    double angle() const;
    double angle(const Line& line) const;
    Vec2 normal() const;
    Vec2 lineVector() const;

    double evaluateAt(double x) const;
    Vec2 interpolate(double coeff) const;
    void interpolate(const Line& l, double coeff);
    Line interpolated(const Line& l, double coeff) const;
    Vec2 projection(const Vec2& v) const;
    Line projectedUnion(const Line& l) const;
    void projectionUnion(const Line& l);
    double projectionDistance(const Line& l) const;
    double projectionOverlap(const Line& p) const;

    bool intersects(const Vec2& v) const;
    bool intersects(const Line& l, bool debug=false) const;
    Vec2 intersection(const Line& l) const;
    Vec2 intersectionInf(const Line& l) const;
    Vec2 rayIntersection(const Vec2& direction, const Vec2& origin = Vec2()) const;

    bool sameSide(const Vec2& v1, const Vec2& v2) const;
    bool sameSide(const Line& l) const;
    bool isLeftOf(const Vec2& p) const;
    bool isRightOf(const Vec2& p) const;

    Vec2 closestPoint(const Vec2& p, bool* tangential=0) const;
    double distance(const Vec2& p) const;
    double orthogonalDistance(const Vec2& p) const;

    double dot(const Line& l) const;
    double dot(const Vec2& v) const;
    double det(const Line& l) const;
    double det(const Vec2& v) const;

    void translate(double dx, double dy);
    void translate(const Vec2& d);
    Line translated(double dx, double dy) const;
    Line translated(const Vec2& d) const;
    void rotate(double angle);
    void rotate(double s, double c);
    Line rotated(double angle) const;
    Line rotated(double s, double c) const;
    void scale(double s);

    // Comparison operator for queing.
    bool operator() (const Line* l1, const Line* l2) {return (l1->length() >= l2->length());}
    bool operator() (const Line& l1, const Line& l2) {return (l1.length() >= l2.length());}

    void draw(QPainter* painter, const QPen &pen=QPen(), double opacity=1.0) const;
    void drawLabel(QPainter* painter, const QPen &pen=QPen(), double opacity=1.0, double rotation=0) const;
    void draw() const;
    void streamOut(QDataStream& out) const;
    void streamIn(QDataStream& in);
    int getId() const;
    void setId(int value);
};

QDebug operator<<(QDebug dbg, const Line &o);
QDataStream& operator<<(QDataStream& out, const Line &o);
QDataStream& operator>>(QDataStream& in, Line &o);

#endif // Line_H
