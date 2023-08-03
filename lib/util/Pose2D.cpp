#include "Pose2D.h"

// The Pose2D represents the position and the orientation (heading)
// of an agent on a plane. The position is the (x,y) coordinates
// on the Cartesian plane, the orientation (z) is the direction
// the robot is facing. Think of it as if the robot has driven to
// coordinates (x,y) first and then turned in place into the
// direction of z. The Pose2D class provides arithmetic functions
// for transforming a point, a line, or a Pose2D into the local
// frame of a Pose2D (e.g. Vec2 - Pose2D), and for transforming
// from local coordinates to world coordinates (e.g. Vec2 + Pose2D).
// Transformations of Vec2, Vec3, Line, and Pose2D are supported
// along with Vectors and LinkedLists filled with these objects.

// Constructs a default Pose2D with (x,y,z) = (0,0,0).
Pose2D::Pose2D()
{
    x = 0;
    y = 0;
    z = 0;
}

// Constructs a Pose2D with (x,y,z) = (o.x,o.y,zo).
Pose2D::Pose2D(const Vec2 &o, double zo)
{
    x = o.x;
    y = o.y;
    z = zo;
}

// Constructs a Pose2D with (x,y,z) = (xo,yo,zo).
Pose2D::Pose2D(double xo, double yo, double zo)
{
    x = xo;
    y = yo;
    z = zo;
}

// Returns true if x,y, and the heading are all (nearly) zero.
bool Pose2D::isNull() const
{
    return (fabs(x) < EPSILON && fabs(y) < EPSILON && fabs(z) < EPSILON);
}

// Resets the Pose2D to all zeros.
void Pose2D::setZero()
{
    x = 0;
    y = 0;
    z = 0;
}

// Sets the (x,y) position of the Pose2D.
void Pose2D::setPos(double x, double y)
{
    this->x = x;
    this->y = y;
}

// Sets the position of the Pose2D to p and the heading to the angle of to.
void Pose2D::set(const Vec2 &p, const Vec2 &to)
{
    setPos(p);
    setHeading(to.angle());
}

// Sets the position of the Pose2D to p and the heading to rot.
void Pose2D::set(const Vec2 &p, double rot)
{
    setPos(p);
    setHeading(rot);
}

// Sets the (x,y) position of the Pose2D.
void Pose2D::setPos(const Vec2& p)
{
    this->x = p.x;
    this->y = p.y;
}

// Returns the (x,y) position of the Pose2D.
Vec2 Pose2D::pos() const
{
    return Vec2(x, y);
}

// Translates the Pose2D by (dx,dy).
void Pose2D::translate(double dx, double dy)
{
    setPos(this->x+dx, this->y+dy);
}

// Translates the Pose2D by d.
void Pose2D::translate(const Vec2 &d)
{
    setPos(this->x+d.x, this->y+d.y);
}

// "Turns" the pose in place, i.e., changes the heading by d.
void Pose2D::turn(double d)
{
    z = ffpicut(z+d);
}

// Returns a pose that is "turned" in place, i.e., changes the heading by d.
Pose2D Pose2D::turned(double a) const
{
    Pose2D d = *this;
    d.turn(a);
    return d;
}

// Rotates the Pose2D counter clockwise by the angle "a" given in radians.
// Be aware that while turn() rotates the pose in place, rotate() rotates
// the pose around the origin.
void Pose2D::rotate(double a)
{
    if (a > EPSILON || a < -EPSILON)
    {
        double c = fcos(a);
        double s = fsin(a);
        double x_ = x;
        x = x_*c + y*-s;
        y = x_*s + y*c;
        turn(a);
    }
}

// Returns the heading z of the Pose2D.
double Pose2D::heading() const
{
    return z;
}

// Sets the heading z of the Pose2D.
void Pose2D::setHeading(double a)
{
    z = ffpicut(a);
}

// Returns the orientation (heading) z of the Pose2D.
double Pose2D::orientation() const
{
    return z;
}

// Sets the orientation (heading) z of the Pose2D.
void Pose2D::setOrientation(double a)
{
    //z = a;
    z = ffpicut(a);
}

// Returns the norm of this transformation, which is |x|+|y|+|z|.
double Pose2D::norm() const
{
    return fabs(x)+fabs(y)+fabs(z);
}

// Returns the distance between two poses defined as (sqrt((x1-x2)²+(y1-y2)²) + |z1-z2|.
double Pose2D::dist(const Pose2D& p) const
{
    return sqrt((x-p.x)*(x-p.x)+(y-p.y)*(y-p.y)) + fabs(ffpicut(z-p.z));
}

// Returns the distance between two poses in the xy plane defined as (sqrt((x1-x2)²+(y1-y2)²).
double Pose2D::distxy(const Pose2D& p) const
{
    return sqrt((x-p.x)*(x-p.x)+(y-p.y)*(y-p.y));
}

bool Pose2D::operator==(const Pose2D &o) const
{
    return (fabs(x-o.x) < EPSILON && fabs(y-o.y) < EPSILON && fabs(ffpicut(z-o.z)) < EPSILON);
}

bool Pose2D::operator!=(const Pose2D &o) const
{
    return (fabs(x-o.x) > EPSILON || fabs(y-o.y) > EPSILON || fabs(ffpicut(z-o.z)) > EPSILON);
}

// Inverts this pose.
Pose2D Pose2D::operator-() const
{
    Pose2D c = *this;
    c.x = -c.x;
    c.y = -c.y;
    c.z = -c.z;
    return c;
}

// Maps this pose into the coordinate frame of the given Pose2D o.
Pose2D Pose2D::operator-(const Pose2D &o) const
{
    Pose2D c = *this;
    c -= o;
    return c;
}

// Maps this pose from the coordinate frame of the given Pose2D o to world coordinates.
Pose2D Pose2D::operator+(const Pose2D &o) const
{
    Pose2D c = *this;
    c += o;
    return c;
}

// Maps this pose from world coordinates into the coordinate frame of the given Pose2D o.
void Pose2D::operator-=(const Pose2D &o)
{
    x -= o.x;
    y -= o.y;
    z -= o.z;
    z = ffpicut(z);

    if (o.z < EPSILON && o.z > -EPSILON)
        return;

    double c = fcos(-o.z);
    double s = fsin(-o.z);
    double x_ = x;
    x = x_*c + y*-s;
    y = x_*s + y*c;

    return;
}

// Maps this pose from the coordinate frame of the given Pose2D o to world coordinates.
void Pose2D::operator+=(const Pose2D &o)
{
    if (o.z > EPSILON || o.z < -EPSILON)
    {
        double c = fcos(o.z);
        double s = fsin(o.z);
        double x_ = x;
        x = x_*c + y*-s;
        y = x_*s + y*c;
    }

    x += o.x;
    y += o.y;
    z += o.z;
    z = ffpicut(z);
}

// Returns a Transform3D matrix fabricated from this Pose2D.
Transform3D Pose2D::getMatrix() const
{
    Transform3D mat;
    mat.setFromParams(x, y, 0, 0, 0, z);
    return mat;
}

QTransform Pose2D::getQTransform() const
{
    QTransform tr;
    tr.translate(x,y);
    tr.rotateRadians(z);
    return tr;
}

// Maps the Vec2 v into the coordinate frame of the Pose2D p.
Vec2 operator-(const Vec2& v, const Pose2D& p)
{
    Vec2 c = v;
    c -= p;
    return c;
}

// Maps the Vec2 v from the coordinate frame of Pose2D p to world coordinates.
Vec2 operator+(const Vec2& v, const Pose2D& p)
{
    Vec2 c = v;
    c += p;
    return c;
}

// Maps the Vec3 v into the coordinate frame of the Pose2D p.
Vec3 operator-(const Vec3& v, const Pose2D& p)
{
    Vec3 c = v;
    c -= p;
    return c;
}

// Maps the Vec2 v from the coordinate frame of Pose2D p to world coordinates.
Vec3 operator+(const Vec3& v, const Pose2D& p)
{
    Vec3 c = v;
    c += p;
    return c;
}

// Maps the Vec2 v into the coordinate frame of the Pose2D p.
void operator-=(Vec2& v, const Pose2D& p)
{
    v.x -= p.x;
    v.y -= p.y;
    v.frotate(-p.z);
}

// Maps the Vec2 v from the coordinate frame of Pose2D p to world coordinates.
void operator+=(Vec2& v, const Pose2D& p)
{
    v.frotate(p.z);
    v.x += p.x;
    v.y += p.y;
}

// Maps the Vec3 v into the coordinate frame of the Pose2D p.
void operator-=(Vec3& v, const Pose2D& p)
{
    v.x -= p.x;
    v.y -= p.y;
    v.frotate(-p.z);
}

// Maps the Vec3 v from the coordinate frame of Pose2D p to world coordinates.
void operator+=(Vec3& v, const Pose2D& p)
{
    v.frotate(p.z);
    v.x += p.x;
    v.y += p.y;
}

Vector<Vec3> operator+(const Vector<Vec3> &v, const Pose2D &p)
{
    static Vector<Vec3> tmp;
    tmp.resize(v.size());
    for (int i = 0; i < v.size(); ++i)
        tmp[i] = v[i] + p;
    return tmp;
}

Vector<Vec3> operator-(const Vector<Vec3> &v, const Pose2D &p)
{
    static Vector<Vec3> tmp;
    tmp.resize(v.size());
    for (int i = 0; i < v.size(); ++i)
        tmp[i] = v[i] - p;
    return tmp;
}

Vector<Vec2> operator+(const Vector<Vec2> &v, const Pose2D &p)
{
    static Vector<Vec2> tmp;

    tmp.resize(v.size());
    for (int i = 0; i < v.size(); ++i)
        tmp[i] = v[i] + p;
    return tmp;
}

Vector<Vec2> operator-(const Vector<Vec2> &v, const Pose2D &p)
{
    static Vector<Vec2> tmp;
    tmp.resize(v.size());
    for (int i = 0; i < v.size(); ++i)
        tmp[i] = v[i] - p;
    return tmp;
}

void operator+=(Vector<Vec3> &v, const Pose2D &p)
{
    for (int i = 0; i < v.size(); ++i)
        v[i] += p;
}

void operator-=(Vector<Vec3> &v, const Pose2D &p)
{
    for (int i = 0; i < v.size(); ++i)
        v[i] -= p;
}

void operator+=(Vector<Vec2> &v, const Pose2D &p)
{
    for (int i = 0; i < v.size(); ++i)
        v[i] += p;
}

void operator-=(Vector<Vec2> &v, const Pose2D &p)
{
    for (int i = 0; i < v.size(); ++i)
        v[i] -= p;
}

// Maps the Line l from the coordinate frame of Pose2D p to world coordinates.
Line operator+(const Line& l, const Pose2D& p)
{
    Line c = l;
    c += p;
    return c;
}

// Maps the Line l into the coordinate frame of the Pose2D p.
Line operator-(const Line& l, const Pose2D& p)
{
    Line c = l;
    c -= p;
    return c;
}

// Maps the Line l from the coordinate frame of Pose2D p to world coordinates.
void operator+=(Line& l, const Pose2D& p)
{
    l.rotate(p.z);
    l.translate(p.x, p.y);
}

// Maps the Line l into the coordinate frame of the Pose2D p.
void operator-=(Line& l, const Pose2D& p)
{
    l.translate(-p.x, -p.y);
    l.rotate(-p.z);
}

Vector<Line> operator+(const Vector<Line> &v, const Pose2D &p)
{
    thread_local Vector<Line> tmp;
    tmp.resize(v.size());
    for (uint i = 0; i < v.size(); ++i)
        tmp[i] = v[i] + p;
    return tmp;
}

Vector<Line> operator-(const Vector<Line> &v, const Pose2D &p)
{
    thread_local Vector<Line> tmp;
    tmp.resize(v.size());
    for (uint i = 0; i < v.size(); ++i)
        tmp[i] = v[i] - p;
    return tmp;
}

void operator+=(Vector<Line> &v, const Pose2D &p)
{
    for (uint i = 0; i < v.size(); ++i)
        v[i] += p;
}

void operator-=(Vector<Line> &v, const Pose2D &p)
{
    for (uint i = 0; i < v.size(); ++i)
        v[i] -= p;
}

LinkedList<Line> operator+(const LinkedList<Line> &v, const Pose2D &p)
{
    thread_local LinkedList<Line> tmp;
    tmp.clear();
    ListIterator<Line> it = v.begin();
    while (it.hasNext())
        tmp << it.next()+p;
    return tmp;
}

LinkedList<Line> operator-(const LinkedList<Line> &v, const Pose2D &p)
{
    thread_local LinkedList<Line> tmp;
    tmp.clear();
    ListIterator<Line> it = v.begin();
    while (it.hasNext())
        tmp << it.next()-p;
    return tmp;
}

void operator+=(LinkedList<Line> &v, const Pose2D &p)
{
    ListIterator<Line> it = v.begin();
    while (it.hasNext())
    {
        Line& l = it.next();
        l += p;
    }
}

void operator-=(LinkedList<Line> &v, const Pose2D &p)
{
    ListIterator<Line> it = v.begin();
    while (it.hasNext())
    {
        Line& l = it.next();
        l -= p;
    }
}

LinkedList<Vec2> operator+(const LinkedList<Vec2> &v, const Pose2D &p)
{
    LinkedList<Vec2> tmp;
    tmp.clear();
    ListIterator<Vec2> it = v.begin();
    while (it.hasNext())
        tmp << it.next()+p;
    return tmp;
}

LinkedList<Vec2> operator-(const LinkedList<Vec2> &v, const Pose2D &p)
{
    LinkedList<Vec2> tmp;
    tmp.clear();
    ListIterator<Vec2> it = v.begin();
    while (it.hasNext())
        tmp << it.next()-p;
    return tmp;
}

LinkedList<Vec3> operator+(const LinkedList<Vec3> &v, const Pose2D &p)
{
    LinkedList<Vec3> tmp;
    tmp.clear();
    ListIterator<Vec3> it = v.begin();
    while (it.hasNext())
        tmp << it.next()+p;
    return tmp;
}

LinkedList<Vec3> operator-(const LinkedList<Vec3> &v, const Pose2D &p)
{
    LinkedList<Vec3> tmp;
    tmp.clear();
    ListIterator<Vec3> it = v.begin();
    while (it.hasNext())
        tmp << it.next()-p;
    return tmp;
}


void operator+=(LinkedList<Vec3> &v, const Pose2D &p)
{
    ListIterator<Vec3> it = v.begin();
    while (it.hasNext())
    {
        Vec3& l = it.next();
        l += p;
    }
}

void operator-=(LinkedList<Vec3> &v, const Pose2D &p)
{
    ListIterator<Vec3> it = v.begin();
    while (it.hasNext())
    {
        Vec3& l = it.next();
        l -= p;
    }
}

void operator+=(LinkedList<Vec2> &v, const Pose2D &p)
{
    ListIterator<Vec2> it = v.begin();
    while (it.hasNext())
    {
        Vec2& l = it.next();
        l += p;
    }
}

void operator-=(LinkedList<Vec2> &v, const Pose2D &p)
{
    ListIterator<Vec2> it = v.begin();
    while (it.hasNext())
    {
        Vec2& l = it.next();
        l -= p;
    }
}

void Pose2D::streamOut(QDataStream& out) const
{
    out << x;
    out << y;
    out << z;
}

void Pose2D::streamIn(QDataStream &in)
{
    in >> x;
    in >> y;
    in >> z;
}

QDataStream& operator<<(QDataStream& out, const Pose2D &o)
{
    o.streamOut(out);
    return out;
}

QDataStream& operator>>(QDataStream& in, Pose2D &o)
{
    o.streamIn(in);
    return in;
}

QDebug operator<<(QDebug dbg, const Pose2D &o)
{
    dbg << o.pos() << o.heading();
    return dbg;
}
