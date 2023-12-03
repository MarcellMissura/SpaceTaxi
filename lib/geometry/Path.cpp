#include "Path.h"
#include "lib/util/DrawUtil.h"
#include "lib/util/GLlib.h"

// The Path class is a...

// Creates an empty Path.
Path::Path()
{

}

// Discards all vertices.
void Path::clear()
{
    vertices.clear();
}

// Returns the number of vertices of this Path.
uint Path::size() const
{
    return vertices.size();
}

// Returns true if the Path has no vertices assigned.
bool Path::isEmpty() const
{
    return vertices.isEmpty();
}

// Returns a Vector of the vertices (corners) of the Path.
const Vector<Vec2>& Path::getVertices() const
{
    return vertices;
}

// Returns the length of the path.
double Path::length() const
{
    double pathLength = 0;
    for (uint i = 1; i < vertices.size(); i++)
        pathLength += (vertices[i] - vertices[i-1]).norm();
    return pathLength;
}

// Sets (overwrites) the vertices of the Path.
void Path::set(const Vector<Vec2> &v)
{
    vertices = v;
}

// Computes the shortest distance between this Path and the point p.
// p is expected to be given in world coordinates. The distance is computed
// between p and the point on the Path that is closest to p.
double Path::distance(const Vec2 &p) const
{
    return (p-closestPoint(p)).norm();
}

// Returns the closest point on the Path to the given point p.
Vec2 Path::closestPoint(const Vec2 &p) const
{
    //qDebug() << "Path::closestPoint(const Vec2 &p):" << p << "polId:" << getId();

    // Determine the closest point.
    double minDist = std::numeric_limits<double>::max();
    Vec2 cp;
    Line edge;
    for (uint i = 1; i < vertices.size(); i++)
    {
        edge.set(vertices[i-1], vertices[i]);
        bool tangential;
        Vec2 pp = edge.closestPoint(p, &tangential);
        //qDebug() << "   edge" << edge << "cp" << pp;
        double d = (pp-cp).norm2();
        if (d <= minDist) // <= is used because sometimes the closest point is p2 of an edge which is the same as p1 of the next edge and p1 is better
        {
            minDist = d;
            cp = pp;
        }
    }

    return cp;
}

// Draws the Path on a QPainter.
// It does not matter whether the Path is transformed or not.
void Path::draw(QPainter *painter, const QPen &pen, double opacity) const
{
    painter->save();
    painter->setPen(pen);
    painter->setBrush(pen.color());
    painter->setOpacity(opacity);
    for (int i = 1; i < vertices.size(); i++)
        if (vertices[i] != vertices[i-1])
            painter->drawLine(vertices[i], vertices[i-1]);
    double s = 0.01;
    for (int i = 0; i < vertices.size(); i++)
        if (!vertices[i].isNan())
            painter->drawEllipse(vertices[i], s, s);
    painter->restore();
}

// Draws the Path in an OpenGL context.
void Path::draw(const QPen& pen, double opacity) const
{
    VecN<4> currentColor;
    glGetDoublev(GL_CURRENT_COLOR, currentColor.data()); // remember the color we had before drawing

    // Set the desired color.
    if (pen.color().isValid())
        glColor4f(pen.color().redF(), pen.color().greenF(), pen.color().blueF(), opacity);

    for (uint i = 1; i < vertices.size(); i++)
        GLlib::drawLine(vertices[i], vertices[i-1], 0.01);

    // Mark the vertices.
    for (uint i = 0; i < vertices.size(); i++)
    {
        Vec2 v = vertices[i];
        glPushMatrix();
        glTranslated(v.x, v.y, 0);
        GLlib::drawCircle(0.005,0.005);
        glPopMatrix();
    }

    glColor4dv(currentColor.data()); // restore the saved color
}

// Appends a vertex to the Path.
Path& Path::operator<<(const Vec2 &p)
{
    vertices << p;
    return *this;
}

// Appends a Vector of vertices to the Path.
Path& Path::operator<<(const Vector<Vec2> &vp)
{
    vertices << vp;
    return *this;
}

// Reverses the order of the path.
void Path::reverse()
{
    vertices.reverse();
}

// Maps the Path l from the coordinate frame of Pose p to world coordinates.
Path operator+(const Path& l, const Pose2D& p)
{
    Path c = l;
    c += p;
    return c;
}

// Maps the Path l into the coordinate frame of the Pose p.
Path operator-(const Path& l, const Pose2D& p)
{
    Path c = l;
    c -= p;
    return c;
}

// Maps the Path from the frame given by Pose to world coordinates.
// The Path is assumed to be in local in the frame given by Pose and is
// transformed to world coordinates by Pose arithmetic.
void Path::operator+=(const Pose2D& p)
{
    vertices += p;
}

// Maps the Path into the local coordinates of the frame given by Pose.
void Path::operator-=(const Pose2D& p)
{
    vertices -= p;
}

// Returns the last node of the path.
Vec2 &Path::last()
{
    return vertices.last();
}

// Returns the last node of the path.
const Vec2 &Path::last() const
{
    return vertices.last();
}

// Writes the Path into a data stream.
void Path::streamOut(QDataStream &out) const
{
    out << vertices;
}

// Reads the Path from a data stream.
void Path::streamIn(QDataStream &in)
{
    in >> vertices;
}

// Writes the Path into a data stream.
QDataStream& operator<<(QDataStream& out, const Path &o)
{
    o.streamOut(out);
    return out;
}

// Reads the Path from a data stream.
QDataStream& operator>>(QDataStream& in, Path &o)
{
    o.streamIn(in);
    return in;
}

QDebug operator<<(QDebug dbg, const Path &o)
{
    dbg << o.getVertices();
    return dbg;
}

QDebug operator<<(QDebug dbg, const Path* o)
{
    dbg << o->getVertices();
    return dbg;
}
