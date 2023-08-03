#include "Box3d.h"
#include "util/ColorUtil.h"
#include "blackboard/Config.h"
#include "util/GLlib.h"
#include "util/Transform3D.h"
#include <GL/gl.h>

// The Box3d is an axis aligned cuboid typically used as a bounding box
// of a segment. A Transform3D defines its position and orientation in
// 3D space while the half extents dx, dy, and dz define the size of the
// box along each dimension.
// Most importantly, Box3d implements intersect() methods for collision
// checking with a number of geometric primitives.

Box3d::Box3d()
{
    dx = 0;
    dy = 0;
    dz = 0;
}

// Sets the box by providing the center point p and the half extent vectors x, y, and z.
void Box3d::set(const Vec3 &p, const Vec3 &x, const Vec3 &y, const Vec3 &z)
{
    this->p = p;
    dx = x.norm();
    dy = y.norm();
    dz = z.norm();
    nx = x.normalized();
    ny = y.normalized();
    nz = z.normalized();
}

// Sets the world (x,y,z) position of the Box3d.
void Box3d::setPos(double x, double y, double z)
{
    p.x = x;
    p.y = y;
    p.z = z;
}

// Sets the world (x,y,z) position of the Box3d.
void Box3d::setPos(const Vec3 &p)
{
    this->p = p;
}

// Returns the world (x,y,z) position of the Box3d.
Vec3 Box3d::pos() const
{
    return p;
}

// Translates the Box3d by (dx,dy,dz).
void Box3d::translate(double dx, double dy, double dz)
{
    p.x += dx;
    p.y += dy;
    p.z += dz;
}

// Translates the Box3d by dp.
void Box3d::translate(const Vec3 &dp)
{
    p += dp;
}

// Applies the transformation T to the Box3d.
void Box3d::transform(const Transform3D &T)
{
    p = T*p;
    nx = T*nx - T.position();
    ny = T*ny - T.position();
    nz = T*nz - T.position();
}

// Returns the vertices of all corners of this box.
Vector<Vec3> Box3d::getVertices() const
{
    Vector<Vec3> vertices;
    vertices << dx*nx + dy*ny + dz*nz + p;
    vertices << -dx*nx + dy*ny + dz*nz + p;
    vertices << dx*nx + -dy*ny + dz*nz + p;
    vertices << -dx*nx + -dy*ny + dz*nz + p;
    vertices << dx*nx + dy*ny + -dz*nz + p;
    vertices << -dx*nx + dy*ny + -dz*nz + p;
    vertices << dx*nx + -dy*ny + -dz*nz + p;
    vertices << -dx*nx + -dy*ny + -dz*nz + p;
    return vertices;
}

// Returns the normalized normals of the faces of the box.
Vector<Vec3> Box3d::getFaceNormals() const
{
    Vector<Vec3> normals;
    normals << nx << ny << nz;
    return normals;
}

// Returns the extents of the box along the x, y, and z axes.
Vector<double> Box3d::getExtents() const
{
    Vector<double> extents;
    extents << dx << dy << dz;
    return extents;
}

// Returns the front (x) left (y) top (z) vertex.
Vec3 Box3d::getFrontLeftTop() const
{
    return p + dx*nx + dy*ny + dz*nz;
}

// Returns the front (x) left (y) top (z) vertex.
Vec3 Box3d::getFrontLeftBottom() const
{
    return p + dx*nx + dy*ny - dz*nz;
}

// Returns the front (x) left (y) top (z) vertex.
Vec3 Box3d::getFrontRightTop() const
{
    return p + dx*nx - dy*ny + dz*nz;
}

// Returns the front (x) left (y) top (z) vertex.
Vec3 Box3d::getFrontRightBottom() const
{
    return p + dx*nx - dy*ny - dz*nz;
}

// Returns the front (x) left (y) top (z) vertex.
Vec3 Box3d::getBackLeftTop() const
{
    return p - dx*nx + dy*ny + dz*nz;
}

// Returns the front (x) left (y) top (z) vertex.
Vec3 Box3d::getBackLeftBottom() const
{
    return p - dx*nx + dy*ny - dz*nz;
}

// Returns the front (x) left (y) top (z) vertex.
Vec3 Box3d::getBackRightTop() const
{
    return p - dx*nx - dy*ny + dz*nz;
}

// Returns the front (x) left (y) top (z) vertex.
Vec3 Box3d::getBackRightBottom() const
{
    return p - dx*nx - dy*ny - dz*nz;
}

// Returns true if the Box3d intersects with (contains) the point p.
bool Box3d::intersects(const Vec3 &p) const
{
    // The point-normal representation of the Box3D was chosen to optimize this method.

    Vec3 pp = p - this->p;
    return !(pp*nx > dx || pp*nx < -dx || pp*ny > dy || pp*ny < -dy || pp*nz > dz || pp*nz < -dz);
}

// Returns true if the "o" Box3d intersects with this one.
bool Box3d::intersects(const Box3d &o) const
{
    // For each face of this box, check if all vertices of the other box lie on the outside.
    Vector<Vec3> vertices = o.getVertices();
    if (allVerticesOnOneSide(vertices))
        return false;

    // For each face of the other box, check if all vertices of this box lie on the outside.
    vertices = getVertices();
    if (o.allVerticesOnOneSide(vertices))
        return false;

    // If no such face could be found, the boxes must intersect.

    return true;
}

// Checks if all vertices lie on one side of the box, i.e. if one face of the box is a separating plane.
bool Box3d::allVerticesOnOneSide(const Vector<Vec3> &vertices) const
{
    // For each face of this box, check if all given vertices lie on the outside of the face.

    bool allOutside = true;
    for (uint i = 0; i < vertices.size(); i++)
    {
        if ((vertices[i]-p)*nx < dx)
        {
            allOutside = false;
            break;
        }
    }

    if (allOutside)
        return true;

    allOutside = true;
    for (uint i = 0; i < vertices.size(); i++)
    {
        if ((vertices[i]-p)*nx > -dx)
        {
            allOutside = false;
            break;
        }
    }

    if (allOutside)
        return true;


    allOutside = true;
    for (uint i = 0; i < vertices.size(); i++)
    {
        if ((vertices[i]-p)*ny < dy)
        {
            allOutside = false;
            break;
        }
    }

    if (allOutside)
        return true;

    allOutside = true;
    for (uint i = 0; i < vertices.size(); i++)
    {
        if ((vertices[i]-p)*ny > -dy)
        {
            allOutside = false;
            break;
        }
    }

    if (allOutside)
        return true;


    allOutside = true;
    for (uint i = 0; i < vertices.size(); i++)
    {
        if ((vertices[i]-p)*nz < dz)
        {
            allOutside = false;
            break;
        }
    }

    if (allOutside)
        return true;

    allOutside = true;
    for (uint i = 0; i < vertices.size(); i++)
    {
        if ((vertices[i]-p)*nz > -dz)
        {
            allOutside = false;
            break;
        }
    }

    if (allOutside)
        return true;

    return false;
}

// Returns true if the bounding Box3d of the line l intersects this Box3d.
bool Box3d::intersects(const Line &l) const
{
    return false;
}

// Returns true if the plane given by point p and normal n intersects the box.
bool Box3d::intersects(const Vec3 &p, const Vec3 &n) const
{
    Vector<Vec3> vertices = getVertices(); // horribly slow

    double sp = (vertices[0]-p)*n;

    for (uint i = 1; i < vertices.size(); i++)
    {
        if ((vertices[i]-p)*n * sp < 0)
        {
            return true;
        }
    }

    return false;
}

// Draws the Box3d in an OpenGL context.
void Box3d::draw(double r, double g, double b, double a) const
{
    Transform3D T;
    T(0, 0) = nx.x;
    T(1, 0) = nx.y;
    T(2, 0) = nx.z;
    T(3, 0) = 0;

    T(0, 1) = ny.x;
    T(1, 1) = ny.y;
    T(2, 1) = ny.z;
    T(3, 1) = 0;

    T(0, 2) = nz.x;
    T(1, 2) = nz.y;
    T(2, 2) = nz.z;
    T(3, 2) = 0;

    T(0, 3) = p.x;
    T(1, 3) = p.y;
    T(2, 3) = p.z;
    T(3, 3) = 1;

    glPushMatrix();
    glMultMatrixd(T);
    if (a > EPSILON)
    {
        glColor4d(r, g, b, a);
        GLlib::drawBox(dx, dy, dz);
    }
    glColor3d(0.0, 0.0, 0.0);
    glLineWidth(1);
    GLlib::drawWireFrame(dx+0.0001, dy+0.0001, dz+0.0001);
    glPopMatrix();
}

void Box3d::streamOut(QDataStream& out) const
{
    out << dx;
    out << dy;
    out << dz;
    //out << T;
}

void Box3d::streamIn(QDataStream& in)
{
    in >> dx;
    in >> dy;
    in >> dz;
    //in >> T;
}

QDebug operator<<(QDebug dbg, const Box3d &o)
{
    dbg << "p:" << o.pos() << "n:" << o.getFaceNormals();
    return dbg;
}
