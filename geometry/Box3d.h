#ifndef BOX3D_H_
#define BOX3D_H_
#include "util/Vec3.h"
#include "util/Vector.h"
#include "geometry/Line.h"
#include "util/Transform3D.h"

// This is a 3D oriented box class used for the purpose of bounding box tests.
// The box is defined by its center point p and its half extent vectors dx, dy, and dz.
// The box offers an interface for fast intersection with geometric primitives.

class Box3d
{
protected:

    // The Box3d is encoded in point-normal form. One point p defines the coordinates
    // of the reference point of the box. Normalized normal vectors nx, ny, and nz
    // describe the orientation of the box, i.e., the normals have to be orthogonal.
    // The dx, dy, dz parameters define the extents of the box along the normals.
    // Storing the parameters in this form makes it easy (and fast) to determine if a
    // point is inside the box by using the scalar product for axis projections.
    Vec3 p, nx, ny, nz;
    double dx,dy,dz;

public:

    Box3d();
    ~Box3d(){}

    void set(const Vec3& p, const Vec3& x, const Vec3& y, const Vec3& z);

    Vec3 pos() const;
    void setPos(const Vec3& p);
    void setPos(double x, double y, double z);
    void translate(double dx, double dy, double dz);
    void translate(const Vec3& dp);
    void transform(const Transform3D& T);

    Vector<Vec3> getVertices() const;
    Vector<Vec3> getFaceNormals() const;
    Vector<double> getExtents() const;

    Vec3 getFrontLeftTop() const;
    Vec3 getFrontLeftBottom() const;
    Vec3 getFrontRightTop() const;
    Vec3 getFrontRightBottom() const;
    Vec3 getBackLeftTop() const;
    Vec3 getBackLeftBottom() const;
    Vec3 getBackRightTop() const;
    Vec3 getBackRightBottom() const;

    bool intersects(const Box3d &o) const;
    bool intersects(const Vec3 &p) const;
    bool intersects(const Line &l) const;
    bool intersects(const Vec3 &p, const Vec3 &n) const;

    void draw(double r=1.0, double g=0.0, double b=0.0, double a=0.0) const;

    void streamOut(QDataStream& out) const;
    void streamIn(QDataStream& in);

private:
    bool allVerticesOnOneSide(const Vector<Vec3>& vertices) const;
};

QDebug operator<<(QDebug dbg, const Box3d &o);

#endif // Box_H
