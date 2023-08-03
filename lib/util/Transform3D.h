#ifndef TRANSFORM3D_H
#define TRANSFORM3D_H
#include "lib/util/Vec3.h"
#include "lib/util/Vector.h"

struct TransformParams
{
    double x = 0.0, y = 0.0, z = 0.0;
    double roll = 0.0, pitch = 0.0, yaw = 0.0;

    bool operator!=(const TransformParams &o) const
    {
        return (fabs(x-o.x) > EPSILON
                || fabs(y-o.y) > EPSILON
                || fabs(z-o.z) > EPSILON
                || fabs(roll-o.roll) > EPSILON
                || fabs(pitch-o.pitch) > EPSILON
                || fabs(yaw-o.yaw) > EPSILON);
    }
};

class Transform3D
{
    double M[16] = {1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1}; // identity

public:

    Transform3D();
    ~Transform3D(){}

    static Transform3D identity;

    Transform3D(double x, double y, double z, double roll, double pitch, double yaw);
    Transform3D(const TransformParams& t);
    Transform3D(const double* v);
    void operator=(const TransformParams& t);
    void operator=(const double* v);

    double &at(uint row, uint col);
    double at(uint row, uint col) const;
    double& operator()(uint row, uint col);
    double operator()(uint row, uint col) const;

    TransformParams getParams() const;
    void setFromParams(double x, double y, double z, double roll, double pitch, double yaw);
    void setFromParams(const TransformParams &t);
    void setFromGroundPlane(const Vec3& n, const Vec3& p);

    Transform3D inverse() const;
    Transform3D transposed() const;

    Vec3 position() const;
    Vec3 translation() const;
    Vec3 getPosition() const;
    Vec3 getTranslation() const;
    void setTranslation(const Vec3& t);

    Transform3D operator*(const Transform3D& o) const;
    Vec3 operator*(const Vec3& v) const;
    Vector<Vec3> operator*(const Vector<Vec3>& pointBuffer) const;
    Vec3 map(const Vec3& v) const;

    // OpenGL support.
    operator const double*() const {return M;}
    const double* data() const {return M;}
    operator double*() {return M;}
    double* data() {return M;}

    void streamOut(QDataStream& out) const;
    void streamIn(QDataStream &in);
};

QDebug operator<<(QDebug dbg, const TransformParams &o);
QDebug operator<<(QDebug dbg, const Transform3D &o);
QDataStream& operator<<(QDataStream& out, const Transform3D &o);
QDataStream& operator>>(QDataStream& in, Transform3D &o);

#endif
