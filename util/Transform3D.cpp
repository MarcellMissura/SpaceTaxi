#include "Transform3D.h"
#include <immintrin.h>

// This is a 3D transformation matrix implementation that integrates well
// into this framework. The Transform3D class exposes functionalities
// to set the transformation matrix from xyzrpy parameters and to decompose
// the matrix into these parameters. The parameters are defined
// in the TransformParams struct. Rotations are assumed to occur in
// rpy order. Internally, an array holds the matrix in column-major order
// for smooth OpenGL integration.

Transform3D Transform3D::identity;

Transform3D::Transform3D()
{

}

Transform3D::Transform3D(const TransformParams& t)
{
    *this = t;
}

Transform3D::Transform3D(const double* v)
{
    *this = v;
}

// TransformParams to Transform3D assignment operator.
void Transform3D::operator=(const TransformParams& t)
{
    setFromParams(t);
}

// For importing raw buffers. The buffer is copied. The data is expected to be in column major order.
void Transform3D::operator=(const double* v)
{
    at(0,0) = v[0];
    at(1,0) = v[1];
    at(2,0) = v[2];
    at(3,0) = v[3];
    at(0,1) = v[4];
    at(1,1) = v[5];
    at(2,1) = v[6];
    at(3,1) = v[7];
    at(0,2) = v[8];
    at(1,2) = v[9];
    at(2,2) = v[10];
    at(3,2) = v[11];
    at(0,3) = v[12];
    at(1,3) = v[13];
    at(2,3) = v[14];
    at(3,3) = v[15];
}

// Returns a mutable reference to the element of the transformation matrix at position (row, column).
double &Transform3D::at(uint row, uint col)
{
    return M[row+col*4];
}

// Returns the element of the transformation matrix at position (row, column).
double Transform3D::at(uint row, uint col) const
{
    return M[row+col*4];
}

// Returns a mutable reference to the element of the transformation matrix at position (row, column).
double &Transform3D::operator()(uint row, uint col)
{
    return M[row+col*4];
}

// Returns the element of the transformation matrix at position (row, column).
double Transform3D::operator()(uint row, uint col) const
{
    return M[row+col*4];
}

// Computes a transformation from the xyzrpy parameters given in t.
void Transform3D::setFromParams(const TransformParams& t)
{
    setFromParams(t.x, t.y, t.z, t.roll, t.pitch, t.yaw);
}

// Computes a transformation given a plane defined by a normal n
// and a point p. The normal must be normalized! The computed
// transform is what is needed to transform the camera so that
// it would see the given plane correctly as ground plane.
// Only roll, pitch, and z (the z intercept) are computed.
// x,y, and yaw remain zero.
void Transform3D::setFromGroundPlane(const Vec3 &n, const Vec3 &p)
{
    Vec3 up(0,0,1);
    Vec3 axis = n^up;
    axis.normalize();
    double angle = n.angleTo(up);
    double z = (n*-p);

    double c = cos(angle);
    double s = sin(angle);
    double t = 1.0 - c;

    at(0,0) = c + axis.x*axis.x*t;
    at(1,1) = c + axis.y*axis.y*t;
    at(2,2) = c + axis.z*axis.z*t;

    double tmp1 = axis.x*axis.y*t;
    double tmp2 = axis.z*s;
    at(1,0) = tmp1 + tmp2;
    at(0,1) = tmp1 - tmp2;
    tmp1 = axis.x*axis.z*t;
    tmp2 = axis.y*s;
    at(2,0) = tmp1 - tmp2;
    at(0,2) = tmp1 + tmp2;
    tmp1 = axis.y*axis.z*t;
    tmp2 = axis.x*s;
    at(2,1) = tmp1 + tmp2;
    at(1,2) = tmp1 - tmp2;
    at(0,3) = 0;
    at(1,3) = 0;
    at(2,3) = z;
    at(3,0) = 0;
    at(3,1) = 0;
    at(3,2) = 0;
    at(3,3) = 1;
}

// Computes the inverse of the transformation matrix
// such that if w = T*v then v = Tinv*w.
Transform3D Transform3D::inverse() const
{
    double inv[16], det;
    int i;

    inv[0] = M[5]  * M[10] * M[15] -
             M[5]  * M[11] * M[14] -
             M[9]  * M[6]  * M[15] +
             M[9]  * M[7]  * M[14] +
             M[13] * M[6]  * M[11] -
             M[13] * M[7]  * M[10];

    inv[4] = -M[4]  * M[10] * M[15] +
              M[4]  * M[11] * M[14] +
              M[8]  * M[6]  * M[15] -
              M[8]  * M[7]  * M[14] -
              M[12] * M[6]  * M[11] +
              M[12] * M[7]  * M[10];

    inv[8] = M[4]  * M[9] * M[15] -
             M[4]  * M[11] * M[13] -
             M[8]  * M[5] * M[15] +
             M[8]  * M[7] * M[13] +
             M[12] * M[5] * M[11] -
             M[12] * M[7] * M[9];

    inv[12] = -M[4]  * M[9] * M[14] +
               M[4]  * M[10] * M[13] +
               M[8]  * M[5] * M[14] -
               M[8]  * M[6] * M[13] -
               M[12] * M[5] * M[10] +
               M[12] * M[6] * M[9];

    inv[1] = -M[1]  * M[10] * M[15] +
              M[1]  * M[11] * M[14] +
              M[9]  * M[2] * M[15] -
              M[9]  * M[3] * M[14] -
              M[13] * M[2] * M[11] +
              M[13] * M[3] * M[10];

    inv[5] = M[0]  * M[10] * M[15] -
             M[0]  * M[11] * M[14] -
             M[8]  * M[2] * M[15] +
             M[8]  * M[3] * M[14] +
             M[12] * M[2] * M[11] -
             M[12] * M[3] * M[10];

    inv[9] = -M[0]  * M[9] * M[15] +
              M[0]  * M[11] * M[13] +
              M[8]  * M[1] * M[15] -
              M[8]  * M[3] * M[13] -
              M[12] * M[1] * M[11] +
              M[12] * M[3] * M[9];

    inv[13] = M[0]  * M[9] * M[14] -
              M[0]  * M[10] * M[13] -
              M[8]  * M[1] * M[14] +
              M[8]  * M[2] * M[13] +
              M[12] * M[1] * M[10] -
              M[12] * M[2] * M[9];

    inv[2] = M[1]  * M[6] * M[15] -
             M[1]  * M[7] * M[14] -
             M[5]  * M[2] * M[15] +
             M[5]  * M[3] * M[14] +
             M[13] * M[2] * M[7] -
             M[13] * M[3] * M[6];

    inv[6] = -M[0]  * M[6] * M[15] +
              M[0]  * M[7] * M[14] +
              M[4]  * M[2] * M[15] -
              M[4]  * M[3] * M[14] -
              M[12] * M[2] * M[7] +
              M[12] * M[3] * M[6];

    inv[10] = M[0]  * M[5] * M[15] -
              M[0]  * M[7] * M[13] -
              M[4]  * M[1] * M[15] +
              M[4]  * M[3] * M[13] +
              M[12] * M[1] * M[7] -
              M[12] * M[3] * M[5];

    inv[14] = -M[0]  * M[5] * M[14] +
               M[0]  * M[6] * M[13] +
               M[4]  * M[1] * M[14] -
               M[4]  * M[2] * M[13] -
               M[12] * M[1] * M[6] +
               M[12] * M[2] * M[5];

    inv[3] = -M[1] * M[6] * M[11] +
              M[1] * M[7] * M[10] +
              M[5] * M[2] * M[11] -
              M[5] * M[3] * M[10] -
              M[9] * M[2] * M[7] +
              M[9] * M[3] * M[6];

    inv[7] = M[0] * M[6] * M[11] -
             M[0] * M[7] * M[10] -
             M[4] * M[2] * M[11] +
             M[4] * M[3] * M[10] +
             M[8] * M[2] * M[7] -
             M[8] * M[3] * M[6];

    inv[11] = -M[0] * M[5] * M[11] +
               M[0] * M[7] * M[9] +
               M[4] * M[1] * M[11] -
               M[4] * M[3] * M[9] -
               M[8] * M[1] * M[7] +
               M[8] * M[3] * M[5];

    inv[15] = M[0] * M[5] * M[10] -
              M[0] * M[6] * M[9] -
              M[4] * M[1] * M[10] +
              M[4] * M[2] * M[9] +
              M[8] * M[1] * M[6] -
              M[8] * M[2] * M[5];

    det = M[0] * inv[0] + M[1] * inv[4] + M[2] * inv[8] + M[3] * inv[12];

    if (det == 0)
        return Transform3D();

    det = 1.0 / det;

    for (i = 0; i < 16; i++)
        inv[i] = inv[i] * det;

    return inv;
}

// Returns the transposed transformation matrix.
Transform3D Transform3D::transposed() const
{
    Transform3D T;
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            T(i,j) = at(j,i);
    return T;
}

// Returns the translation component of the 3D transformation matrix.
Vec3 Transform3D::translation() const
{
    return Vec3(at(0,3), at(1,3), at(2,3));
}

// Returns the translation component of the 3D transformation matrix.
Vec3 Transform3D::getTranslation() const
{
    return Vec3(at(0,3), at(1,3), at(2,3));
}

void Transform3D::setTranslation(const Vec3 &t)
{
    at(0,3) = t.x; at(1,3) = t.y; at(2,3) = t.z;
}

// Returns the translation component of the 3D transformation matrix.
Vec3 Transform3D::position() const
{
    return Vec3(at(0,3), at(1,3), at(2,3));
}

// Returns the translation component of the 3D transformation matrix.
Vec3 Transform3D::getPosition() const
{
    return Vec3(at(0,3), at(1,3), at(2,3));
}

// Computes the transformation matrix from the xyzrpy parameters.
void Transform3D::setFromParams(double x, double y, double z, double roll, double pitch, double yaw)
{
    // Following http://planning.cs.uiuc.edu/node102.html#eqn:yprmat

    double cy = cos(yaw); // alpha
    double cp = cos(pitch); // beta
    double cr = cos(roll); // gamma
    double sy = sin(yaw);
    double sp = sin(pitch);
    double sr = sin(roll);

    at(0,0) = cy*cp;
    at(0,1) = cy*sp*sr-sy*cr;
    at(0,2) = cy*sp*cr+sy*sr;
    at(0,3) = x;
    at(1,0) = sy*cp;
    at(1,1) = sy*sp*sr+cy*cr;
    at(1,2) = sy*sp*cr-cy*sr;
    at(1,3) = y;
    at(2,0) = -sp;
    at(2,1) = cp*sr;
    at(2,2) = cp*cr;
    at(2,3) = z;
    at(3,0) = 0;
    at(3,1) = 0;
    at(3,2) = 0;
    at(3,3) = 1;
}

// Decomposes the transformation matrix into xyzrpy parameters.
TransformParams Transform3D::getParams() const
{
    // Following the formulas at http://planning.cs.uiuc.edu/node103.html

    TransformParams t;
    t.x = at(0,3);
    t.y = at(1,3);
    t.z = at(2,3);

    double sq = sqrt(at(0,0)*at(0,0)+at(1,0)*at(1,0));
    t.yaw = atan2(at(1,0),at(0,0)); // alpha
    t.pitch = atan2(-at(2,0),sq); // beta
    t.roll = atan2(at(2,1),at(2,2)); // gamma

    return t;
}

// Concatenates two transformations.
Transform3D Transform3D::operator*(const Transform3D &o) const
{
    Transform3D TT;

    for (uint i = 0; i < 4; i++)
    {
        for (uint j = 0; j < 4; j++)
        {
            double sum = 0;
            for (uint k = 0; k < 4; k++)
            {
                sum += at(i,k)*o.at(k,j);
            }
            TT.at(i,j) = sum;
        }
    }

    return TT;
}

// Applies the transform to the vector v.
Vec3 Transform3D::operator*(const Vec3 &v) const
{
    Vec3 out;

    out.x = M[0]*v.x + M[4]*v.y + M[8]*v.z + M[12];
    out.y = M[1]*v.x + M[5]*v.y + M[9]*v.z + M[13];
    out.z = M[2]*v.x + M[6]*v.y + M[10]*v.z + M[14];

    return out;
}

Vector<Vec3> Transform3D::operator*(const Vector<Vec3> &pointBuffer) const
{
    static Vector<Vec3> tmp;

    tmp.resize(pointBuffer.size());
    for (size_t i = 0; i < pointBuffer.size(); ++i) {
        tmp[i] = *this * pointBuffer[i];
    }

    return tmp;
}

// Applies the transform to the vector v.
Vec3 Transform3D::map(const Vec3 &v) const
{
    Vec3 out;

    out.x = M[0]*v.x + M[4]*v.y + M[8]*v.z + M[12];
    out.y = M[1]*v.x + M[5]*v.y + M[9]*v.z + M[13];
    out.z = M[2]*v.x + M[6]*v.y + M[10]*v.z + M[14];

    return out;
}


void Transform3D::streamOut(QDataStream& out) const
{
    for (int i = 0; i < 16; i++)
        out << M[i];
}

void Transform3D::streamIn(QDataStream &in)
{
    for (int i = 0; i < 16; i++)
        in >> M[i];
}

QDebug operator<<(QDebug dbg, const TransformParams &o)
{
    dbg << "r:" << o.roll << "p:" << o.pitch << "y:" << o.yaw << "x:" << o.x << "y:" << o.y << "z:" << o.z;
    return dbg;
}

QDebug operator<<(QDebug dbg, const Transform3D &o)
{
    dbg << "[" << "\n"
        << o.at(0, 0) << o.at(0, 1) << o.at(0, 2) << o.at(0, 3) << "\n "
        << o.at(1, 0) << o.at(1, 1) << o.at(1, 2) << o.at(1, 3) << "\n"
        << o.at(2, 0) << o.at(2, 1) << o.at(2, 2) << o.at(2, 3) << "\n"
        << o.at(3, 0) << o.at(3, 1) << o.at(3, 2) << o.at(3, 3) << "\n"
        "]";
    return dbg;
}

QDataStream& operator<<(QDataStream& out, const Transform3D &o)
{
    o.streamOut(out);
    return out;
}

QDataStream& operator>>(QDataStream& in, Transform3D &o)
{
    o.streamIn(in);
    return in;
}

