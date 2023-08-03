#ifndef CAMERA_H_
#define CAMERA_H_
#include "lib/util/Transform3D.h"
#include "lib/util/Vector.h"
#include "lib/util/RGBPixel.h"
#include "lib/util/Mutex.h"
#include <QPoint>

const double CAMERA_PLANE_DISTANCE = 0.1;
const double CAMERA_OPENING_ANGLE_HORIZONTAL = 69.3;
const double CAMERA_OPENING_ANGLE_VERTICAL = 42.3;

struct CameraInfo
{
    unsigned int width, height; // Width and height of the image in pixels.
    double cx, cy, fx, fy, Tx, Ty; // Internal rectified camera parameters.

    CameraInfo()
    {
        fx = 533.916109993337;
        fy = 533.5469165200249;
        cx = 317.6107153070701;
        cy = 241.0347253606744;
        height = 480;
        width = 640;
    }
};

class RGBDSensor
{
    uint frameId;
    static Vector<Vec3> unitImage;
    CameraInfo cameraInfo;

    // The camera transformation matrix C that converts local points to world coordinates, and its inverse.
    Transform3D C;
    Transform3D Cinv;

    // Allocated buffer for the point cloud.
    Vector<Vec3> pointBuffer;

    // Allocated buffer for the color pixels.
    Vector<RGBPixel> colorBuffer;

    // Mutexes for thread safety.
    mutable Mutex camMutex;
    mutable Mutex transformMutex;

public:

    RGBDSensor();
    ~RGBDSensor();

    RGBDSensor(const RGBDSensor &o);
    RGBDSensor& operator=(const RGBDSensor &o);

    void init();

    const CameraInfo& getCameraInfo() const;
    void setCameraInfo(const CameraInfo& info);

    double getCameraPlaneAspectRatio() const;
    double getResolutionAspectRatio() const;

    const Transform3D& getTransform() const;
    const Transform3D& getTransformInverse() const;
    void setTransform(const Transform3D& T);
    void setTransform(const TransformParams& tp);
    Vec3 fromWorldToCameraCoordinates(const Vec3& v) const;
    Vec3 fromCameraToWorldCoordinates(const Vec3& v) const;
    QPoint fromCameraToImageCoordinates(const Vec3& v) const;
    QPoint fromWorldToImageCoordinates(const Vec3& v) const;
    Vec3 fromImageToWorldCoordinates(uint j, uint i) const;

    void writeColorBuffer(const Vector<RGBPixel>& colors);
    void writePointBuffer(const Vector<Vec3>& points);
    void writeBuffers(const Vector<RGBPixel>& colors, const Vector<Vec3>& points);
    void readColorBuffer(Vector<RGBPixel> &colors) const;
    void readPointBuffer(Vector<Vec3>& points) const;
    void readBuffers(Vector<RGBPixel>& colors, Vector<Vec3> &points) const;
    const Vector<RGBPixel>& getColorBuffer() const;
    const Vector<Vec3>& getPointBuffer() const;
    Vector<Vec3> getTransformedPointBuffer() const;

    Transform3D frustum();

    void drawCameraPlane() const;
    void drawPointCloud(double floor=0, double ceiling=0) const;

    void streamOut(QDataStream& out) const;
    void streamIn(QDataStream &in);

};

QDataStream& operator<<(QDataStream& out, const RGBDSensor &o);
QDataStream& operator>>(QDataStream& in, RGBDSensor &o);
QDebug operator<<(QDebug dbg, const RGBDSensor &o);

#endif
