#include "RGBDSensor.h"
#include <GL/glu.h>
#include <QGLViewer/qglviewer.h>
#include "blackboard/Config.h"

// The RGBDSensor class bundles all things related to the RGB-D sensor we are using
// for perception. The RGBDSensor class contains buffers for the rgb data
// (colorBuffer), which make up the pixels of the camera image, and for the
// 3D point cloud data (pointBuffer). The RGBDSensor class also contains a
// transformation matrix that describes the pose of the camera. The RGBDSensor class
// offers an interface for setting and reading this transformation, and for
// coordinate conversion between world, camera frame, and pixel coordinates.
// The RGBDSensor class also offers an interface to read and write
// the color and point buffers, but this is a bit more complex. Since these buffers
// are large and are written and read in different threads, there is a mutex in
// place that can be used (or not) with one of the following usage patterns:
//
// 1. Data copy
// The easiest and safest way to write and read the buffers are the
//
// writeColorBuffer(const Vector<RGBPixel>& colors)
// writePointBuffer(const Vector<Vec3>& points)
// writeBuffers(const Vector<RGBPixel>& colors, const Vector<Vec3>& points)
// readColorBuffer(Vector<RGBPixel>& colors)
// readPointBuffer(Vector<Vec3>& points)
// readBuffers(Vector<RGBPixel>& colors, Vector<Vec3>& points)
//
// functions where you provide your own copy of the color and point buffers.
// If you call a write* function, the mutex locks and the buffer is overwritten
// with the content of yours after which the mutex is immediately unlocked.
// Programmatically, this is the easiest way of manipulating
// the data, but it comes with the overhead of the extra memory
// allocated in local buffers. There is also a computational
// overhead for copying the data which would not occur if the
// buffers were written into directly, but on the other hand,
// this minimizes the time the RGBDSensor object spends in a locked
// state and that allows other threads to proceed faster.
//
// 2. Locked access
// A slighty faster but unsafe way of *reading* the buffers is to call
//
// const Vector<RGBPixel>& colorBuffer = getColorBuffer();
// const Vector<Vec3>& pointBuffer = getPointBuffer();
//
// The getBuffer methods will lock the mutex in order to wait for write
// operations to finish, but then they return only a reference to the
// data and unlock. This way, multiple threads can read the buffers at
// the same time, but if they take too long and the reading overlaps
// with the next write operations, bad things can happen.

Vector<Vec3> RGBDSensor::unitImage;

RGBDSensor::RGBDSensor()
{
    frameId = 0;
    init();
}

RGBDSensor::~RGBDSensor()
{

}

// Copy constructor.
RGBDSensor::RGBDSensor(const RGBDSensor &o)
{
    *this = o;
}

// Assignment operator.
// It is needed to copy the internal buffers.
// An assignment deep copies a RGBDSensor object.
// The assignment operator needs to exist so that the
// operation can be mutexed.
RGBDSensor& RGBDSensor::operator=(const RGBDSensor &o)
{
    if (this == &o)
        return *this;

    MutexLocker locker(camMutex);
    MutexLocker locker2(transformMutex);

    frameId = o.frameId;
    C = o.C;
    Cinv = o.Cinv;
    pointBuffer = o.pointBuffer;
    colorBuffer = o.colorBuffer;

    return *this;
}

// Initialization after construction.
void RGBDSensor::init()
{
    // There is an issue with the projection of the camera plane
    // onto a canvas with the size of the camera resolution. When the
    // aspect ratio of the camera plane is different from the aspect
    // ratio of the resolution, with a naive corner to corner mapping,
    // the image will look stretched. One way to fix this is to use a
    // larger viewport with a resolution that matches the camera plane
    // aspect ratio and then cutting out an image with the desired camera
    // image resolution. But then there is a mismatch between the area
    // of the camera plane and the camera image. Perhaps a better way is
    // to enforce the same aspect ratio for the camera plane and the
    // resolution. This means that the vertical opening angle is ignored.
    // The horizontal size of the camera plane (unitImage) is computed
    // from the horizontal opening angle parameter and the vertical size
    // of the camera plane is inferred from the resolution aspect ratio.
    // I found that the opening angles of the RealSense D415 have an
    // aspect ratio very close to the resolution aspect ratio and so
    // the vertical opening angle can be safely ignored.

    // Compute the unit image, which is a pixelwise representation of the camera plane.
    // The unit image is used to determine the frustum and for the conversion of depth to point data.
    double aspectRatio = (double)cameraInfo.width/cameraInfo.height;
    unitImage.resize(cameraInfo.width * cameraInfo.height);
    double cameraOpeningAngleHorizontal = 2.0 * atan(cameraInfo.width / (2.0 * cameraInfo.fx));
    double halfWidth = CAMERA_PLANE_DISTANCE * tan(0.5 * cameraOpeningAngleHorizontal);
    double halfHeight = halfWidth / aspectRatio;
    for (int i = 0; i < cameraInfo.height; i++)
    {
        for (int j = 0; j < cameraInfo.width; j++)
        {
            unitImage[i*cameraInfo.width+j].x = CAMERA_PLANE_DISTANCE;
            unitImage[i*cameraInfo.width+j].y = halfWidth - j*2.0*halfWidth/(cameraInfo.width-1.0);
            unitImage[i*cameraInfo.width+j].z = halfHeight - i*2.0*halfHeight/(cameraInfo.height-1.0);
        }
    }

    //qDebug() << "Camera plane aspect ratio:" << unitImage[0].y/unitImage[0].z << "unit:" << unitImage[0];
    //qDebug() << "Opening angle aspect ratio:" << config.cameraOpeningAngleHorizontal/config.cameraOpeningAngleVertical << config.cameraOpeningAngleHorizontal << "x" << config.cameraOpeningAngleVertical;
    //qDebug() << "Resolution aspect ratio:" << (double)CAMERA_RESOLUTION_WIDTH/CAMERA_RESOLUTION_HEIGHT << CAMERA_RESOLUTION_WIDTH << "x" << CAMERA_RESOLUTION_HEIGHT;

    pointBuffer.resize(cameraInfo.width*cameraInfo.height);
    colorBuffer.resize(cameraInfo.width*cameraInfo.height);

    return;
}

const CameraInfo &RGBDSensor::getCameraInfo() const
{
    return cameraInfo;
}

void RGBDSensor::setCameraInfo(const CameraInfo &info)
{
    MutexLocker locker(camMutex);
    cameraInfo = info;
    init();
}

// Returns the aspect ratio (horizontal / vertical) of the camera plane.
// This figure is influenced by the horizontal and vertical opening
// angles and the camera plane distance.
double RGBDSensor::getCameraPlaneAspectRatio() const
{
    return unitImage[0].y/unitImage[0].z;
}

// Returns the aspect ratio of the pixel resolution of the camera (width/height).
double RGBDSensor::getResolutionAspectRatio() const
{
    return (double)cameraInfo.width/cameraInfo.height;
}

// Returns the currently set camera transform.
const Transform3D &RGBDSensor::getTransform() const
{
    MutexLocker locker(transformMutex);
    return C;
}

// Returns the inverse of the currently set camera transform.
const Transform3D &RGBDSensor::getTransformInverse() const
{
    MutexLocker locker(transformMutex);
    return Cinv;
}

// Sets the camera transform.
void RGBDSensor::setTransform(const Transform3D &T)
{
    MutexLocker locker(transformMutex);
    C = T;
    Cinv = C.inverse();
    return;
}

// Sets the camera transform from xyzrpy parameters.
void RGBDSensor::setTransform(const TransformParams &tp)
{
    MutexLocker locker(transformMutex);
    C.setFromParams(tp);
    Cinv = C.inverse();
    return;
}

// Locks the mutex and overwrites the color buffer with the rgb pixels found in colors.
void RGBDSensor::writeColorBuffer(const Vector<RGBPixel> &colors)
{
    MutexLocker locker(camMutex);
    colorBuffer = colors;
    frameId++;
    return;
}

// Locks the mutex and overwrites the point buffer with the points found in points.
void RGBDSensor::writePointBuffer(const Vector<Vec3> &points)
{
    MutexLocker locker(camMutex);
    pointBuffer = points;
    return;
}

// Convenience method for locking the sensor just once and writing both buffers at the same time.
void RGBDSensor::writeBuffers(const Vector<RGBPixel>& colors, const Vector<Vec3>& points)
{
    MutexLocker locker(camMutex);
    colorBuffer = colors;
    pointBuffer = points;
    frameId++;
    return;
}

// Locks the mutex and overwrites the color buffer colors with the rgb pixels in this colorBuffer.
void RGBDSensor::readColorBuffer(Vector<RGBPixel>& colors) const
{
    MutexLocker locker(camMutex);
    colors = colorBuffer;
    return;
}

// Locks the mutex and overwrites the point buffer points with the points in this pointBuffer.
void RGBDSensor::readPointBuffer(Vector<Vec3> &points) const
{
    MutexLocker locker(camMutex);
    points = pointBuffer;
    return;
}

// Convenience method for locking the sensor just once and reading both buffers at the same time.
void RGBDSensor::readBuffers(Vector<RGBPixel> &colors, Vector<Vec3> &points) const
{
    MutexLocker locker(camMutex);
    colors = colorBuffer;
    points = pointBuffer;
    return;
}

// Waits for write operations to finish and then returns an unmutexed read only
// reference to the internal color buffer. Needless to say, this is not an entirely
// thread sage way of accessing the data. The use case is that after writing
// the buffer, multiple threads can have read access to the buffer at the same
// time. However, if they take too long with their reading, they might overlap
// the next write operation.
const Vector<RGBPixel>& RGBDSensor::getColorBuffer() const
{
    MutexLocker locker(camMutex);
    return colorBuffer;
}

// Waits for write operations to finish and then returns an unmutexed read only
// reference to the internal point buffer. Needless to say, this is not an entirely
// thread sage way of accessing the data. The use case is that after writing
// the buffer, multiple threads can have read access to the buffer at the same
// time. However, if they take too long with their reading, they might overlap
// the next write operation.
const Vector<Vec3>& RGBDSensor::getPointBuffer() const
{
    MutexLocker locker(camMutex);
    return pointBuffer;
}

// Waits for write operations to finish and then returns a copy of the point buffer
// transformed by the camera transform C.
Vector<Vec3> RGBDSensor::getTransformedPointBuffer() const
{
    MutexLocker locker(camMutex);
    return C * pointBuffer;
}

// Transforms a point given in world coordinates to camera frame coordinates.
Vec3 RGBDSensor::fromWorldToCameraCoordinates(const Vec3 &v) const
{
    return Cinv*v;
}

// Transforms a point given in camera frame coordinates to world coordinates.
Vec3 RGBDSensor::fromCameraToWorldCoordinates(const Vec3 &v) const
{
    return C*v;
}

// Transforms a point given in camera frame coordinates to pixel coordinates.
// Note that the pixel coordinates might be outside of the image.
QPoint RGBDSensor::fromCameraToImageCoordinates(const Vec3 &v) const
{
    //qDebug() << "Converting:" << v;
    Vec3 p = v;
    p.z = CAMERA_PLANE_DISTANCE*p.z/p.x;
    p.y = CAMERA_PLANE_DISTANCE*p.y/p.x;
    p.x = CAMERA_PLANE_DISTANCE;
    QPoint qp(
        0.5*cameraInfo.width*(1.0 - p.y/unitImage[0].y), // Using the tl pixel of the unit image.
        0.5*cameraInfo.height*(1.0 - p.z/unitImage[0].z)
    );

    //qDebug() << p << unitImage[0] << qp;

    return qp;
}

// Transforms a point given in world coordinates to pixel coordinates.
// Note that the pixel coordinates might be outside of the image.
QPoint RGBDSensor::fromWorldToImageCoordinates(const Vec3 &v) const
{
    return fromCameraToImageCoordinates(fromWorldToCameraCoordinates(v));
}

// Converts from image coordinates (i,j) to world coordinates.
// (i,j) is the pixel coordinate where (0,0) is the top left
// corner of the image, i is the ith row and j is the jth column.
// The returned world coordinate is the floor intersection of the
// unit ray (i,j). This can be behind the camera plane if the
// camera is looking up, or undefined if the ray is parallel to
// the ground.
Vec3 RGBDSensor::fromImageToWorldCoordinates(uint j, uint i) const
{
    const Vec3& p = unitImage[i*cameraInfo.width+j]/CAMERA_PLANE_DISTANCE;
    Vec3 cp = C*p;
    Vec3 dcp = cp - C.position();
    double a = C.position().z / dcp.z;
    Vec3 out = C.position() - a * dcp;
    //qDebug() << j << i << "p:" << p << "cp:" << cp << "pos:" << C.position() << "dcp:" << dcp << "a:" << a << "out:" << out;
    return out;
}

// Draws the camera frame, the camera plane, and the frustum in an OpenGL context.
void RGBDSensor::drawCameraPlane() const
{
    // Compute the tl, tr, bl, br corners of the image plane.
    Vec3 tlNear = unitImage[0]/CAMERA_PLANE_DISTANCE;
    Vec3 trNear = unitImage[cameraInfo.width-1]/CAMERA_PLANE_DISTANCE;
    Vec3 blNear = unitImage[cameraInfo.width*cameraInfo.height-cameraInfo.width]/CAMERA_PLANE_DISTANCE;
    Vec3 brNear = unitImage[cameraInfo.width*cameraInfo.height-1]/CAMERA_PLANE_DISTANCE;

    // Transform into the camera frame.
    glPushMatrix();
    transformMutex.lock();
    glMultMatrixd(C);
    transformMutex.unlock();

    // Draw a little coordinate system for the camera.
    QGLViewer::drawAxis(0.25);

    // Scale the camera plane down a little.
    glScaled(0.6, 0.6, 0.6);

    // Set up a texture used for the camera image display.
    // and load the camera image into the texture.
    static bool textureGenerated = false;
    if (!textureGenerated)
    {
        GLuint textureID;
        glGenTextures(1, &textureID);
        glBindTexture(GL_TEXTURE_2D, textureID);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_GENERATE_MIPMAP, GL_TRUE);
        glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
        textureGenerated = true;
    }
    camMutex.lock();
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, cameraInfo.width, cameraInfo.height, 0, GL_RGB, GL_UNSIGNED_BYTE, colorBuffer.data());
    camMutex.unlock();

    // Draw the camera image texture.
    glEnable(GL_TEXTURE_2D);
    glBegin(GL_QUADS);
    glTexCoord2f(0.0, 0.0); glVertex3dv(tlNear);
    glTexCoord2f(0.0, 1.0); glVertex3dv(blNear);
    glTexCoord2f(1.0, 1.0); glVertex3dv(brNear);
    glTexCoord2f(1.0, 0.0); glVertex3dv(trNear);
    glEnd();
    glDisable(GL_TEXTURE_2D);

    // Draw the frustum.

    glLineWidth(2);
    glColor4d(0.0, 0.5, 0.0, 0.4);

    glBegin(GL_LINE_LOOP);
    glVertex3d(0,0,0);
    glVertex3dv(blNear);
    glVertex3dv(brNear);
    glEnd();

    glBegin(GL_LINE_LOOP);
    glVertex3d(0,0,0);
    glVertex3dv(blNear);
    glVertex3dv(tlNear);
    glEnd();

    glBegin(GL_LINE_LOOP);
    glVertex3d(0,0,0);
    glVertex3dv(tlNear);
    glVertex3dv(trNear);
    glEnd();

    glBegin(GL_LINE_LOOP);
    glVertex3d(0,0,0);
    glVertex3dv(brNear);
    glVertex3dv(trNear);
    glEnd();

    glPopMatrix();
}

// Draws the camera frame, the camera plane, and the frustum in an OpenGL context.
void RGBDSensor::drawPointCloud(double floor, double ceiling) const
{
    if (pointBuffer.empty() || colorBuffer.empty())
        return;

    //glPushMatrix();
    //glMultMatrixd(C);

    glPointSize(3);
    glBegin(GL_POINTS);

    for (uint i = 0; i < pointBuffer.size(); i++)
    {
        if (pointBuffer[i].isNan())
            continue;

        const Vec3 &p = C * pointBuffer[i];
        if (floor != 0 || ceiling != 0)
            if (p.z < floor || p.z > ceiling)
                continue;
        glColor3ubv(colorBuffer[i]);
        glVertex3dv(p);
    }

    glEnd();
    //glPopMatrix();
}

// Computes the OpenGL projection matrix given the opening angle and the camera plane
// distance parameters. The camera plane is treated as the near plane. The distance of
// the far plane has to be provided as a parameter.
Transform3D RGBDSensor::frustum()
{
    double near = CAMERA_PLANE_DISTANCE;
    double far = config.sceneRadius; // The same "far" must be used to convert the depth buffer value back to z.
    double top = unitImage[0].z;
    double bottom = -top;
    double right = unitImage[0].y;
    double left = -right;

    Transform3D M;

    M.at(0,0) = 2 * near / (right - left);
    M.at(1,0) = 0;
    M.at(2,0) = 0;
    M.at(3,0) = 0;

    M.at(0,1) = 0;
    M.at(1,1) = 2 * near / (top - bottom);
    M.at(2,1) = 0;
    M.at(3,1) = 0;

    M.at(0,2) = (right + left) / (right - left);
    M.at(1,2) = (top + bottom) / (top - bottom);
    M.at(2,2) = -(far + near) / (far - near);
    M.at(3,2) = -1;

    M.at(0,3) = 0;
    M.at(1,3) = 0;
    M.at(2,3) = -2 * far * near / (far - near);
    M.at(3,3) = 0;

    return M;
}

void RGBDSensor::streamOut(QDataStream& out) const
{
    MutexLocker locker(camMutex);
    out << frameId;
    out << C;
    out << pointBuffer;
    out << colorBuffer;
}

void RGBDSensor::streamIn(QDataStream &in)
{
    MutexLocker locker(camMutex);
    in >> frameId;
    in >> C;
    Cinv = C.inverse();
    in >> pointBuffer;
    in >> colorBuffer;
}

QDataStream& operator<<(QDataStream& out, const RGBDSensor &o)
{
    o.streamOut(out);
    return out;
}

QDataStream& operator>>(QDataStream& in, RGBDSensor &o)
{
    o.streamIn(in);
    return in;
}

QDebug operator<<(QDebug dbg, const RGBDSensor &o)
{
    dbg << o.getTransform();
    return dbg;
}
