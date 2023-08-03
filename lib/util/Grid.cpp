#include "Grid.h"
#include "lib/util/Statistics.h"
#include "lib/util/ColorUtil.h"
#include "lib/util/Vec3.h"
#include <QFile>
#include <GL/glu.h>

// This is a generic DIM dimensional uniform grid construction class.
// Grid nodes are evenly distributed along each axis of the bounded
// input space such that the first node is located at the minimum and
// the last node is located at the maximum. To use the Grid, first
// provide the grid parameters using setDim(), setN(), setMin(), and
// setMax(). Then, call init()! It is crucial that you call
// init() after all parameters have been provided and before you
// make use of the Grid in any way. Example:
//
// Grid grid; // Construct a grid object.
// grid.setDim(3); // Set the number of dimensions.
// grid.setN(Vec3u(101,201,301)); // Set the numer of nodes per dimension.
// grid.setMin(Vec3(config.xMin, config.yMin, config.zMin)); // Set the minimum values per dimension.
// grid.setMax(Vec3(config.xMax, config.yMax, config.zMax)); // Set the maximum values per dimension.
// grid.init(); // Compute the grid representation.
//
// Now you can access the grid node coordinates using either a DIM
// dimensional unsigned integer index (dim index), or a one dimensional
// unsigned integer index (flat index) between 0 and nodeCount that
// enummerates all grid nodes. Example:
//
// for (uint n=0; n < grid.getNodeCount(); n++)
// {
//    Vec3 coordinates = grid.getNodeCoordinates(n); // Use a flat index to look up the coordinates of a grid node.
//    Vec3u idx = grid.convertIndex(n); // Convert the flat index to a dim index.
//    Vec3 sameCoordinates = grid.getNodeCoordinates(idx); // Use the dim index to look up the coordinates of a grid node.
// }
//
// A number of other nifty methods allow you to convert between the dim
// and the flat index representations, query the nearest node index of
// any point on the grid, retrieve all enveloping nodes of a query point,
// retrieve the neighborhood of a grid node, and to save and load the
// grid to and from a binary file.
//
// When setting up the grid, heap memory needs to be allocated (setDim()
// and init()) and this is not a real time capable operation. Grid
// construction should happen outside of time critical loops.

Grid::Grid()
{
    nodeCount = 0;
}

// Sets the number of dimensions of the grid.
void Grid::setDim(uint d)
{
    DIM = d;
    N.resize(DIM);
    max.resize(DIM);
    min.resize(DIM);
    raster.resize(DIM);
    cumN.resize(DIM);
    stride.resize(DIM);
    strideinv.resize(DIM);
    _idx.resize(DIM);
    _iidx.resize(DIM);
    _x.resize(DIM);
}

// Returns DIM, the number of dimensions.
uint Grid::getDim() const
{
    return DIM;
}

// Sets the min boundaries of the DIM dimensional data range.
// The argument double will be set as minimum for all dimensions.
void Grid::setMin(double minn)
{
    for (uint d = 0; d < DIM; d++)
        min[d] = minn;
}

// Sets the max boundaries of the DIM dimensional data range.
// The argument double will be set as maximum for all dimensions.
void Grid::setMax(double maxx)
{
    for (uint d = 0; d < DIM; d++)
        max[d] = maxx;
}

// Sets the min boundaries of the DIM dimensional data range.
// It is your responsibility to make sure the pointer passed as the argument
// points to a valid vector of parameters of size DIM. The util/Vec* classes
// can be used for convenience. setMin(Vec3(m1,m2,m3))
void Grid::setMin(const double *minn)
{
    for (uint d = 0; d < DIM; d++)
        min[d] = minn[d];
}

// Sets the max boundaries of the DIM dimensional data range.
// It is your responsibility to make sure the pointer passed as the argument
// points to a valid vector of parameters of size DIM. The util/Vec* classes
// can be used for convenience. setMax(Vec3(m1,m2,m3))
void Grid::setMax(const double *maxx)
{
    for (uint d = 0; d < DIM; d++)
        max[d] = maxx[d];
}

// Returns the min data range boundaries.
// The returned double* automatically converts to the util/Vec* classes.
// Vec3 min = grid.getMin();
const double* Grid::getMin() const
{
    return min.data();
}

// Returns the max data range boundaries.
// The returned double* automatically converts to the util/Vec* classes.
// Vec3 min = grid.getMin();
const double* Grid::getMax() const
{
    return max.data();
}

// Sets N, the number of nodes per dimension. This method sets N to be the same
// for every dimension and thus creates a uniform grid.
void Grid::setN(uint N_)
{
    uint N__[DIM];
    for (uint d=0; d < DIM; d++)
        N__[d] = N_;
    setN(N__);
}

// Sets N, the number of nodes per dimension. This method sets a different N for
// each individual dimension.
// grid.setN(Vec3u(11,21,31));
void Grid::setN(const uint* N_)
{
    for (uint d=0; d < DIM; d++)
        N[d] = N_[d];
}

// Returns the number of nodes per dimension. The size of the vector is DIM, of course.
// It converts nicely to a util/Vec* class. Vec3u N = grid.getN();
const uint* Grid::getN() const
{
    return N.data();
}

// Calculates the raster of the grid coordinates. The grid nodes are distributed between the
// respective min and max values of each dimension such that the first node is located at the
// min and the last node is located at the max. Dim, N, min, and max must be set before
// computing the raster. Make sure to set the grid parameters first and then call this method
// to prepare the grid before using it. Some of the methods of this class will segfault if init
// has not been called. You have been warned.
void Grid::init()
{
    nodeCount = N[0];
    for (uint d = 1; d < DIM; d++)
        nodeCount *= N[d];

    // Accumulate the number of nodes per dimension to compute a "stride" in index space.
    // This speeds up index coversions.
    cumN[0] = 1;
    for (uint d = 1; d < DIM; d++)
        cumN[d] = cumN[d-1]*N[d-1];

    // Compute the stride and the raster in the grid space.
    for (uint d = 0; d < DIM; d++)
    {
        double l = max[d]-min[d];
        stride[d] = l/(N[d]-1);
        strideinv[d] = 1.0/stride[d];
        raster[d].resize(N[d]);
        for (uint n = 0; n < N[d]; n++)
            raster[d][n] = min[d]+n*stride[d];
    }
}

// Returns a pointer to a DIM sized vector that contains the strides for each dimension.
// The stride is the size of a cell in the respective dimension.
// Vec3 strides = grid.getStride();
const double* Grid::getStride() const
{
    return stride.data();
}

// Returns the total number of Grid nodes.
uint Grid::getNodeCount() const
{
    return nodeCount;
}

// Converts a DIM dimensional index to a flat index.
// uint flatIdx = grid.convertIndex(dimIdx);
uint Grid::convertIndex(const uint *idx) const
{
    uint k = idx[0];
    for (uint d = 1; d < DIM; d++)
        k += idx[d]*cumN[d];
    return k;
}

// Converts a flat index to a DIM dimensional index.
// Invalidates all dim index references returned so far.
// Vec3u dimIdx = grid.convertIndex(flatIdx);
const uint* Grid::convertIndex(uint idx) const
{
    uint v = idx;
    for (uint d = 0; d < DIM; d++)
    {
        _idx[d] = v % N[d];
        v = (uint)(v/N[d]);
    }
    return _idx.data();
}

// Computes the "bottom left" DIM dimensional index of the grid node that contains point x.
// Invalidates all dim index references returned so far.
const uint* Grid::getNodeIndexBl(const double* x) const
{
    for (uint d = 0; d < DIM; d++)
        _idx[d] = (uint)qBound(0, (int)((x[d]-min[d])*strideinv[d]), (int)N[d]-2);
    return _idx.data();
}

// Computes the DIM dimensional index of the grid node closest to the point x.
// Out of bounds queries are truncated to the closest cell on the border.
// Invalidates all dim index references returned so far.
const uint* Grid::getNodeIndex(const double* x) const
{
    for (uint d = 0; d < DIM; d++)
        _idx[d] = (uint)qBound(0, qRound((x[d]-min[d])*strideinv[d]), (int)N[d]-1);
    return _idx.data();
}

// Computes the DIM dimensional index of the grid node closest to the point x.
// The index is unbounded and returns a theoretical index even outside of the
// grid area. Invalidates all dim index references returned so far.
const int* Grid::getUnboundedNodeIndex(const double* x) const
{
    for (uint d = 0; d < DIM; d++)
        _iidx[d] = qRound((x[d]-min[d])*strideinv[d]);
    return _iidx.data();
}

// Computes the "bottom left" flat index of the grid node that contains point x.
// Invalidates all dim index references returned so far.
uint Grid::getNodeFlatIndexBl(const double* x) const
{
    for (uint d = 0; d < DIM; d++)
        _idx[d] = (uint)qBound(0, (int)((x[d]-min[d])*strideinv[d]), (int)N[d]-2);
    return convertIndex(_idx.data());
}

// Computes the flat index of the grid node closest to the point x.
// Invalidates all dim index references returned so far.
uint Grid::getNodeFlatIndex(const double* x) const
{
    for (uint d = 0; d < DIM; d++)
        _idx[d] = (uint)qBound(0, qRound((x[d]-min[d])*strideinv[d]), (int)N[d]-1);
    return convertIndex(_idx.data());
}

// Returns the Grid coordinates of the node specified by the DIM dimensional index.
// Invalidates all point references returned so far.
const double* Grid::getNodeCoordinates(const uint* idx) const
{
    for (uint d = 0; d < DIM; d++)
        _x[d] = raster[d][idx[d]];
    return _x.data();
}

// Returns the Grid coordinates of the node specified by the flat index.
// Invalidates all point references returned so far.
const double* Grid::getNodeCoordinates(uint n) const
{
    const uint* idx = convertIndex(n);
    for (uint d = 0; d < DIM; d++)
        _x[d] = raster[d][idx[d]];
    return _x.data();
}

// Enumerates the flat indexes of the nodes in a neighborhood of radius r around
// the node specified by the flat index n. The radius is interpreted as the Manhattan
// distance in index space where directly neighboring nodes have the distance 1.
// When radius = 0, the method returns only the node n itself.
Vector<uint> Grid::getNeighborHood(uint n, uint radius) const
{
    return getNeighborHood(convertIndex(n), radius);
}

// Enumerates the flat indexes of the nodes in a neighborhood of radius r around
// the node specified by DIM index idx. The radius is interpreted as the Manhattan
// distance in index space where directly neighboring nodes have the distance 1.
// When radius = 0, the method returns only the node n itself.
Vector<uint> Grid::getNeighborHood(const uint *idx, uint radius) const
{
    uint center = convertIndex(idx);

    // Using the radius, determine the min and max boundaries of the enveloping
    // hypercube in index space while respecting the grid boundaries.
    uint bmin[DIM];
    uint bmax[DIM];
    for (uint d = 0; d < DIM; d++)
    {
        bmin[d] = (idx[d] < radius)? (uint)0 : (idx[d] - radius);
        //bmin[d] = qMax(idx[d]-radius, (uint)0);
        bmax[d] = qMin(idx[d]+radius,  N[d]-1);
        //qDebug() << idx[d] << radius << bmin[d] << bmax[d];
    }

    // Count the neighbors.
    uint neighbors = 1;
    for (uint d = 0; d < DIM; d++)
        neighbors *= bmax[d]-bmin[d]+1;
    // Generate the flat coordinates.
    Vector<uint> neighborhood;
    for (uint d = 0; d < DIM; d++)
        _idx[d] = bmin[d];

    for (uint i = 0; i < neighbors; i++)
    {
        uint c = convertIndex(_idx.data());
        if (c != center)
            neighborhood << c;
        uint d = 0;
        while (d < DIM)
        {
            _idx[d]++;
            if (_idx[d] <= bmax[d])
                break;
            _idx[d] = bmin[d];
            d++;
        }
    }

    return neighborhood;
}

// Returns the flat node indices of the hypercube that contains the given point x.
// If a radius > 0 is specified, the function expands the enveloping hypercube by
// radius in index space and returns all included node indices.
Vector<uint> Grid::getNeighborHood(const double* x, uint radius) const
{
    // Determine the bl node of the hypercube that contains x.
    const uint* idx = getNodeIndexBl(x);

    // Using the radius, determine the min and max boundaries of the enveloping
    // hypercube in index space while respecting the grid boundaries.
    uint bmin[DIM];
    uint bmax[DIM];
    for (uint d = 0; d < DIM; d++)
    {
        bmin[d] = qMax(idx[d]-radius, (uint)0);
        bmax[d] = qMin(idx[d]+radius+1, N[d]-1);
    }

    // Count the nodes.
    uint neighbors = 1;
    for (uint d = 0; d < DIM; d++)
        neighbors *= bmax[d]-bmin[d]+1;

    // Generate the flat coordinates.
    Vector<uint> neighborhood;
    for (uint d = 0; d < DIM; d++)
        _idx[d] = bmin[d];
    for (uint i = 0; i < neighbors; i++)
    {
        neighborhood << convertIndex(_idx.data());

        uint d = 0;
        while (d < DIM)
        {
            _idx[d]++;
            if (_idx[d] <= bmax[d])
                break;
            _idx[d] = bmin[d];
            d++;
        }
    }

    return neighborhood;
}

// Returns true if the given Cartesian point is within the boundaries of the grid.
bool Grid::contains(const double *x) const
{
    for (uint d = 0; d < DIM; d++)
        if (x[d] > max[d] || x[d] < min[d])
            return false;

    return true;
}

// Returns true if the given idx is "in range".
bool Grid::contains(const uint *idx) const
{
    for (uint d = 0; d < DIM; d++)
        if (idx[d] >= N[d])
            return false;
    return true;
}

// Returns a uniformly sampled point from the grid space.
// Invalidates all point references returned so far.
const double *Grid::uniformSample() const
{
    for (uint d = 0; d < DIM; d++)
        _x[d] = Statistics::uniformSample(min[d], max[d]);
    return _x.data();
}

// Loads a binary saved Grid.
void Grid::load(QString name)
{
    QString fileName = name.section(".", 0, 0);
    if (fileName.isEmpty())
    {
        qDebug() << "Grid::load(): invalid file name" << name;
        return;
    }
    fileName += ".gri";

    QFile file(fileName);
    if (!file.open(QIODevice::ReadOnly))
    {
        qDebug() << "Grid::load(): Could not open file" << file.fileName();
        return;
    }

    QDataStream in(&file);
    in >> DIM;
    setDim(DIM);
    for (uint i = 0; i < DIM; i++)
        in >> min[i];
    for (uint i = 0; i < DIM; i++)
        in >> max[i];
    for (uint i = 0; i < DIM; i++)
        in >> N[i];
    file.close();
    init();
}

// Saves the Grid in a binary file.
void Grid::save(QString name) const
{
    QString fileName = name.section(".", 0, 0);
    if (fileName.isEmpty())
    {
        qDebug() << "Grid::save(): invalid file name" << name;
        return;
    }
    fileName += ".gri";

    QFile file(fileName);
    if (!file.open(QIODevice::WriteOnly))
    {
        qDebug() << "Grid::save(): could not write to file" << file.fileName();
        return;
    }

    QDataStream out(&file);
    out << DIM;
    for (uint i = 0; i < DIM; i++)
        out << min[i];
    for (uint i = 0; i < DIM; i++)
        out << max[i];
    for (uint i = 0; i < DIM; i++)
        out << N[i];
    file.close();
}

// Prints a textual represenation of the Grid node coordinates to the console.
void Grid::printGridNodes() const
{
    qDebug() << "d n coord";
    for (uint d = 0; d < DIM; d++)
    {
        qDebug() << "-----------";
        for (uint n = 0; n < N[d]; n++)
            qDebug() << d << n << raster[d][n];
    }
}

// QPainter code that draws the grid node coordinates in a 2D space.
void Grid::drawGridNodes(QPainter* painter, uint sampleFactor) const
{
    if (DIM < 2)
        return;

    sampleFactor = qMax(sampleFactor, (uint)1);

    double ws = 0.05;

    painter->save();
    painter->setPen(drawUtil.pen);
    painter->setBrush(drawUtil.brush);
    for (uint j = 0; j < N[1]; j=j+sampleFactor)
        for (uint i = 0; i < N[0]; i=i+sampleFactor)
            painter->drawEllipse(QPointF(raster[0][i], raster[1][j]), ws, ws);
    painter->restore();
}

// OpenGL code that draws the grid node coordinates in a 2D space.
void Grid::drawGridNodes(uint sampleFactor) const
{
    if (DIM < 2)
        return;

    sampleFactor = qMax(sampleFactor, (uint)1);

    glBegin(GL_POINTS);
    for (uint j = 0; j < N[1]; j=j+sampleFactor)
        for (uint i = 0; i < N[0]; i=i+sampleFactor)
            glVertex2d(raster[0][i], raster[1][j]);
    glEnd();
}

// OpenGL code that draws the border of the grid using the node coordinates in a 2D space.
void Grid::drawBorder(uint sampleFactor) const
{
    if (DIM < 2)
        return;

    sampleFactor = qMax(sampleFactor, (uint)1);

    glBegin(GL_POINTS);
    for (uint i = 0; i < N[0]; i=i+sampleFactor)
    {
        glVertex2d(raster[0][i], raster[1][0]);
        glVertex2d(raster[0][i], raster[1][N[1]-1]);
    }
    for (uint j = 1; j < N[1]-1; j=j+sampleFactor)
    {
        glVertex2d(raster[0][0], raster[1][j]);
        glVertex2d(raster[0][N[0]-1], raster[1][j]);
    }
    glEnd();
}
