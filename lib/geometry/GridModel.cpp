#include "GridModel.h"
#include "PathAStar.h"
#include "board/Config.h"
#include "lib/util/Logger.h"
#include "lib/util/DrawUtil.h"
#include "lib/util/GLlib.h"
#include "lib/util/Vec2i.h"
#include <iostream>

// The grid model is a reactangular grid of cells used for the representation of
// occupancy maps. Each cell contains a real value between 0 and 1 that indicates
// the occupancy of the cell. The GridModel extends Grid and needs to be init()-ed like so:
//
// GridModel gm;
// gm.setDim(2); // Set the number of dimensions. This is always 2.
// gm.setN(Vec2u(101, 201); // Set the numer of nodes per dimension.
// gm.setMin(Vec2(config.xMin, config.yMin)); // Set the minimum values per dimension.
// gm.setMax(Vec2(config.xMax, config.yMax)); // Set the maximum values per dimension.
// gm.init(); // Initialize the grid.
//
// In the heart of the GridModel is an opencv matrix M of type uchar (1 channel).
// Each number of the matrix (each cell in the grid) can have one of 256 values
// that are mapped to the double valued [0,1] range. The methods of the GridModel
// class revolve around this matrix such as init()-ializing it, setting and
// reading values of cells with setAt() and getAt(), and checking for occupancy
// with the isOccupied() method.

// Furthermore, we can use image filtering techniques to for example erode() and
// dilate(), which is a great way of erasing or expanding obstacles. You can also
// blur() the map, which smoothens the edges with a Gaussian blur.

// Last but not least, the GridModel offers a way to computeOccupancyGrid() from
// a 2D or a 3D point cloud or a set of polygons. The GridModel can also convert
// the grid to polygons with extractPolygons() using a contour detection algorithm.

// The GridModel provides draw() methods for QPainter and OpenGL.

// Now there are also path planning methods integrated into the GridModel.
// After setting up the occupancy grid, call Vector<Vec2> path = computePath(from, to).
// Or, call initDijkstraMap(const Vec2 &to) to prepare a Dijkstra map for a fixed
// target and then call Vector<Vec2> path = computeDijkstraPath(from, to) to compute
// a path using the prepared Dijkstra map.

GridModel::GridModel()
{

}

// Copy constructor.
GridModel::GridModel(const GridModel &o) : Grid(o)
{
    *this = o;
}

// Assignment operator.
GridModel& GridModel::operator=(const GridModel &o)
{
    if (this == &o)
        return *this;

    // Set up the grid parameters.
    setDim(o.getDim());
    setN(o.getN());
    setMin(o.getMin());
    setMax(o.getMax());
    init();

    M = o.M.clone(); // The reason why we need the assignment operator.

    return *this;
}

// cv::Mat Copy constructor.
GridModel::GridModel(const cv::Mat &o)
{
    *this = o;
}

// cv::Mat Assignment operator.
GridModel& GridModel::operator=(const cv::Mat &o)
{
    // Set up the grid parameters.
    setDim(2);
    setN(Vec2u(o.cols, o.rows));
    setMin(Vec2(0, -config.gridCellSize*o.rows));
    setMax(Vec2(config.gridCellSize*o.cols, 0));
    init();

    M = o.clone();

    return *this;
}

// Swap the backing matrix in an initialized grid
void GridModel::setMat(const cv::Mat &o)
{
    M = o.clone();
}

// Get a copy of the backing matrix
const cv::Mat& GridModel::getMat() const
{
    return M;
}

// The init method sets up the grid structure (min, max, number of cells) and computes the
// raster of the grid coordinates. This is where the data matrix M is initialized.
void GridModel::init()
{
    Grid::init();

    // Set up the openCV matrix M (the data structure for the grid).
    Vec2u N = getN();
    M = cv::Mat::zeros(cv::Size(N.x,N.y), cv::DataType<uchar>::type);
}

// Clears the data (resets to 0), but leaves grid the structure.
void GridModel::clear()
{
    M = cv::Scalar(0);
}

// Returns the width of the grid (number of cells).
uint GridModel::getWidth() const
{
    return getN()[0];
}

// Returns the height of the grid (number of cells).
uint GridModel::getHeight() const
{
    return getN()[1];
}

// Computes an occupancy grid using the Vector of Polygons passed as a parameter.
// After this method, all cells that intersect with the blocking lines of the
// polygon, but will have a value of 1. All other cells a value of 0.
// The polygon has to be provided in the coordiate frame of the grid.
// A thickness parameter can be provided to draw thicker lines.
void GridModel::computeOccupancyGrid(const Vector<Polygon>& polygons, int thickness)
{
    //qDebug() << "GridModel::computeOccupancyGrid():";
    for (uint i = 0; i < polygons.size(); i++)
        computeOccupancyGrid(polygons[i], thickness);

    return;
}

// Computes an occupancy grid using the polygons passed as a parameter best
// suited for collision checking. The borders of the given polygons, but only
// the blocking lines and not the sight lines, are drawn with a padding towards
// the inside of the polygon. The polygons have to be provided in the coordiate
// frame of the grid.
void GridModel::computeOccupancyGrid(const LinkedList<Polygon>& polygons, int thickness)
{
    clear();
    ListIterator<Polygon> polIt = polygons.begin();
    while (polIt.hasNext())
        computeOccupancyGrid(polIt.next(), thickness);
}

// Computes an occupancy grid using the polygon passed as a parameter.
// After this method, all cells that intersect with the border of the given
// polygon will have a value of 1. All other cells a value of 0.
// The polygon has to be provided in the coordinate frame of the grid.
// A thickness parameter can be provided to draw thicker lines.
void GridModel::computeOccupancyGrid(const Polygon& polygon, int thickness)
{
    //qDebug() << "GridModel::computeOccupancyGrid():";
    //qDebug() << polygon;

    // The drawing functions from the OpenCV library are used
    // to draw the polygons directly onto the matrix M.

    std::vector<std::vector<cv::Point>> segmentsAsContour;
    std::vector<cv::Point> segment;
    ListIterator<Line> it = polygon.edgeIterator();
    while (it.hasNext() && it.cur().isSightLine())
        it.next();
    while (it.hasNext())
    {
        const Line& edge = it.next();
        Vec2i idx = getUnboundedNodeIndex(edge.p1());
        segment.push_back(cv::Point(idx.x, idx.y));
        if (edge.isSightLine())
        {
            if (segment.size() > 1)
                segmentsAsContour.push_back(segment);
            segment.clear();
        }
    }
    if (!segment.empty())
    {
        Vec2i idx = getUnboundedNodeIndex(it.cur().p1());
        segment.push_back(cv::Point(idx.x, idx.y));
        segmentsAsContour.push_back(segment);
    }

    cv::Scalar color(255);
    cv::polylines(M, segmentsAsContour, false, color, thickness);

    return;
}

// Computes an occupancy grid from the given 3D point cloud (typically rgbd data).
// All cells that at least one point projects into will be set to a value of 1.
// All other cells remain untouched. Points whose z coordinate is lower
// than floorHeight and higher than ceilingHeight are discarded.
void GridModel::computeOccupancyGrid(const Vector<Vec3>& pointCloud, double floorHeight, double ceilingHeight)
{
    clear();
    int count = 0;
    for (uint i = 0; i < pointCloud.size(); i++)
    {
        const Vec3& pt = pointCloud[i];
        if (pt.isNan())
            continue;

        if (pt.z < floorHeight || pt.z > ceilingHeight)
            continue;

        if (!contains(pt))
            continue;

        setAt(getNodeIndex(pt), 1);
        count++;
    }
}

// Computes a binary occupancy grid from the given 2D point cloud (typically laser data).
// All cells that contain at least one point are set to a value of 1.
// All other cells are set to 0.
void GridModel::computeOccupancyGrid(const Vector<Vec2>& pointCloud)
{
    clear();
    for (uint i = 0; i < pointCloud.size(); i++)
        if (!pointCloud[i].isNull() && contains(pointCloud[i]) && pointCloud[i].norm() < config.laserLineMaxDistance)
            setAt(pointCloud[i], 1);
}

// Clears the cells that overlap with the polygon.
void GridModel::clearPolygon(const Polygon& polygon)
{
    // Drawing functions from the OpenCV library are used
    // to draw the polygon directly onto the matrix M.

    std::vector<std::vector<cv::Point>> segmentsAsContour;
    std::vector<cv::Point> segment;
    ListIterator<Line> it = polygon.edgeIterator();
    while (it.hasNext())
    {
        const Line& edge = it.next();
        Vec2i idx = getUnboundedNodeIndex(edge.p1());
        segment.push_back(cv::Point(idx.x, idx.y));
    }
    segmentsAsContour.push_back(segment);

    cv::Scalar color(0);
    cv::fillPoly(M, segmentsAsContour, color);

    return;
}

// Sets all border cells to a value of 1 (occupied).
// This can be used to keep A* from accessing the grid out of bounds.
void GridModel::blockBorder()
{
    Vec2u N = getN();
    for (uint i = 0; i < N[0]; i++)
    {
        setAt(Vec2u(i, 0), 1);
        setAt(Vec2u(i, N[1]-1), 1);
    }
    for (uint j = 0; j < N[1]; j++)
    {
        setAt(Vec2u(0, j), 1);
        setAt(Vec2u(N[0]-1, j), 1);
    }
}

// Returns true if the cell that the point x is in is occupied.
// All values > 1 - EPSILON are considered to be occupied.
bool GridModel::isOccupied(const Vec2 &x) const
{
    return isOccupied(getNodeIndex(x));
}

// Returns true if the cell with the two dimensional index idx is occupied.
// All values > 0.99 are considered to be occupied.
bool GridModel::isOccupied(const Vec2u &idx) const
{
    return (getAt(idx) > 0.99);
}

// Returns true if the cell with the index i,j is occupied.
// All values > 0.99 are considered to be occupied.
bool GridModel::isOccupied(uint i, uint j) const
{
    return (getAt(i, j) > 0.99);
}

// Returns true if the cell with the flat index n is occupied.
// All values > 1 - EPSILON are considered to be occupied.
bool GridModel::isOccupied(uint n) const
{
    return isOccupied(convertIndex(n));
}

// Returns true if the line between cell A and cell B does not come across an occupied cell.
// The implementation is based on the Bresenham algorithm.
// https://de.wikipedia.org/wiki/Bresenham-Algorithmus
bool GridModel::hasLineOfSight(const Vec2u& cellIdxA, const Vec2u& cellIdxB) const
{
    Vec2u p;
    int t, dx, dy, incx, incy, pdx, pdy, ddx, ddy, deltaslowdirection, deltafastdirection, err;

    int xstart = cellIdxA.x;
    int xend = cellIdxB.x;
    int ystart = cellIdxA.y;
    int yend = cellIdxB.y;

    /* Entfernung in beiden Dimensionen berechnen */
    dx = xend - xstart;
    dy = yend - ystart;

    /* Vorzeichen des Inkrements bestimmen */
    incx = sgn0(dx);
    incy = sgn0(dy);
    if(dx<0) dx = -dx;
    if(dy<0) dy = -dy;

    /* feststellen, welche Entfernung größer ist */
    if (dx>dy)
    {
        /* x ist schnelle Richtung */
        pdx=incx; pdy=0;    /* pd. ist Parallelschritt */
        ddx=incx; ddy=incy; /* dd. ist Diagonalschritt */
        deltaslowdirection =dy;   deltafastdirection =dx;   /* Delta in langsamer Richtung, Delta in schneller Richtung */
    }
    else
    {
        /* y ist schnelle Richtung */
        pdx=0;    pdy=incy; /* pd. ist Parallelschritt */
        ddx=incx; ddy=incy; /* dd. ist Diagonalschritt */
        deltaslowdirection =dx;   deltafastdirection =dy;   /* Delta in langsamer Richtung, Delta in schneller Richtung */
    }

    /* Initialisierungen vor Schleifenbeginn */
    p.x = xstart;
    p.y = ystart;
    err = deltafastdirection/2;

    if (isOccupied(p))
        return false;

    /* Pixel berechnen */
    for(t=0; t<deltafastdirection; ++t) /* t zaehlt die Pixel, deltafastdirection ist Anzahl der Schritte */
    {
        /* Aktualisierung Fehlerterm */
        err -= deltaslowdirection;
        if(err<0)
        {
            /* Fehlerterm wieder positiv (>=0) machen */
            err += deltafastdirection;
            /* Schritt in langsame Richtung, Diagonalschritt */
            p.x += ddx;
            p.y += ddy;
        }
        else
        {
            /* Schritt in schnelle Richtung, Parallelschritt */
            p.x += pdx;
            p.y += pdy;
        }

        if (isOccupied(p))
            return false;
    }

    return true;
}

// Performs a quick but unprecise check if the polygon collides with the grid.
// Every vertex of the polygon is mapped to the grid and checked if the cell
// is occupied. This is done with the centroid of the polygon as well. If one
// of the touched cells is occupied, the polygon is considered to be colliding.
// The polygon is assumed to be given in the coordinate system of the grid.
bool GridModel::polygonCollisionCheck(const Polygon &p) const
{
    ListIterator<Line> it = p.edgeIterator();
    while (it.hasNext())
        if (isOccupied(it.next().p1()))
            return true;
//    if (isOccupied(p.centroid()))
//        return true;
    return false;
}

// Computes a path from "from" to "to" using the LazyTheta* algorithm.
Vector<Vec2> GridModel::computePath(const Vec2 &from, const Vec2 &to) const
{
    PathAStar pathAStar;
    pathAStar.init();
    //pathAStar.setDebug(config.debugLevel);
    pathAStar.setGridModel(this);
    pathAStar.setStartState(from);
    pathAStar.setTargetState(to);
    pathAStar.lazyThetaStarSearch();
    return pathAStar.getPath();
}

// Computes a path from "from" using a Dijkstra table lookup.
// The target "to" must have been already provided for the
// precomputation of the Dijkstra map with initDijkstraMap(to).
Vector<Vec2> GridModel::computeDijkstraPath(const Vec2 &from, const Vec2 &to) const
{
    return dij.dijkstraPath(from);
}

// Precomputes a Dijkstra map using the occupancy grid and the target "to".
void GridModel::initDijkstraMap(const Vec2 &to)
{
    //qDebug() << "GridModel::initDijkstraMap(to)" << to;
    dij.computeDijkstraMap(*this, to);
}

// Returns the maximum grid value taken at the corners of the polygon.
double GridModel::getAt(const Polygon &p) const
{
    double max = 0;
    ListIterator<Line> it = p.edgeIterator();
    while (it.hasNext())
        max = qMax(max, getAt(it.next().p1()));
    //max = qMax(max, getAt(p.centroid()));
    return max;
}

// Returns the bounding box of the grid area.
Box GridModel::boundingBox() const
{
    return Box(getMax()[1], getMin()[0], getMin()[1], getMax()[0]);
}

// Adds GridModel o to this one by executing the opencv mat += operation.
void GridModel::operator+=(const GridModel &o)
{
    M += o.M;
}

GridModel GridModel::operator+(const GridModel &o)
{
    GridModel gm = *this;
    gm += o;
    return gm;
}

void GridModel::operator-=(const GridModel &o)
{
    cv::Mat negative;
    cv::bitwise_not(o.M, negative);
    M = M & negative;
}

// Sets this grid to the max of this and o. (max(this[i][j], o[i][j]))
void GridModel::max(const GridModel &o)
{
    uchar* Mi = M.ptr<uchar>(0);
    const uchar* Oi = o.getMat().ptr<uchar>(0);
    for(int j = 0; j < M.cols*M.rows; j++)
        Mi[j] = std::max(Mi[j], Oi[j]);
    return;
}

// Applies an erode operation by radius to the occupancy grid.
// The radius is given in meters.
void GridModel::erode(double radius)
{
    Vec2 stride = getStride();
    if (!((2*radius/stride.x) > 1) || !((2*radius/stride.y) > 1))
        return;

    // Execute the OpenCV erode routine.
    cv::Mat mask = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2*radius/stride.x, 2*radius/stride.y));
    cv::erode(M, M, mask);
}

// Applies a dilate operation by radius (in meters) to the occupancy grid.
// This is a great way of expanding the obstacles by the robot size.
void GridModel::dilate(double radius)
{
    Vec2 stride = getStride();
    if (!((2*radius/stride.x) > 1) || !((2*radius/stride.y) > 1))
        return;

    // Execute the OpenCV dilate routine.
    cv::Mat mask = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2*radius/stride.x, 2*radius/stride.y));
    cv::dilate(M, M, mask);
}

// Applies a blur operation by radius (in meters) to the occupancy grid.
// This is useful to smoothen an occupancy map for DWA.
void GridModel::blur(double radius)
{
    Vec2 stride = getStride();
    uint rx = 2*radius/stride.x;
    uint ry = 2*radius/stride.y;
    if (!(rx > 1) || !(ry > 1))
        return;

    // Execute the OpenCV blur filter.
    cv::blur(M, M, cv::Size(rx, ry));
    //cv::GaussianBlur(M, M, cv::Size(7, 7), 100, 100);
}

// Applies a Canny edge filter to the occupancy grid.
// This has never been used so far.
void GridModel::canny()
{
    // Execute the OpenCV canny edge detector.
    cv::Canny(M, M, 0, 1);
}

// Invert the grid so the free space is marked and the occupied space is unmarked.
void GridModel::invert()
{
    cv::bitwise_not(M,M);
}

// Returns a list of polygons extracted from the grid. The algorithm segments the grid
// by means of contour detection. The segments are converted to non-convex disjunct polygons.
// The returned polygons have a zero pose and the coordinates of their vertices are in the
// local coordinate frame of the grid.
LinkedList<Polygon> GridModel::extractPolygons() const
{
    Vec2 stride = getStride();

    // Contour detection.
    // The most external contours are always sufficient when it comes to sensing polygons.
    std::vector<std::vector<cv::Point>> segmentsAsContour;
    std::vector<cv::Vec4i> hierachy;
    // RETR_EXTERNAL: retrieve only the most external (top-level) contours. (fastest)
    // RETR_LIST, retrieve all contours without any hierarchical information
    // RETR_TREE, retrieve all contours with hierarchical information
    // CHAIN_APPROX_SIMPLE: compresses horizontal, vertical, and diagonal segments and leaves only their end points
    cv::findContours(M, segmentsAsContour, hierachy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // Convert the polygons to static obstacles.
    thread_local LinkedList<Polygon> polygons;
    polygons.clear();
    for (uint i = 0; i < segmentsAsContour.size(); i++)
    {
        if (segmentsAsContour[i].size() < 3)
            continue;

        thread_local Polygon pol;
        pol.clear();
        pol.setId(polygons.size()+1);
        for (uint j = 0; j < segmentsAsContour[i].size(); j++)
            pol << Vec2(segmentsAsContour[i][j].x, segmentsAsContour[i][j].y);
        pol.scale(stride.x, stride.y);
        pol.translate(getMin());
        pol.transform();
        polygons << pol;
    }

    // The obstacles sensed from the grid ARE already transformed.
    return polygons;
}

// Returns a random collision-free pose in the map according to isOccupied().
// Since the position is obtained by random sampling, it can theoretically happen that a collision-free
// pose is never found. After 1000 tries, the method gives up trying and returns a random colliding position.
Vec2 GridModel::getRandomFreePosition() const
{
    bool good = false;
    int counter = 0;
    Vec2 rp;
    while (!good && counter < 1000)
    {
        rp = uniformSample();
        if (!isOccupied(rp))
            good = true;
        counter++;
        if (counter >= 1000)
            qDebug() << "Warning! GridModel::getRandomFreePosition() failed to find a free position.";
    }

    return rp;
}

// Returns the output value of the grid cell that contains point x.
double GridModel::getAt(Vec2 x) const
{
    Vec2u idx = getNodeIndex(x);
    return (double)M.at<uchar>(idx.y, idx.x)/255;
}

// Returns the output value of the grid cell specified by the index idx.
double GridModel::getAt(Vec2u idx) const
{
    return (double)M.at<uchar>(idx.y, idx.x)/255;
}

uchar GridModel::getCountAt(Vec2u idx) const
{
    return M.at<uchar>(idx.y, idx.x);
}

// Returns the output value of the grid cell specified by the index i,j.
double GridModel::getAt(uint i, uint j) const
{
    return (double)M.at<uchar>(j, i)/255;
}

// Returns the output value of the grid cell specified by the flat index n.
double GridModel::getAt(uint n) const
{
    Vec2u idx = convertIndex(n);
    return (double)M.at<uchar>(idx.y, idx.x)/255;
}

// Sets the grid cell that contains the point x to value v.
// v is expected to be in [0,1] otherwise the behavior is undefined.
void GridModel::setAt(Vec2 x, double v)
{
    Vec2u idx = getNodeIndex(x);
    M.at<uchar>(idx.y, idx.x) = v*255;
}

// Sets the grid cell specified by the index idx to value v.
// v is expected to be in [0,1] otherwise the behavior is undefined.
void GridModel::setAt(Vec2u idx, double v)
{
    M.at<uchar>(idx.y, idx.x) = v*255;
}

// Sets the grid cell specified by the flat index n to value v.
// v is expected to be in [0,1] otherwise the behavior is undefined.
void GridModel::setAt(uint n, double v)
{
    Vec2u idx = convertIndex(n);
    M.at<uchar>(idx.y, idx.x) = v*255;
}

// Sets the grid cell specified by the index i,j to value v.
// v is expected to be in [0,1] otherwise the behavior is undefined.
void GridModel::setAt(uint i, uint j, double v)
{
    M.at<uchar>(j, i) = v*255;
}


// Draws the occupancy grid on a QPainter.
void GridModel::draw(QPainter *painter, const QBrush& brush, double opacity, double min, double max) const
{
    drawUtil.setHeatMapColor(brush.color());

    Vec2u n = getN();
    Vec2 stride = getStride();

    painter->save();
    painter->setOpacity(opacity);
    painter->setPen(Qt::NoPen);

    // Draw the grid nodes and the occupancy squares.
    for (uint j = 0; j < n.y; j++)
    {
        for (uint i = 0; i < n.x; i++)
        {
            Vec2u idx(i,j);
            Vec2 c = getNodeCoordinates(idx);

            if (getAt(idx) <= 0)
                continue;

            // The occupancy square.
            QRectF q(c.x, c.y, stride.x, stride.y);
            q.translate(-0.5*stride.x, -0.5*stride.y);
            painter->setBrush(drawUtil.getHeatMapColor(getAt(idx), min, max));
            if (isOccupied(idx))
                painter->setBrush(drawUtil.brushRed);
            painter->drawRect(q);
            //qDebug() << i << j << idx << getAt(idx) << colorUtil.getHeatMapColor(getAt(idx), min, max);

            // The grid node.
            double circleRadius = 0.002;
            painter->setBrush(drawUtil.brush);
            painter->drawEllipse(c, circleRadius, circleRadius);
        }
    }

    drawUtil.resetHeatMapColor();
    painter->restore();
}

// Draws the occupancy grid in OpenGL.
void GridModel::draw(const QBrush& brush, double opacity, double min, double max) const
{
    //qDebug() << "GridModel::draw():" << c;

    Vec2u n = getN();
    Vec2 stride = getStride();

    drawUtil.setHeatMapColor(brush.color());

    glPushMatrix();
    glTranslated(-0.5*stride.x, 0.5*stride.y, 0);

    QColor color;
    for (uint j = 0; j < n.y; j++)
    {
        for (uint i = 0; i < n.x; i++)
        {
            Vec2u idx(i,j);
            if (!getAt(idx))
                continue;

            Vec2 c = getNodeCoordinates(idx);
            color = drawUtil.getHeatMapColor(getAt(idx), min, max);
            glColor4f(color.redF(), color.greenF(), color.blueF(), opacity*color.alphaF());
            //GLlib::drawQuad(c.x-0.5*stride.x, c.y+0.5*stride.y, stride.x, stride.y);
            GLlib::drawQuad(c.x, c.y, stride.x, stride.y);

            //qDebug() << "drawing" << i << j << c << "value:" << valueAt(idx) << colorUtil.getHeatMapColor(valueAt(idx), min, max) << color;
            //qDebug() << "Empty cells:" << emptyCount << "occupied:" << (n.x * n.y - emptyCount);
        }
    }

    glPopMatrix();

    drawUtil.resetHeatMapColor();
}

void GridModel::drawCounts(const QBrush &brush, double opacity, double min, double max) const
{
    Vec2u n = getN();
    Vec2 stride = getStride();

    drawUtil.setHeatMapColor(brush.color());

    glPushMatrix();
    glTranslated(-0.5*stride.x, 0.5*stride.y, 0);

    QColor color;
    for (uint j = 0; j < n.y; j++)
    {
        for (uint i = 0; i < n.x; i++)
        {
            Vec2u idx(i,j);

            if (!getCountAt(idx))
                continue;

            Vec2 c = getNodeCoordinates(idx);
            color = drawUtil.getHeatMapColor(getCountAt(idx), min, max);
            glColor4f(color.redF(), color.greenF(), color.blueF(), opacity);
            GLlib::drawQuad(c.x, c.y, stride.x, stride.y);
        }
    }

    glPopMatrix();
    drawUtil.resetHeatMapColor();
}

void GridModel::drawOccupancy(const QColor &color, double opacity) const
{
    Vec2u n = getN();
    Vec2 stride = getStride();

    glPushMatrix();
    glTranslated(-0.5*stride.x, 0.5*stride.y, 0);

    for (uint j = 0; j < n.y; j++)
    {
        for (uint i = 0; i < n.x; i++)
        {
            Vec2u idx(i,j);

            if (!getAt(idx))
                continue;

            Vec2 c = getNodeCoordinates(idx);
            glColor4f(color.redF(), color.greenF(), color.blueF(), opacity);
            GLlib::drawQuad(c.x, c.y, stride.x, stride.y);
        }
    }

    glPopMatrix();
}

// Draws the border of the occupancy grid on a QPainter.
void GridModel::drawBorder(QPainter *painter) const
{
    double circleRadius = 0.02;

    painter->save();
    painter->setOpacity(1.0);
    painter->setPen(Qt::NoPen);
    painter->setBrush(drawUtil.brush);

    Vec2u n = getN();
    for (uint j = 0; j < n.y; j++)
    {
        painter->drawEllipse(Vec2(getNodeCoordinates(Vec2u(0,j))), circleRadius, circleRadius);
        painter->drawEllipse(Vec2(getNodeCoordinates(Vec2u(n.x-1,j))), circleRadius, circleRadius);
    }
    for (uint i = 0; i < n.x; i++)
    {
        painter->drawEllipse(Vec2(getNodeCoordinates(Vec2u(i,0))), circleRadius, circleRadius);
        painter->drawEllipse(Vec2(getNodeCoordinates(Vec2u(i,n.y-1))), circleRadius, circleRadius);
    }

    painter->restore();
}

void GridModel::drawBorder() const
{
    static const double circleRadius = 0.02;
    Transform3D T;

    // Black.
    glColor3f(0, 0, 0);

    Vec2u n = getN();
    for (uint j = 0; j < n.y; j++)
    {
        const Vec2& pt1 = getNodeCoordinates(Vec2u(0,j));
        T.setFromParams(pt1.x, pt1.y, 0, 0, 0, 0);
        glPushMatrix();
        glMultMatrixd(T);
        GLlib::drawFilledCircle(circleRadius);
        glPopMatrix();

        const Vec2& pt2 = getNodeCoordinates(Vec2u(n.x-1,j));
        T.setFromParams(pt2.x, pt2.y, 0, 0, 0, 0);
        glPushMatrix();
        glMultMatrixd(T);
        GLlib::drawFilledCircle(circleRadius);
        glPopMatrix();
    }
    for (uint i = 0; i < n.x; i++)
    {
        const Vec2& pt1 = getNodeCoordinates(Vec2u(i,0));
        T.setFromParams(pt1.x, pt1.y, 0, 0, 0, 0);
        glPushMatrix();
        glMultMatrixd(T);
        GLlib::drawFilledCircle(circleRadius);
        glPopMatrix();

        const Vec2& pt2 = getNodeCoordinates(Vec2u(i,n.y-1));
        T.setFromParams(pt2.x, pt2.y, 0, 0, 0, 0);
        glPushMatrix();
        glMultMatrixd(T);
        GLlib::drawFilledCircle(circleRadius);
        glPopMatrix();
    }
}

void GridModel::drawDijkstraMap(QPainter *painter) const
{
    dij.draw(painter);
}

// Prints a text output to the console.
void GridModel::printOutputs() const
{
    for (uint i = 0; i < getNodeCount(); i++)
        qDebug() << i << Vec2u(convertIndex(i)) << getAt(i);
}

QDebug operator<<(QDebug dbg, const GridModel &w)
{
    dbg  << "dim:" << w.getDim() << "N:" << Vec2u(w.getN()) << "min:" << Vec2(w.getMin()) << "max:" << Vec2(w.getMax());
    return dbg;
}
