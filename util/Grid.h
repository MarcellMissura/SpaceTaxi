#ifndef GRID_H_
#define GRID_H_
#include <QString>
#include <QPainter>
#include "util/Vector.h"

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
// uint N[3] = {101,201,301};
// grid.setN(N); // Set the number of nodes per dimension.
// double min[3] = {config.xMin, config.yMin, config.zMin};
// grid.setMin(min); // Set the minimum values per dimension.
// double max[3] = {config.xMax, config.yMax, config.zMax};
// grid.setMax(max); // Set the maximum values per dimension.
// grid.init(); // Initialize the grid representation.
//
// Now you can access the grid node coordinates using either a DIM
// dimensional unsigned integer index, or a flat unsigned integer index
// that can sequentially enummerate all grid nodes. Example:
//
// for (uint n=0; n < grid.getNodeCount(); n++)
//    Vec3 coordinates = grid.getNodeCoordinates(n);
//
// A number of other nifty methods allow you to convert between the dim
// index and the flat index representations, query the nearest node index
// of any point on the grid, retrieve all eveloping nodes of a query point,
// retreive the neighborhood of a grid node, and to save and load the
// grid to and from a binary file.
//
// As a general rule, points in the grid space are represented as
// QVector<double>, dim indexes are represented as QVector<uint>, and
// flat indexes are represented as uint.
//
// Please note that when setting the number of dimensions with setDim()
// and when calling rasterize(), heap memory needs to be allocated and
// this is not a real time capable operation. If performance is crucial,
// construct the grid offline first. All other methods are as fast as
// can be.

class Grid
{
private:

    // Grid parameters.
    uint DIM; // Number of dimensions.
    std::vector<uint> N; // Number of nodes for each dimension.
    std::vector<double> max; // Input range max values for each dimension.
    std::vector<double> min; // Input range min values for each dimension.

    // Internal structure.
    uint nodeCount; // nodes in total
    std::vector<std::vector<double> >  raster; // Grid coordinates.
    std::vector<uint> cumN; // speeds up index to flat index transformations
    std::vector<double> stride; // speeds up getAt()
    std::vector<double> strideinv; // speeds up evaluateAt()

private:
    mutable std::vector<uint> _idx; // Temporary storage for a dim index.
    mutable std::vector<int> _iidx; // Temporary storage for a dim index.
    mutable std::vector<double> _x; // Temporary storage for a point.

public:

    Grid();
    virtual ~Grid(){}

    // Grid structural methods.
    void setDim(uint d); // Sets DIM, the number of dimensions.
    uint getDim() const; // Returns DIM, the number of dimensions.
    void setMin(double minn); // Sets the lower data range boundaries.
    void setMax(double maxx); // Sets the upper data range boundaries.
    void setMin(const double* minn); // Sets the lower data range boundaries.
    void setMax(const double* maxx); // Sets the upper data range boundaries.
    const double* getMin() const;
    const double* getMax() const;
    void setN(uint N_); // Sets the number of nodes per dimension.
    void setN(const uint* N_); // Sets the number of nodes per dimension.
    const uint* getN() const; // Returns the number of nodes per dimension.
    virtual void init(); // Generates the Grid node coordinates.

    const double* getStride() const; // Returns a reference to the DIM sized vector that contains the strides for each dimension.
    uint getNodeCount() const; // Returns the total number of Grid nodes.
    uint convertIndex(const uint* idx) const; // Converts a DIM dimensional index to a flat index.
    const uint* convertIndex(uint idx) const; // Converts a flat index to a DIM dimensional index.
    const uint* getNodeIndexBl(const double* x) const; // Finds the bottom left node index of the point x.
    const uint* getNodeIndex(const double* x) const; // Finds the index of the point x.
    const int* getUnboundedNodeIndex(const double* x) const; // Finds the unbounded index of the point x.
    uint getNodeFlatIndexBl(const double* x) const; // Finds the bottom left flat index of the point x.
    uint getNodeFlatIndex(const double* x) const; // Finds the flat index of the point x.
    const double* getNodeCoordinates(const uint* idx) const; // Returns the Grid coordinates of the node specified by the DIM dimensional index.
    const double* getNodeCoordinates(uint idx) const; // Returns the Grid coordinates of the node specified by the flat index.
    bool contains(const double* x) const;
    bool contains(const uint* idx) const;
    Vector<uint> getNeighborHood(const double* x, uint radius=0) const; // Returns the flat indices of the nodes surrounding the query point.
    Vector<uint> getNeighborHood(const uint* idx, uint radius=1) const; // Returns a list of flat indexes of the neighbors of node idx within radius r.
    Vector<uint> getNeighborHood(uint idx, uint radius=1) const; // Returns a list of flat indexes of the neighbors of node idx within radius r.
    const double* uniformSample() const; // Returns a uniformly sampeled point from the grid space.

    virtual void load(QString name = "Grid.gri");
    virtual void save(QString name = "Grid.gri") const;

    void printGridNodes() const; // Text console output of the raster coordinates.
    void drawGridNodes(QPainter *painter, uint sampleFactor=1) const; // Qt drawing code.
    void drawGridNodes(uint sampleFactor=1) const; // OpenGL drawing code.
    void drawBorder(uint sampleFactor=1) const; // OpenGL drawing code.
};

#endif /* Grid_H_ */
