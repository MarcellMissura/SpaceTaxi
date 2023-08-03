#ifndef ADJACENCYMATRIX_H_
#define ADJACENCYMATRIX_H_
#include "Vector.h"
#include "Vec2u.h"

// This is a memory preserving implementation of an adjacency matrix.
// Memory preserving means that the matrix grows automatically in size
// when needed, but memory is never released again to avoid reoccuring
// heap allocations. There is no need to define the structure.
// The matrix will always accomodate an index (i,j).

class AdjacencyMatrix
{
    Vector< Vector<char> > m;

public:

    AdjacencyMatrix();
    ~AdjacencyMatrix(){}

    void clear();
    void set(uint i, uint j);
    void check(uint i, uint j);
    void unset(uint i, uint j);
    bool isSet(uint i, uint j) const;
    bool isUnset(uint i, uint j) const;
    bool isChecked(uint i, uint j) const;
    Vec2u size() const;

};

#endif
