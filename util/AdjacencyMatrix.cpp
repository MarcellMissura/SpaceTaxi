#include "AdjacencyMatrix.h"

// This is a memory preserving implementation of an adjacency matrix.
// Memory preserving means that the matrix grows automatically in size
// when needed, but memory is never released again to avoid reoccuring
// heap allocations. There is no need to define the structure.
// The matrix will automatically accomodate an index (i,j) when you
// set() it.

AdjacencyMatrix::AdjacencyMatrix()
{

}

// Resets the matrix to all zeros.
// It overwrites every entry with zero so it's pretty costly.
void AdjacencyMatrix::clear()
{
    for (uint i = 0; i < m.size(); i++)
        for (uint j = 0; j < m[i].size(); j++)
            m[i][j] = 0;
}

// Set the matrix entry at (i,j) to 1.
// The matrix will automatically grow to accomodate an element i,j.
void AdjacencyMatrix::set(uint i, uint j)
{
    if (j > i)
    {
        uint temp = j;
        j = i;
        i = temp;
    }

    m.ensureSize(i+1);
    m[i].ensureSize(j+1);
    m[i][j] = 1;
}

// Set the matrix entry at (i,j) to 2.
// This operation will never cause the matrix to grow.
// No bound checking is performed, so if i,j is out of
// bounds, it segfaults and you only have yourself to blame.
void AdjacencyMatrix::check(uint i, uint j)
{
    if (j > i)
    {
        uint temp = j;
        j = i;
        i = temp;
    }

    m.ensureSize(i+1);
    m[i].ensureSize(j+1);
    m[i][j] = 2;
}

// Set the matrix entry at (i,j) to 0.
// This operation will never cause the matrix to grow.
// No bound checking is performed, so if i,j is out of
// bounds, it segfaults and you only have yourself to blame.
void AdjacencyMatrix::unset(uint i, uint j)
{
    if (j > i)
    {
        uint temp = j;
        j = i;
        i = temp;
    }

    if (i < m.size() && j < m[i].size())
        m[i][j] = 0;
}

// Checks if the matrix entry at (i,j) is set to 1.
// This operation will never cause the matrix to grow.
// No bound checking is performed, so if i,j is out of
// bounds, it segfaults and you only have yourself to blame.
bool AdjacencyMatrix::isSet(uint i, uint j) const
{
    if (j > i)
    {
        uint temp = j;
        j = i;
        i = temp;
    }

    if (i+1 > m.size() || j+1 > m[i].size())
        return false;
    return (m[i][j] > 0);
}

// Checks if the matrix entry at (i,j) is set to 0 (unset).
// This operation will never cause the matrix to grow.
// No bound checking is performed, so if i,j is out of
// bounds, it segfaults and you only have yourself to blame.
bool AdjacencyMatrix::isUnset(uint i, uint j) const
{
    if (j > i)
    {
        uint temp = j;
        j = i;
        i = temp;
    }

    if (i+1 > m.size() || j+1 > m[i].size())
        return false;
    return (m[i][j] == 0);
}

// Checks if the matrix entry at (i,j) is set to 2.
// This operation will never cause the matrix to grow.
// No bound checking is performed, so if i,j is out of
// bounds, it segfaults and you only have yourself to blame.
bool AdjacencyMatrix::isChecked(uint i, uint j) const
{
    if (j > i)
    {
        uint temp = j;
        j = i;
        i = temp;
    }

    if (i+1 > m.size() || j+1 > m[i].size())
        return false;
    return (m[i][j] > 1);
}

// Determines the size of the adjacency matrix.
// This is a somewhat costly operation.
Vec2u AdjacencyMatrix::size() const
{
    Vec2u s;
    s.x = m.size();
    for (uint i = 0; i < m.size(); i++)
        s.y = max(s.y, m[i].size());
    return s;
}
