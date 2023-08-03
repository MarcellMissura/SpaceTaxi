#include "AdjacencyMatrix.h"

// This is a memory preserving implementation of an adjacency matrix.
// Memory preserving means that the matrix grows automatically in size
// when needed, but memory is never released again to avoid reoccuring
// heap allocations. There is no need to define the structure.
// The matrix will automatically accomodate an index (i,j) when you
// call set(i,j). The entries of the matrix can be in one of three states:
// 0 - no edge between i and j. isSet(i,j) and isChecked(i,j) return false.
// 1 - unknown, isSet(i,j) is true and isChecked(i,j) is false.
// 2 - checked edge between i and j, isSet(i,j) and isChecked(i,j) both return true.
// If you check for an index (i,j) with isSet(i,j) or isChecked(i,j) outside
// of the allocated memory, false is returned. To manipulate the state of an edge,
// call set(i,j), unset(i,j) or check(i,j).

AdjacencyMatrix::AdjacencyMatrix()
{

}

// Resets the matrix to all zeros (no edge state).
// It overwrites every entry with zero so it's pretty costly.
void AdjacencyMatrix::clear()
{
    for (uint i = 0; i < m.size(); i++)
        for (uint j = 0; j < m[i].size(); j++)
            m[i][j] = 0;
}

// Set the matrix entry at (i,j) to 1 (unknown state).
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

// Set the matrix entry at (i,j) to 0 (no edge state).
// This operation will never cause the matrix to grow.
// An element (i,j) outside of the currently allocated
// size is always considered to be unset.
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

// Checks if the matrix entry at (i,j) is set to 1 (unknown state).
// For edges i,j that are checked, the function also returns true.
// This operation will never cause the matrix to grow.
// An element (i,j) outside of the currently allocated
// size is always considered to be unset.
bool AdjacencyMatrix::isSet(uint i, uint j) const
{
    if (j > i)
    {
        uint temp = j;
        j = i;
        i = temp;
    }

    if (i < m.size() && j < m[i].size())
        return (m[i][j] > 0);
    return false;
}

// Checks if the matrix entry at (i,j) is set to 0 (no edge state).
// This operation will never cause the matrix to grow.
// An element (i,j) outside of the currently allocated
// size is always considered to be unset.
bool AdjacencyMatrix::isUnset(uint i, uint j) const
{
    if (j > i)
    {
        uint temp = j;
        j = i;
        i = temp;
    }

    if (i < m.size() && j < m[i].size())
        return (m[i][j] < 1);
    return true;
}

// Set the matrix entry at (i,j) to 2 (known edge state).
// The matrix will automatically grow to accomodate an element i,j.
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

// Checks if the matrix entry at (i,j) is set to 2 (known edge state).
// This operation will never cause the matrix to grow.
// An element (i,j) outside of the currently allocated size
// is always considered to be unchecked.
bool AdjacencyMatrix::isChecked(uint i, uint j) const
{
    if (j > i)
    {
        uint temp = j;
        j = i;
        i = temp;
    }

    if (i < m.size() && j < m[i].size())
        return (m[i][j] > 1);
    return false;
}

// Determines the size of the adjacency matrix, which is
// defined by the highest index i,j used so far.
// This is a somewhat costly operation.
Vec2u AdjacencyMatrix::size() const
{
    Vec2u s;
    s.x = m.size();
    for (uint i = 0; i < m.size(); i++)
        s.y = max(s.y, m[i].size());
    return s;
}
