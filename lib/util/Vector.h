#ifndef VECTOR_H
#define VECTOR_H
#include <QDataStream>
#include <QDebug>
#include "lib/util/LinkedList.h"

// The Vector class is a memory-preserving container of objects that are arranged in a contiguous
// space in memory. Since memory allocation is an expensive operation, the Vector class has been
// designed to grow as needed, but to never shrink and release memory. The Vector class wraps a
// std::vector and apart from slightly changing the behavior of std::vector, it adds to it a nicer,
// Qt compatible interface. The following sums up the behavior of the Vector class.
// 1. The Vector class is templated and you can use it to store any object: Vector<YourClass> v;
// 2. You can push objects into it and it will allocate memory as needed. v << yourObject;
//    Memory allocation may result in a copy of the entire Vector. For this reason, it is best
//    if you never use a pointer to an element of a Vector. Use index-based access instead.
// 3. When you erase an element or when you clear() the Vector, it does not free its memory. The
//    capacity() remains the same. Contrary to the std::vector, the destructors of removed elements
//    are NOT called. When the container is filled again, already allocate memory is simply overwritten.
// 4. The default assignment performs a deep copy of the data.

template <typename T>
class Vector
{
    std::vector<T> d; // Reusing most of the functionality of std::vector.
    int tailIdx = -1; // The index of the last element in the Vector. Not the same as the size of d.

public:

    // Default constructor.
    Vector() {}

    // Instantiates a Vector of k default constructed elements.
    Vector(uint k) {resize(k);}

    // LinkedList assignment.
    Vector(const LinkedList<T>& o) {*this = o;}
    void operator=(const LinkedList<T>& v)
    {
        clear();
        ListIterator<T> it = v.begin();
        while (it.hasNext())
            push_back(it.next());
    }

    // Returns the capacity of the Vector, i.e. the number of objects memory has been allocated for.
    // The capacity is larger or equal to the actual size.
    int capacity() const {return d.capacity();}

    // Reserves capacity for k number of objects. It does not change the size of the Vector.
    // Since the capacity of the Vector grows automatically with the number of inserted items,
    // reserve() should only be used if you know the expected size in advance.
    void reserve(uint k) {d.reserve(k);}

    // Returns the size of the Vector, i.e. the number of objects in the Vector. This is not the same as the capacity.
    uint size() const {return tailIdx+1;}
    uint length() const {return size();} // Same as size().

    // Resizes the Vector to hold k number of objects. If the current size is greater than k, the size simply shrinks
    // to k and the superflous objects are left in memory. If the current size is smaller than k, the size grows to k.
    // If present, old objects are regained for preserving memory and left in an undefined state. Otherwise default
    // constructed objects are appended.
    void resize(uint k) {if (size() < k) d.resize(k); tailIdx=k-1;}

    // Makes sure the size of the Vector is at least k. If the size is smaller than k, it resizes the Vector to k.
    // If the size is larger or equal to k, nothing is done. Use this only if you actually need k objects to be
    // present in the Vector! Otherwise you want reserve().
    void ensureSize(uint k) {if (size() < k) resize(k);}

    // Returns true if there are no objects in the Vector. Otherwise it returns false.
    bool isEmpty() const {return (tailIdx == -1);}
    bool empty() const {return isEmpty();} // Same as isEmpty().

    // Returns the object stored in position i.
    T& operator[](uint i) {return d[i];}
    const T& operator[](uint i) const {return d[i];}
    const T& at(uint i) const {return d.at(i);}

    // It clears the Vector, i.e. sets the size to 0. This is a very fast O(1) operation that leaves the stored
    // objects untouched in memory. Note that this behavior differs from std::vector because the destructor of the
    // stored objects is not called.
    void clear() {tailIdx=-1;}

    // Inserts an object at the front of the Vector. This is an expensive O(N) operation that requires allocation
    // of new memory and copying of all objects in the Vector and a possible reallocation of the entire Vector.
    void push_front(T const& e) {d.insert(d.begin(), e); tailIdx++;}
    void prepend(T const& e) {push_front(e);}

    // Appends an object to the end of the Vector. This is a fast, amortized O(1) operation that reuses already
    // allocated memory. If the size exceeds the capacity, the entire Vector needs to be reallocated.
    void push_back(T const& e)
    {
        tailIdx++;
        if ((uint)tailIdx < d.size())
        {
            // Reuse already allocated memory (fast).
            d[tailIdx] = e;
        }
        else
        {
            // Also reuses memory, but may reallocate the Vector (very slow).
            d.push_back(e);
        }
    }
    void append(T const& e) {push_back(e);} // Same as push_back().
    Vector<T>& operator<<(T const& e) {push_back(e); return *this;} // Same as push_back().

    // Appends the Vector o to this one.
    Vector<T>& operator<<(const Vector<T> &o)
    {
        for (uint i=0; i < o.size(); i++)
            push_back(o[i]);
        return *this;
    }

    // Returns a subset of the Vector including the elements at startIdx and endIdx.
    Vector<T> subSet(uint startIdx, uint endIdx) const
    {
        Vector<T> v;
        for (uint i = startIdx; i <= endIdx; i++)
            v << d[i];
        return v;
    }

    // Unifies this Vector with item o in a way that the item appears only once in the Vector.
    // o is pushed only if it is not already contained.
    void unify(const T& o)
    {
        if (!contains(o))
            push_back(o);
        return;
    }

    // Unifies this Vector with Vector o in a way that any element of o appears only once in the result.
    void unify(const Vector<T>& o)
    {
        for (uint i = 0; i < o.size(); i++)
            if (!contains(o[i]))
                push_back(o[i]);
        return;
    }

    // Unifies this Vector with LinkedList o in a way that any element of o appears only once in the result.
    void unify(const LinkedList<T>& o)
    {
        ListIterator<T> it = o.begin();
        while (it.hasNext())
        {
            if (!contains(it.cur()))
                push_back(it.cur());
            it.next();
        }
        return;
    }

    // Inserts an object at index i. This is an expensive O(N) operation because all subsequent objects are
    // copied one slot down and may result in the reallocation of the entire Vector.
    void insert(uint i, T const& e) {d.insert(i, e); tailIdx++;}

    // Fills the Vector with a copies of the provided object e.
    // The vector is filled only up to the current size(), not the entire available memory.
    void fill(T const& e)
    {
        for (uint i = 0; i < size(); i++)
            d[i] = e;
    }

    // Returns the first object in the Vector. Same as [0].
    T& first() {return d[0];}
    const T& first() const {return d[0];}

    // Returns the last object in the Vector. Same as [size()-1].
    T& last() {return d[size()-1];}
    const T& last() const {return d[size()-1];}

    // Returns a random object in the Vector.
    T& random() {return d[((double)rand()/RAND_MAX)*size()];}
    const T& random() const {return d[((double)rand()/RAND_MAX)*size()];}

    // Removes the first object from the Vector and returns it. This is an expensive O(N) operation that calls
    // the assignment operator of all objects in the Vector because they are moved up by one slot.
    T pop_front() {T e = d[0]; d.erase(d.begin()); tailIdx--; return e;}
    T takeFirst() {return pop_front();}

    // Removes the last object from the Vector and returns it. This is a very fast O(1) operation.
    T pop_back() {tailIdx--; return d[size()];}
    T takeLast() {return pop_back();}
    T removeLast() {return pop_back();}

    // Erases the object in position i from the Vector. This is a slow O(N) operation.
    void remove(uint i) {d.erase(d.begin()+i); tailIdx--;}
    void removeAt(uint i) {remove(i);}

    // Removes all occurences of t from the Vector. It uses the == operator to compare objects. It's a very slow operation.
    void removeAll(const T& t)
    {
        for (int i = size()-1; i >= 0; i--)
            if (d[i] == t)
                remove(i);
    }

    // Removes the first occurence of t from the Vector. It uses the == operator to compare objects. It's a slow operation.
    void removeOne(const T& t)
    {
        for (uint i = 0; i < size(); i++)
        {
            if (d[i] == t)
            {
                remove(i);
                return;
            }
        }
    }

    // Returns a raw pointer to the data.
    const T* data() const {return d.data();}

    // Swaps the objects between slot i and j.
    void swap(uint i, uint j) {T e = d[i]; d[i]=d[j]; d[j]=e;}

    // Returns true if the Vector contains at least one occurance of t, i.e. at least one object evaluates the == operator to true.
    bool contains(const T& t) const
    {
        for (uint i = 0; i < size(); i++)
            if (d[i] == t)
                return true;
        return false;
    }

    // Returns true if this Vector contains all elements of LinkedList o, i.e. the == operator
    // evaluates to true for all elements of o at least with one element of this Vector.
    bool contains(const LinkedList<T>& o) const
    {
        // Empty list case.
        if (isEmpty())
            return false;

        ListIterator<T> it = o.begin();
        while (it.hasNext())
            if (!contains(it.next()))
                return false;
        return true;
    }

    // Returns true if this Vector contains all elements of Vector o, i.e. the == operator
    // evaluates to true for all elements of o at least with one element of this Vector.
    bool contains(const Vector<T>& o) const
    {
        // Empty list case.
        if (isEmpty())
            return false;

        for (uint i = 0; i < o.size(); i++)
            if (!contains(o[i]))
                return false;
        return true;
    }

    // Returns the index of the first element that evalues the == operator with t to true.
    // If no such element is found, -1 is returned.
    int indexOf(const T& t) const
    {
        for (uint i = 0; i < size(); i++)
            if (d[i] == t)
                return i;
        return -1;
    }

    // Returns true if o has the same size() as this Vector and all elements of Vector o evaluate
    // the == operator to true with all elements of this Vector at the same index.
    bool operator==(const Vector<T>& o) const
    {
        if (size() != o.size())
            return false;
        for (uint i = 0; i < size(); i++)
            if (at(i) != o.at(i))
                return false;
        return true;
    }

    // Returns true if o is of a different size() than this Vector or if at least one element of Vector o
    // evaluates the == operator to false with the element of this Vector at the same index.
    bool operator!=(const Vector<T>& o) const
    {
        if (size() != o.size())
            return true;
        for (uint i = 0; i < size(); i++)
            if (at(i) != o.at(i))
                return true;
        return false;
    }

    // Sorts the objects in the Vector in ascending (default) or descending (when direction = -1) order using the < or the > operator.
    void sort(int direction=1)
    {
        if (direction < 0)
            std::sort(d.begin(), d.begin()+size(), std::greater<T>());
        else
            std::sort(d.begin(), d.begin()+size());
    }

    // Reverses the order of the items in the Vector. This is an O(N) operation.
    void reverse()
    {
        T tmp;
        for (uint i = 0; i < size()/2; i++)
        {
            tmp = d[i];
            d[i] = d[size()-1-i];
            d[size()-1-i] = tmp;
        }
    }

    // Sorts the vector into a random order.
    void randomOrder()
    {
        for (uint i = 0; i < size(); i++)
            swap(i, ((double)rand()/RAND_MAX)*size());
    }

};

// QDebug output.
template <typename T>
QDebug operator<<(QDebug dbg, const Vector<T> *o)
{
    dbg << "sz:" << o->size() << " | ";
    dbg << "[";
    if  (o->size() > 1)
        for (uint i = 0; i < o->size()-1; i++)
            dbg << &o->at(i) << ",";
    if (o->size() > 0)
        dbg << &o->last();
    dbg << "]";
    return dbg;
}


// QDebug output.
template <typename T>
QDebug operator<<(QDebug dbg, const Vector<T> &o)
{
    dbg << "sz:" << o.size() << " | ";
    dbg << "[";
    for (uint i = 0; i < o.size(); i++)
        dbg << o[i] << "\n";
    dbg << "]";
    return dbg;
}

// Streams the content of the Vector into the QTextStream.
template <typename T>
QTextStream& operator<<(QTextStream& out, const Vector<T> &o)
{
    for (uint i = 0; i < o.size(); i++)
        out << o[i] << "\n";
    return out;
}

// Streams the content of the Vector into the QDataStream.
template <typename T>
QDataStream& operator<<(QDataStream& out, const Vector<T> &o)
{
    out << o.size();
    for (uint i = 0; i < o.size(); i++)
        out << o[i];
    return out;
}

// Streams the contents of the QDataStream into the Vector.
template <typename T>
QDataStream& operator>>(QDataStream& in, Vector<T> &o)
{
    uint k;
    in >> k;
    o.resize(k);
    for (uint i=0; i < k; i++)
        in >> o[i];
    return in;
}

#endif
