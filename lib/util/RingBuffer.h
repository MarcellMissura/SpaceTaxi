#ifndef RINGBUFFER_H
#define RINGBUFFER_H
#include <QDebug>
#include "Vector.h"
#include "globals.h"

// The RingBuffer class is a memory-preserving container of objects that are arranged in a contiguous
// space in memory. The RingBuffer typically has a fixed size (if you set one) and discards the oldest
// element in the container in order to make room for the next one. The [0] operator and the last()
// function return the last element that was added to the buffer. Operator [k] accesses the older elements.
// As long as you don't set the max size, the RingBuffer behaves just like a vector. After you set the
// max size, or resize the buffer, the size will remain fixed and the ring buffering takes effect.
template <typename T>
class RingBuffer
{
    Vector<T> d; // Reusing most of the functionality of Vector.
    uint bufferOffset = 0;
    uint maxSize = 0; // 0 means no max size is set. It HAS to be set at construction.

public:

    RingBuffer() {}

    // Instantiates a RingBuffer of max size k.
    RingBuffer(uint k) {maxSize = k;}

    void setMaxSize(uint m) {maxSize = m;}

    // Returns the actual number of objects in the RingBuffer.
    // This is not the same as the maximum size.
    uint size() const {return d.size();}

    // Returns true if there are no objects in the RingBuffer. Otherwise it returns false.
    bool isEmpty() const {return d.isEmpty();}
    bool empty() const {return isEmpty();} // Same as isEmpty().

    // Returns the object stored in position i.
    // i = 0 returns the latest object in the buffer.
    // i = 1 returns the one before that, and so on.
    // if i > max size, the index wraps.
    // Segfaults if you request an item size < i < maxSize.
    T& operator[](uint i) {return d[mod(bufferOffset-i, maxSize)];}
    const T& operator[](uint i) const {return d[mod(bufferOffset-i, maxSize)];}
    const T& at(uint i) const {return d.at(mod(bufferOffset-i, maxSize));}

    // It clears the RingBuffer, i.e. sets the size to 0. This is a very fast O(1) operation that leaves the stored
    // objects untouched in memory. Note that the destructor of the stored objects is not called.
    void clear() {d.clear(); bufferOffset = 0;}

    // Appends an object to the end of the RingBuffer. This is a fast, amortized O(1) operation that reuses already
    // allocated memory. If the size exceeds the capacity, the entire RingBuffer needs to be reallocated.
    void push(T const& e)
    {
        // No buffer size set.
        if (maxSize == 0)
        {
            if (!d.isEmpty())
                bufferOffset++;
            d.push_back(e);
            //qDebug() << "0_push offset:" << bufferOffset << d;
            return;
        }

        // The buffer is smaller than the maximum size.
        if (size() < maxSize)
        {
            if (!d.isEmpty())
                bufferOffset++;
            d.push_back(e);
            //qDebug() << "s_push offset:" << bufferOffset << d;
            return;
        }

        // Append to ring buffer.
        bufferOffset = (bufferOffset + 1) % size();
        d[bufferOffset] = e;

        //qDebug() << "push offset:" << bufferOffset << d;
    }

    RingBuffer<T>& operator<<(T const& e) {push(e); return *this;} // Same as push_back().


    void streamOut(QDataStream& out) const
    {
        out << maxSize;
        out << bufferOffset;
        out << d;
    }

    void streamIn(QDataStream &in)
    {
        in >> maxSize;
        in >> bufferOffset;
        in >> d;
    }
};



// QDebug output.
template <typename T>
QDebug operator<<(QDebug dbg, const RingBuffer<T> *o)
{
    dbg << "sz:" << o->size() << " | ";
    dbg << "[";
    if  (o->size() > 1)
        for (uint i = 0; i < o->size()-1; i++)
            dbg << &o->at(i) << ",";
    if (o->size() > 0)
        dbg << &o->at(o->size()-1);
    dbg << "]";
    return dbg;
}

// QDebug output.
template <typename T>
QDebug operator<<(QDebug dbg, const RingBuffer<T> &o)
{
    dbg << "sz:" << o.size() << " | ";
    dbg << "[";
    for (uint i = 0; i < o.size(); i++)
        dbg << o[i] << "\n";
    dbg << "]";
    return dbg;
}

// Streams the content of the RingBuffer into the QDataStream.
template <typename T>
QDataStream& operator<<(QDataStream& out, const RingBuffer<T> &o)
{
    o.streamOut(out);
    return out;
}

// Streams the contents of the QDataStream into the RingBuffer.
template <typename T>
QDataStream& operator>>(QDataStream& in, RingBuffer<T> &o)
{
    o.streamIn(in);
    return in;
}

#endif
