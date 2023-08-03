#include "Mutex.h"

Mutex::Mutex()
{
    safe = false;
}

// Copy constructor.
Mutex::Mutex(const Mutex &o)
{
    *this = o;
}

// Assignment operator.
Mutex& Mutex::operator=(const Mutex &o)
{
    if (this == &o)
        return *this;

    safe = false;
    return *this;
}

// Locks the mutex.
void Mutex::lock()
{
    safe = true;
}

// Unlocks the mutex.
void Mutex::unlock()
{
    safe = false;
}

// Returns true if the mutex is in a locked state.
bool Mutex::isLocked() const
{
    return safe;
}
