#ifndef MUTEX_H_
#define MUTEX_H_

#include <atomic>
#include <QString>

class Mutex
{
    QString name;
    std::atomic<bool> safe;

public:

    Mutex();
    ~Mutex(){}

    Mutex(const Mutex &o);
    Mutex& operator=(const Mutex &o);
    void setName(const QString& n);

    void lock();
    void unlock();
    bool isLocked() const;
};

class MutexLocker
{
    Mutex* m;

public:

    MutexLocker(Mutex& arg) {m = &arg; m->lock();}
    ~MutexLocker(){m->unlock();}
};

#endif
