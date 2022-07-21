#ifndef LOGGER_H_
#define LOGGER_H_

#include <QFile>
#include <QTextStream>
#include <QMutex>

#include "util/Vec2.h"
#include "util/Vec3.h"
#include "util/Vec2u.h"
#include "util/Vec3u.h"
#include "util/VecN.h"
#include "util/Pose2D.h"
#include "ExperimentConfig.h"

#define SPACE " "
#define COMMA ","
#define TAB "    " // 4 spaces



class Logger
{

private:

    QFile file;
    QTextStream stream;
    bool append;
    QString fileName;
    QString separator;
    bool verbose;
    bool blocked;
    QMutex mutex;

public:

    Logger();
    Logger(QString rl, bool append = false, const QString& separator=SPACE);
    ~Logger();

    static bool enabled;

    void open(QString rl);
    void setAppend(bool append);
    void clear();
    void flush();
    void newLine();
    void setSeparator(const QString& separator);
    void block();

    QString getFileName() const;

    Logger& operator<<(const VecN<4>& v);
    Logger& operator<<(const Vec2& v);
    Logger& operator<<(const Vec3& v);
    Logger& operator<<(const Vec2u& v);
    Logger& operator<<(const Vec3u& v);
    Logger& operator<<(const Pose2D& v);
    Logger& operator<<(const ExperimentConfig &ex);
    Logger& operator<<(const ExperimentConfig *ex);

    Logger& operator<<(const QString s);
    Logger& operator<<(const double d);
    Logger& operator<<(const float f);
    Logger& operator<<(const int i);
    Logger& operator<<(const uint i);
    Logger& operator<<(const uint64_t i);
    Logger& operator<<(const char c);
    Logger& operator<<(const long l);
    Logger& operator++();
    Logger& operator++(int c);
};

#endif /* LOGGER_H_ */
