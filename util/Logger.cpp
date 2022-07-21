#include "util/Logger.h"

bool Logger::enabled = true;

Logger::Logger()
{
    append = false;
    verbose = false;
    blocked = false;
    separator = SPACE;
}

Logger::Logger(QString rl, bool append, const QString &separator)
{
    this->fileName = rl;
    this->append = append;
    this->separator = separator;
    open(fileName);
    blocked = false;
}

Logger::~Logger()
{
    flush();
    file.close();
}

void Logger::open(QString rl)
{
    if (!enabled) {
        return;
    }
    this->fileName = rl;
    if (file.isOpen())
    {
        flush();
        file.close();
    }
    file.setFileName(fileName);
    file.open((append ? QFile::Append : QFile::WriteOnly) | QFile::Text);
    stream.setDevice(&file);
}

// Sets the logger into append mode where output will be appended
// to the logfile rather than the logfile being overwritten. The
// append mode is off by default.
void Logger::setAppend(bool append)
{
    this->append = append;
    if (file.isOpen())
        open(fileName);
}

// Clears the contents of the logfile.
void Logger::clear()
{
    file.resize(0);
}

// Flushes unwritten parts of the text stream, if any, into the log file.
void Logger::flush()
{
    stream.flush();
}

void Logger::newLine()
{
    stream << "\n";
    stream.flush();
}

void Logger::setSeparator(const QString &separator)
{
    this->separator = separator;
}

void Logger::block()
{
    blocked = true;
}

QString Logger::getFileName() const
{
    return file.fileName();
}

Logger& Logger::operator<<(const Vec2& v)
{
    if (enabled)
    {
        QMutexLocker locker(&mutex);
        stream << " " << v.x << " " << v.y;
    }
    return *this;
}

Logger& Logger::operator<<(const Vec3& v)
{
    if (enabled)
    {
        QMutexLocker locker(&mutex);
        stream << " " << v.x << " " << v.y << " " << v.z;
    }
    return *this;
}

Logger& Logger::operator<<(const Vec2u& v)
{
    if (enabled)
    {
        QMutexLocker locker(&mutex);
        stream << " " << v;
    }
    return *this;
}

Logger& Logger::operator<<(const Vec3u& v)
{
    if (enabled)
    {
        QMutexLocker locker(&mutex);
        stream << " " << v;
    }
    return *this;
}

Logger& Logger::operator<<(const VecN<4>& v)
{
    if (enabled)
    {
        QMutexLocker locker(&mutex);
        stream << " " << v.x << " " << v.y << " " << v.z << " " << v[3];
    }
    return *this;
}

Logger& Logger::operator<<(const Pose2D& p)
{
    if (enabled)
    {
        QMutexLocker locker(&mutex);
        stream << " " << p.x << " " << p.y << " " << p.z;
    }
    return *this;
}

Logger& Logger::operator<<(const QString s)
{
    if (enabled)
    {
        QMutexLocker locker(&mutex);
        stream << s << separator ;
    }
    return *this;
}

Logger& Logger::operator<<(const double d)
{
    if (enabled)
    {
        QMutexLocker locker(&mutex);
        stream << d << separator ;
    }
    return *this;
}

Logger& Logger::operator<<(const float f)
{
    if (enabled)
    {
        QMutexLocker locker(&mutex);
        stream << f << separator ;
    }
    return *this;
}

Logger& Logger::operator<<(const int i)
{
    if (enabled)
    {
        QMutexLocker locker(&mutex);
        stream << i << separator ;
    }
    return *this;
}

Logger& Logger::operator<<(const uint i)
{
    if (enabled)
    {
        QMutexLocker locker(&mutex);
        stream << i << separator ;
    }
    return *this;
}

Logger &Logger::operator<<(const uint64_t i)
{
    if (enabled)
    {
        QMutexLocker locker(&mutex);
        stream << i << separator ;
    }
    return *this;
}

Logger& Logger::operator<<(const char c)
{
    if (enabled)
    {
        QMutexLocker locker(&mutex);
        stream << c << separator ;
    }
    return *this;
}

Logger &Logger::operator<<(const long l)
{
    if (enabled)
    {
        QMutexLocker locker(&mutex);
        stream << l << separator;
    }
    return *this;
}

Logger &Logger::operator<<(const ExperimentConfig &ex)
{
    if (enabled)
    {
        QMutexLocker locker(&mutex);
        stream << ex << separator;
    }
    return *this;
}

Logger &Logger::operator<<(const ExperimentConfig *ex)
{
    if (enabled)
    {
        QMutexLocker locker(&mutex);
        stream << ex << separator;
    }
    return *this;
}

Logger& Logger::operator++()
{
    if (enabled)
    {
        QMutexLocker locker(&mutex);
        stream << "\n";
        stream.flush();
    }
    return *this;
}

Logger& Logger::operator++(int c)
{
    if (enabled)
    {
        QMutexLocker locker(&mutex);
        stream << "\n";
        stream.flush();
    }
    return *this;
}
