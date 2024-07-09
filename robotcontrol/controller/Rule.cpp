#include "Rule.h"

Rule::Rule()
{
    id = -1;
}

Rule::~Rule()
{

}

// Computes the ray distance between this rule and the given rays.
double Rule::rayDistance(const Vector<double>& rs) const
{
    double raydist = 0;
    for (uint i = 0; i < rays.size(); i++)
        raydist += (rays[i]-rs[i])*(rays[i]-rs[i]);
    return raydist;
}

// Computes the target distance between this rule and the given target.
double Rule::targetDistance(const Vec2 &tar) const
{
    return (target-tar).norm();
}

// Computes the carrot distance between this rule and the given carrot.
double Rule::carrotDistance(const Vec2 &car) const
{
    return (carrot-car).norm();
}

QDataStream& operator<<(QDataStream& out, const Rule &o)
{
    out << o.id;
    out << o.rays;
    out << o.target;
    out << o.carrot;
    return out;
}

QDataStream& operator>>(QDataStream& in, Rule &o)
{
    in >> o.id;
    in >> o.rays;
    in >> o.target;
    in >> o.carrot;
    return in;
}

QDebug operator<<(QDebug dbg, const Rule &o)
{
    dbg << o.id /*<< o.rays*/ << "target:" << o.target << "carrot:" << o.carrot;
    return dbg;
}
