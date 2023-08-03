#ifndef RULE_H_
#define RULE_H_

#include "lib/util/Vector.h"
#include "lib/util/Vec2.h"

class Rule
{
public:

    int id;
    Vector<double> rays; // input
    Vec2 target; // input
    Vec2 carrot; // output

    Rule();
    ~Rule();

    double rayDistance(const Vector<double>& rs) const;
    double targetDistance(const Vec2& tar) const;
    double carrotDistance(const Vec2& car) const;
};

QDebug operator<<(QDebug dbg, const Rule &n);
QDataStream& operator<<(QDataStream& out, const Rule &o);
QDataStream& operator>>(QDataStream& in, Rule &o);

#endif
