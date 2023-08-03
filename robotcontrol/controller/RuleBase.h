#ifndef RULEBASE_H_
#define RULEBASE_H_
#include "robotcontrol/controller/Rule.h"
#include "lib/util/Vector.h"
#include <QPainter>
#include <QDebug>

class RuleBase
{
    Vector<Rule> rules;
    Rule bestRule;

    Vector<double> inputRays;
    Vec2 inputTarget;

public:

    RuleBase();
    ~RuleBase(){}

    void clear();

    Vector<Rule> getRules() const;
    void addRule(const Rule& rule);
    bool addRule(const Vector<double>& rays, const Vec2 &target, const Vec2& carrot);
    void deleteRule(int id);
    void deleteCurrentRule();
    const Rule& getCurrentRule();
    Vec2 query(const Vector<double>& rays, const Vec2 &target, int debug=0);

    const Rule& getRule(uint idx) const;
    uint size() const;

    void save(QString fileName) const;
    void load(QString fileName);

    void draw(QPainter* painter) const;

    void streamOut(QDataStream &out) const;
    void streamIn(QDataStream &in);
};

QDebug operator<<(QDebug dbg, const RuleBase &w);
QDataStream& operator<<(QDataStream& out, const RuleBase &o);
QDataStream& operator>>(QDataStream& in, RuleBase &o);

#endif
