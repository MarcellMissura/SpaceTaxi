#ifndef RULEBASE_H_
#define RULEBASE_H_
#include "controller/Rule.h"
#include "util/Vector.h"
#include <QPainter>
#include <QDebug>

class RuleBase
{
    Vector<Rule> rules;
    Rule bestRule;

    int debug;
public:

    RuleBase();
    ~RuleBase(){}

    // Model construction and query methods.
    void setDebug(uint d);
    void clear();

    Vector<Rule> getRules() const;
    void addRule(const Rule& rule);
    bool addRule(const Vector<double>& rays, const Vec2 target, const Vec2& carrot);
    void deleteRule(int id);
    void deleteCurrentRule();
    const Rule& getCurrentRule();
    Vec2 query(const Vector<double>& rays, const Vec2 target);

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
