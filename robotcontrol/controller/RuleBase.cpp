#include "RuleBase.h"
#include "board/Config.h"
#include "board/Command.h"
#include "lib/util/DrawUtil.h"
#include <QFile>

RuleBase::RuleBase()
{

}

// Clears the rule base to a blank state.
void RuleBase::clear()
{
    rules.clear();
}

Vector<Rule> RuleBase::getRules() const
{
    return rules;
}

// Adds a rule to the base.
void RuleBase::addRule(const Rule &o)
{
    rules << o;
}

// Creates a new rule in the base with given rays (input), target (input), and the carrot (output).
bool RuleBase::addRule(const Vector<double> &rays, const Vec2& target, const Vec2 &carrot)
{
    // Discard rules with a zero output (carrot).
    if (target.isNull() || carrot.isNull())
        return false;

    // Discard rules where the target and the carrot point in "opposite" directions.
    if (fabs(target.angleTo(carrot)) > PI34)
        return false;

    // Discard rules that already exist, i.e. we can find a really close one.
    for (uint k = 0; k < rules.size(); k++)
    {
        if (rules[k].targetDistance(target) < config.rbTargetDistanceThreshold
                && rules[k].carrotDistance(carrot) < config.rbCarrotDistanceThreshold
                && rules[k].rayDistance(rays) < config.rbRayDistanceThreshold)
        {
//            if (debug > 0)
//                qDebug() << "rule discarded, because a similar rule already exists." << rules[k].targetDistance(target) << rules[k].carrotDistance(carrot) << rules[k].rayDistance(rays);
            return false;
        }
    }

    //qDebug() << "adding rule" << target << carrot << target.angleTo(carrot);

    Rule rule;
    rule.id = rules.size();
    rule.rays = rays;
    rule.target = target;
    rule.carrot = carrot;
    rules << rule;

    return true;
}

// Deletes the rule with the id.
void RuleBase::deleteRule(int id)
{
    for (uint i = 0; i < rules.size(); i++)
    {
        if (rules[i].id == id)
        {
            rules.removeAt(i);
            return;
        }
    }
}

// Deletes the currently best rule from the database.
void RuleBase::deleteCurrentRule()
{
    deleteRule(bestRule.id);
    query(inputRays, inputTarget);
}

// Returns the currently best rule.
const Rule &RuleBase::getCurrentRule()
{
    return bestRule;
}

// Returns the closest rule to the given rays and target.
Vec2 RuleBase::query(const Vector<double> &rays, const Vec2& target, int debug)
{
    inputRays = rays;
    inputTarget = target;

    // Preselect rules that have a small target distance.
    Vector<Rule> selectedRules;
    double minTargetDist = 100000000;
    for (uint k = 0; k < rules.size(); k++)
    {
        double td = rules[k].targetDistance(target);
        if (td < config.rbTargetDistanceThreshold)
            selectedRules << rules[k];
        if (td < minTargetDist)
            minTargetDist = td;
    }
    if (debug > 0)
        qDebug() << "preselected rules:" << selectedRules.size() << "min:" << minTargetDist;

    // Find the best matching rays.
    uint minIndex = 0;
    double minRayDist = 100000000;
    for (uint k = 0; k < selectedRules.size(); k++)
    {
        double raydist = selectedRules[k].rayDistance(rays);
        if (debug > 1)
            qDebug() << "  " << selectedRules[k].id << "target:" << selectedRules[k].target << selectedRules[k].targetDistance(target)
                     << "raydist:" << raydist << "carrot:" << selectedRules[k].carrot;
        if (raydist < minRayDist)
        {
            minRayDist = raydist;
            minIndex = k;
        }
    }

    if (selectedRules.size() > 0)
        bestRule = selectedRules[minIndex];

    if (debug > 0)
        qDebug() << "best rule:" << rules[minIndex];
    return bestRule.carrot;
}

// Returns the number of rules in the rule base.
uint RuleBase::size() const
{
    return rules.size();
}

// Saves the rule base to file.
void RuleBase::save(QString fileName) const
{
    QFile file(fileName);
    file.open(QIODevice::WriteOnly);
    QDataStream out(&file);
    out << rules;
    file.close();
}

// Loads the rule base from file.
void RuleBase::load(QString fileName)
{
    QFile file(fileName);
    file.open(QIODevice::ReadOnly);
    QDataStream in(&file);
    in >> rules;
    file.close();
}

// Returns a reference to the rule with the idx.
const Rule& RuleBase::getRule(uint idx) const
{
    return rules[idx];
}

// Draws the a visualization of the rule base controller on a QPainter.
void RuleBase::draw(QPainter *painter) const
{
    // The rays of the best matching rule.
    if (!command.showRayModel)
    {
        Vec2 base(1,0);
        for (int i = 0; i < bestRule.rays.size(); i++)
        {
            Vec2 ray = base.rotated(-config.raysAngleRange + i*2*config.raysAngleRange/(config.raysNumber-1));
            ray.normalize(bestRule.rays[i]);
            painter->save();
            painter->setBrush(drawUtil.brushRed);
            painter->setPen(drawUtil.penRedThin);
            painter->drawLine(QPointF(), ray);
            painter->drawEllipse(ray, 0.03, 0.03);
            painter->restore();
        }
    }

    // The target (input) of the best matching rule drawn in magenta.
    drawUtil.drawCross(painter, bestRule.target, drawUtil.pen, drawUtil.brushMagenta, 0.08);

    // The carrot (output) of the best matching rule drawn in yellow.
    drawUtil.drawCross(painter, bestRule.carrot, drawUtil.pen, drawUtil.brushYellow, 0.06);
}


// Writes the RuleBase into a data stream.
void RuleBase::streamOut(QDataStream &out) const
{
    out << rules;
}

// Reads the RuleBase from a data stream.
void RuleBase::streamIn(QDataStream &in)
{
    in >> rules;
}

QDataStream& operator<<(QDataStream& out, const RuleBase &o)
{
    o.streamOut(out);
    return out;
}

QDataStream& operator>>(QDataStream& in, RuleBase &o)
{
    o.streamIn(in);
    return in;
}

QDebug operator<<(QDebug dbg, const RuleBase &w)
{
    dbg << w.getRules();
    return dbg;
}
