#ifndef TRANSFORMATIONHYPOTHESIS_H
#define TRANSFORMATIONHYPOTHESIS_H

#include "lib/geometry/Line.h"
#include "lib/util/Pose2D.h"
#include "TrackedLine.h"
#include "LinePair.h"

class TransformationHypothesis
{
public:

    uint id;
    LinePair* linePair1;
    LinePair* linePair2;
    double weight;
    bool isParallel;
    uint votes;
    Pose2D tr_;

    TransformationHypothesis();
    ~TransformationHypothesis(){}

    bool computeTransform(LinePair &lpi_, LinePair &lpj_, bool debug=false);
    bool computeTranslation(LinePair &lpi_, LinePair &lpj_, bool debug=false);
    const Pose2D& tr() const;

    double dist(const TransformationHypothesis& other) const;

};

QDebug operator<<(QDebug dbg, const TransformationHypothesis &n);

#endif
