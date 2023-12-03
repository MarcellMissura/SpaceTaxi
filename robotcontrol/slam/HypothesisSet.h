#ifndef HYPOTHESISSET_H_
#define HYPOTHESISSET_H_

#include "TransformationHypothesis.h"

class HypothesisSet
{
    Vector<TransformationHypothesis> hyps;
    mutable Pose2D _avgTransform;
    double _snapQuality;

public:

    HypothesisSet();
    ~HypothesisSet(){}

    void clear();
    bool isEmpty() const;
    uint size() const;

    void set(const Vector<TransformationHypothesis> v);
    const Vector<TransformationHypothesis>& getHyps() const;

    const Vector<HypothesisSet>& cluster(double eps, bool debug=false) const;
    const Vector<HypothesisSet>& splitConflictFreeSubClusters(bool debug=false) const;
    void discardOutliers(double eps, bool debug=false);
    double snapQuality(const Vector<TrackedLine> &inputLines, bool debug=false);
    double getSnapQuality() const;
    const Pose2D &avgTransform(bool recompute=false) const;
    Vector<LinePair> getConfirmedLinePairs();

    bool isInConflictWith(const TransformationHypothesis& tr) const;

    bool operator==(const HypothesisSet &o) const {return hyps == o.hyps;}
    bool operator!=(const HypothesisSet &o) const {return hyps != o.hyps;}
    bool operator<(const HypothesisSet &o) const {if (_snapQuality == 0) {return size() < o.size();} else {return _snapQuality < o._snapQuality;}}
    bool operator>(const HypothesisSet &o) const {if (_snapQuality == 0) {return size() > o.size();} else {return _snapQuality > o._snapQuality;}}

    // Drawing methods.
    void draw() const;
    void draw(QPainter* painter) const;

    HypothesisSet& operator<<(const TransformationHypothesis &th);
    HypothesisSet& operator<<(const Vector<TransformationHypothesis> &th);
};

QDebug operator<<(QDebug dbg, const HypothesisSet &w);
QDebug operator<<(QDebug dbg, const HypothesisSet *w);

#endif
