#ifndef HOLONOMICAGENT_H_
#define HOLONOMICAGENT_H_
#include "HolonomicObstacle.h"
#include "controller/PathAStar.h"
#include "controller/LazyThetaStar.h"
#include "controller/HolonomicDWA.h"
#include "controller/VisibilityGraph.h"
#include "GeometricModel.h"
#include "GridModel.h"
#include <QPainter>

class HolonomicAgent : public HolonomicObstacle
{
    int agentId;
    Hpm2D mainTarget;
    Hpm2D intermediateTarget;

    GeometricModel geometricModel;
    GridModel gridModel;

    Hpm2D currentState;
    int targetDropOffId;

    PathAStar pathAstar;
    LazyThetaStar lazyThetaStar;
    VisibilityGraph visibilityGraph;
    HolonomicDWA holonomicDWA;

public:

    HolonomicAgent();
    ~HolonomicAgent(){}

public:

    virtual const QString getName() const;

    void init(const Vec2& p);

    void sense();
    void act();
    void step();

    void collisionResponse(const Obstacle* o);
    void draw(QPainter* painter) const;
};

#endif
