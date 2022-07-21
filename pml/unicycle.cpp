#include "unicycle.h"
#include "globals.h"
#include "util/fresnelnr.h"
#include "blackboard/Config.h"
#include "util/GLlib.h"

// This is a unicycle model that supports prediction, intersection, and conversion
// functionalities. The state of a unicycle is the position x,y, the orientation
// theta, the linear (forward) velocity v, and the angular velocity omega (w). The
// controls of a unicycle are the accelerations a and b. a accelerates the linear
// velocity v and b accelerates the angular velocity omega.

Unicycle::Unicycle()
{
    dt = 0;
    x = 0;
    y = 0;
    theta = 0;
    v = 0;
    w = 0;
    a = 0;
    b = 0;
    boundingBoxValid = false;
}

Unicycle::Unicycle(double x, double y, double theta, double v, double omega)
{
    this->dt = 0;
    this->x = x;
    this->y = y;
    this->theta = theta;
    this->v = v;
    this->w = omega;
    this->a = 0;
    this->b = 0;
    boundingBoxValid = false;
}

// Holonomic to unicycle conversion.
Unicycle::Unicycle(const Hpm2D& o)
{
    *this = o;
    boundingBoxValid = false;
}

// Holonomic to unicycle conversion.
Unicycle& Unicycle::operator=(const Hpm2D& o)
{
    // The conversion between the holonomic model and the unicycle model
    // equates the direction of the holonomic velocity vector with the
    // orientation theta of the unicycle. The linear (forward) velocity
    // v of the unicycle therefore equals the magnitude of the holonomic
    // velocity vector v = sqrt(vx²+vy²). Consequently, the linear
    // acceleration a of the unicycle must be equal to the "growth" of the
    // holonomic velocity. i.e. a = (d/dt)sqrt(vx(t)², vy(t)²). Respectively,
    // the angular velocity w of the unicycle is equivalent to the first
    // derivative of the angle of the holonomic velocity vector:
    // w = (d/dt)atan(vx(t), vy(t)). The angular acceleration b of the
    // unicycle b = (d/dt)² atan(vx(t), vy(t)) is then simply the second
    // derivative of the angle of the holonomic velocity vector. The same
    // set of equations is used in both directions, we are just given
    // different parameters.

    // The conversion from holo to unicycle is mathematically unproblematic
    // in the sense that it is easy to analytically match the current state
    // of motion between a holonomic and a unicycle vehicle. However, the
    // extrapolation of the motion with the controls remaining constant is
    // unprecise and the holonomic model will quickly diverge from the unicycle.

    // vx(t) = vx+ax*t+0.5*jx*t*t
    // vy(t) = vy+ay*t+0.5*jy*t*t
    // w = (d/dt)atan(vx(t), vy(t))
    // v = sqrt(vx^2, vy^2)
    // a = (d/dt)sqrt(vx(t)^2+vy(t)^2)
    // b = (d/dt)²atan(vx(t), vy(t))

    dt = o.dt;
    x = o.x;
    y = o.y;

    if (o.vel().isNull() && o.acc().isNull() && o.jerk().isNull())
    {
        theta = 0;
        v = 0;
        w = 0;
        a = 0;
        b = 0;
    }
    else if (o.vel().isNull() && o.acc().isNull())
    {
        theta = atan2(o.jy, o.jx); // Angle of the jerk vector.
        v = 0;
        w = 0;
        a = 0;
        b = 0;
    }
    else if (o.vel().isNull())
    {
        theta = atan2(o.ay, o.ax); // Angle of the acceleration vector.
        v = 0;
        w = 0;
        a = o.acc().norm();
        b = 0;
    }
    else
    {
        double n2 = o.vx*o.vx+o.vy*o.vy;
        theta = atan2(o.vy, o.vx); // Angle of the velocity vector.
        v = sqrt(n2); // Magnitude of the velocity vector.
        w = (o.vx*o.ay-o.vy*o.ax)/n2; // First derivative of the angle of the velocity vector atan2(vx(t), vy(t)) with respect to time at t=0. Should match the curvature.
        a = (o.vx*o.ax+o.vy*o.ay)/v; // First derivative of the magnitude of the velocity vector sqrt(vx(t)², vy(t)²) with respect to time at t=0.
        b = (o.jy*o.vx-o.jx*o.vy)/n2 + 2.0*(o.ax*o.vy-o.ay*o.vx)*(o.ax*o.vx+o.ay*o.vy) / (n2*n2); // Second derivative of the angle of the velocity vector atan2(vx(t), vy(t)) with respect to t.
    }

    boundingBoxValid = false;

//    qDebug() << "Holonomic to Unicycle";
//    qDebug() << "Holo:" << o;
//    qDebug() << "Uni:" << *this;

    return *this;
}

// Unicycle to holonomic conversion.
Unicycle::operator Hpm2D() const
{
    // The conversion between the holonomic model and the unicycle model
    // equates the direction of the holonomic velocity vector with the
    // orientation theta of the unicycle. The linear (forward) velocity
    // v of the unicycle therefore equals the magnitude of the holonomic
    // velocity vector v = sqrt(vx²+vy²). Consequently, the linear
    // acceleration a of the unicycle must be equal to the "growth" of the
    // holonomic velocity. i.e. a = (d/dt)sqrt(vx(t)², vy(t)²). Respectively,
    // the angular velocity w of the unicycle is equivalent to the first
    // derivative of the angle of the holonomic velocity vector:
    // w = (d/dt)atan(vx(t), vy(t)). The angular acceleration b of the
    // unicycle b = (d/dt)² atan(vx(t), vy(t)) is then simply the second
    // derivative of the angle of the holonomic velocity vector. The same
    // set of equations is used for the conversion in both directions, we
    // are just given different parameters.

    // However, the issue with the conversion from unicycle to holonomic
    // is that after assigning position and velocity, the unicycle
    // parameters w and a are used to compute the holonomic acceleration
    // (ax, ay). Now only b is left to compute the jerk parameters (jx,jy),
    // which means we have an underdetermined system of equations. Also,
    // the jerk is a control parameter that will be changed by a controller,
    // so there is not much sense in computing them. In conclusion, when
    // converting a unicycle to a hpm, we obtain a holonomic vehicle state
    // that represents the unicycle state including x,y,theta,v, and a, but
    // not the angular acceleration b.

    // vx(t) = vx+ax*t+0.5*jx*t*t
    // vy(t) = vy+ay*t+0.5*jy*t*t
    // w = (d/dt)atan(vx(t), vy(t))(0)
    // v = sqrt(vx^2, vy^2)


    Hpm2D h;
    h.dt = dt;

    // The position is trivial.
    h.x = x;
    h.y = y;

    // The velocity is straight forward.
    double c = fcos(theta);
    double s = fsin(theta);
    h.vx = v*c;
    h.vy = v*s;

    // The accelerations are gained from setting the first derivative of the
    // angle of the velocity vector with respect to time equal to omega and
    // setting the first derivate of the length of the vector with respect
    // to time equal equal to a, and solving these equations for ax and ay.
    double s1 = h.vx*h.vx+h.vy*h.vy;
    double s2 = sqrt(s1);
    double k1 = sgn(v)*a*s2*h.vy/h.vx;
    double k2 = (h.vy*h.vy)/h.vx;
    double ay = (w*s1+k1)/(h.vx+k2);
    double ax = (sgn(v)*a*s2-ay*h.vy)/h.vx;
    h.ax = ax;
    h.ay = ay;
    if (fabs(v) < EPSILON)
    {
        h.ax = a*c;
        h.ay = a*s;
    }

    // The jerks are not computed and remain zero.

//    qDebug() << "Unicycle to Holonomic";
//    qDebug() << "Uni:" << *this;
//    qDebug() << "Holo:" << h;

    return h;
}

// Sets the (x,y) position of the Unicycle.
void Unicycle::setPos(double x, double y)
{
    aabb.translate(x-this->x, y-this->y);
    this->x = x;
    this->y = y;
}

// Sets the (x,y) position of the Unicycle.
void Unicycle::setPos(const Vec2& p)
{
    aabb.translate(p.x-x, p.y-y);
    this->x = p.x;
    this->y = p.y;
}

// Returns the (x,y) position of the Unicycle.
Vec2 Unicycle::pos() const
{
    return Vec2(x, y);
}

// Translates the Unicycle by (dx,dy).
void Unicycle::translate(double dx, double dy)
{
    setPos(this->x+dx, this->y+dy);
}

// Translates the Unicycle by d.
void Unicycle::translate(const Vec2 &d)
{
    setPos(this->x+d.x, this->y+d.y);
}

// Rotates the Unicycle by the angle phi given in radians. This is a geometric rotation
// around the origin of the reference frame that also updates the heading. If you just
// want to turn, use turn() for that.
void Unicycle::rotate(double phi)
{
    setPos(pos().frotated(phi));
    turn(phi);
}

// Returns a rotated Unicycle by the angle phi given in radians. This is a geometric rotation
// around the origin of the reference frame that also updates the heading. If you just
// want to turn, use turn() for that.
Unicycle Unicycle::rotated(double phi) const
{
    Unicycle u = *this;
    u.rotate(phi);
    return u;
}

// Returns the heading theta of the Unicycle.
double Unicycle::heading() const
{
    return theta;
}

// Sets the heading theta of the Unicycle.
void Unicycle::setHeading(double theta)
{
    this->theta = theta;
    boundingBoxValid = false;
}

// Turns the unicycle by increment a. This simply changes the heading.
void Unicycle::turn(double dtheta)
{
    theta += dtheta;
    boundingBoxValid = false;
}

// Returns the (x,y, theta) pose of the Unicycle.
Pose2D Unicycle::pose() const
{
    return Pose2D(x, y, theta);
}

// Sets the pose of the unicycle.
void Unicycle::setPose(const Pose2D &v)
{
    setPos(v.pos());
    setHeading(v.heading());
}

// Sets the pose of the unicycle.
void Unicycle::setPose(double xx, double yy, double th)
{
    setPos(xx,yy);
    setHeading(th);
}

// Returns the velocity of the Unicycle. It is a vector of the linear
// and angular velocities v and omega.
Vec2 Unicycle::vel() const
{
    return Vec2(v, w);
}

// Sets the velocity of the Unicycle. It is a vector of the linear and
// angular velocities v and omega.
void Unicycle::setVel(const Vec2 &v)
{
    this->v = v.x;
    this->w = v.y;
    boundingBoxValid = false;
}

// Sets the velocity of the Unicycle. It is a vector of the linear and
// angular velocities v and omega.
void Unicycle::setVel(double v, double omega)
{
    this->v = v;
    this->w = omega;
    boundingBoxValid = false;
}

// Returns the acceleration of the Unicycle. It is a vector of the
// linear acceleration a and the angular acceleration b.
Vec2 Unicycle::acc() const
{
    return Vec2(a, b);
}

// Sets the acceleration of the Unicycle. It is a vector of the
// linear acceleration a and the angular acceleration b.
void Unicycle::setAcc(const Vec2 &c)
{
    a = c.x;
    b = c.y;
    boundingBoxValid = false;
}

// Sets the linear acceleration a and the angular acceleration b
// of the Unicycle.
void Unicycle::setAcc(double a, double b)
{
    this->a = a;
    this->b = b;
    boundingBoxValid = false;
}

// Updates the state of the unicycle analytically given the controls a, b, and dt.
void Unicycle::predict()
{
    // b = 0 and omega = 0 case -> linear motion only
    if (fabs(b) < 0.001 && fabs(w) < 0.001)
    {
        //qDebug() << "linear case";

        w = 0;
        b = 0;

        double l = (0.5*a*dt+v)*dt;
        x += l*fcos(theta);
        y += l*fsin(theta);
        v += a*dt;
    }

    // a = b = 0 case -> motion on an arc.
    else if (fabs(a) < 0.001 && fabs(b) < 0.001)
    {
        //qDebug() << "arc case";

        a = 0;
        b = 0;
        double dtheta = dt*w;
        double r = v/w;
        double thetapdt = fpicut(theta+dtheta);
        x += r*(fsin(thetapdt)-fsin(theta));
        y += -r*(fcos(thetapdt)-fcos(theta));
        theta = thetapdt;
    }

    // b = 0 case -> fresnel formula singularity (b0 spiral).
    else if (fabs(b) < 0.001)
    {
        //qDebug() << "b0 case";

        b = 0;
        double dtheta = w*dt;
        double dv = a*dt;
        double thetadt = fpicut(theta+dtheta);
        double ct = fcos(theta);
        double st = fsin(theta);
        double codt = fcos(thetadt);
        double sodt = fsin(thetadt);
        double aww = a/(w*w);
        x += aww*(codt-ct)+((dv+v)*sodt-v*st)/w;
        y += aww*(sodt-st)-((dv+v)*codt-v*ct)/w;
        theta = thetadt;
        v += dv;
    }

    // Full Fresnel spiral prediciton.
    else
    {
        //qDebug() << "fresnel case" << a << b;

        bool flipped = false;
        if (b < 0)
        {
            b = -b;
            w = -w;
            flipped = true;
        }

        double dtheta = (0.5*b*dt+w)*dt;
        double sb = sqrt(b);
        double pb15 = pow(b,1.5);
        double wwbt = fpicut(0.5*w*w/b-theta);
        double gamma = fcos(wwbt);
        double sigma = fsin(wwbt);
        double c1,s1,c0,s0;
        fresnel((w+b*dt)/(sb*SPI), &s1, &c1);
        fresnel(w/(sb*SPI), &s0, &c0);
        double C = c1-c0;
        double S = s1-s0;
        double thetadt = fpicut(theta+dtheta);
        double dx = SPI*(b*v-a*w)*(sigma*S+gamma*C)/pb15 + (a/b)*(fsin(thetadt)-fsin(theta));
        double dy = SPI*(b*v-a*w)*(gamma*S-sigma*C)/pb15 - (a/b)*(fcos(thetadt)-fcos(theta));

        if (flipped)
        {
            b = -b;
            w = -w;
            double c2d = fcos(2*theta);
            double s2d = fsin(2*theta);
            double dxt = dx;
            dx = c2d*dx+s2d*dy;
            dy = s2d*dxt-c2d*dy;
        }

        x += dx;
        y += dy;
        theta = fpicut(theta+(0.5*b*dt+w)*dt);
        v += a*dt;
        w += b*dt;
    }

    boundingBoxValid = false;

    return;
}

// The predict method forwards this object in time by dt.
// The current acceleration a,b is used.
void Unicycle::predict(double dt)
{
    this->dt = dt;
    predict();
}

// Returns a forwarded object in time by dt.
// The current acceleration a, b is used.
Unicycle Unicycle::predicted() const
{
    Unicycle v = *this;
    v.predict();
    return v;
}

// Returns a forwarded object in time by dt.
// The current acceleration a, b is used.
Unicycle Unicycle::predicted(double dt) const
{
    Unicycle v = *this;
    v.predict(dt);
    return v;
}

// Returns the "instantenous center of curvature", the current
// center of rotation, in world coordinates.
Vec2 Unicycle::icc() const
{
    if (fabs(v) < EPSILON || fabs(w) < EPSILON)
        return pos();

    Vec2 pp(v/w, 0);
    pp.rotate(theta+PI2);
    return pp + pos();
}

// Returns the current "orbit angle" of the unicycle, which is the angle
// between the x axis and the ray from the icc through the position of the unicycle.
double Unicycle::orbitAngle() const
{
    return fpicut(theta-sgn(v*w)*PI2); // The angle of the position of the robot with respect to the icc.
}

// Returns the bounding box of the motion described by this unicycle instance.
const Box& Unicycle::boundingBox() const
{
    //qDebug() << "Unicycle::boundingBox():" << *this;

    if (boundingBoxValid)
        return aabb;
    boundingBoxValid = true;

    // b = 0 and omega = 0 case -> linear motion only
    if (fabs(b) < 0.001 && fabs(w) < 0.001)
    {
        //qDebug() << "LINEAR case";
        Hpm2D hpm = *this;
        aabb = hpm.boundingBox();
        return aabb;
    }

    // a = b = 0 case -> motion on an arc.
    else if (fabs(a) < 0.001 && fabs(b) < 0.001)
    {
        //qDebug() << "ARC case";

        // Computing the bounding box of a circular
        // trajectory boils down to taking the maximum
        // in each direction (top, left, bottom, right)
        // over the current position, the final position,
        // and the critical points at angles 0, PI2,
        // -PI2, and PI that are within reach of the
        // trajectory. As the radius remains constant
        // over the entire arc, the computation of the
        // positions at the critical points is trivial.
        // 0 can only set the right bound.
        // PI2 can only set the upper bound.
        // PI can only set the left bound.
        // -PI2 can only set the bottom bound.

        // Current position.
        double top = y;
        double left = x;
        double bottom = y;
        double right = x;

        Unicycle uf = predicted();

        // The final position.
        if (uf.y > top) top = uf.y;
        else if (uf.y < bottom) bottom = uf.y;
        if (uf.x < left) left = uf.x;
        else if (uf.x > right) right = uf.x;

        int direction = sgn(w);
        if (direction != 0)
        {
            double dangle = dt*fabs(w);
            double angle0 = orbitAngle();
            if (direction < 0)
                angle0 = uf.orbitAngle();

            double r = fabs(v/w);
            Vec2 ic = icc();

            //qDebug() << "icc:" << ic << "angle0:" << angle0 << "dangle:" << dangle << "r:" << r << "direction:" << direction;

            if ( (angle0 < 0 && dangle >= -angle0) || (angle0 > 0 && dangle >= PII-angle0) )
            {
                right = ic.x+r;
                //qDebug() << "0 included. Right set to" << right;
            }
            if ( (angle0 < PI2 && dangle >= PI2-angle0) || (angle0 > PI2 && dangle >= PII-(angle0-PI2)) )
            {
                top = ic.y+r;
                //qDebug() << "PI2 included. Top set to" << top;
            }
            if ( (angle0 < -PI2 && dangle >= -PI2-angle0) || (angle0 > -PI2 && dangle >= PII-(angle0+PI2)) )
            {
                bottom = ic.y-r;
                //qDebug() << "-PI2 included. Bottom set to" << bottom;
            }
            if (dangle > PI-angle0)
            {
                left = ic.x-r;
                //qDebug() << "PI included. Left set to" << left;
            }
        }

        aabb.set(x, y, top-y, left-x, bottom-y, right-x);
        return aabb;
    }

    // b = 0 case -> fresnel formula singularity.
    else if (fabs(b) < 0.001)
    {
        //qDebug() << "B0 case";

        // A B0 type bounding box is more difficult than the arc, because due to
        // the linear acceleration a, a special case can occur. When the linear
        // velocity flips sign, we have a cusp in the trajectory that can contribute
        // to the bounding box. The critical points at orbit angle 0, PI2, -PI2, and
        // PI can all contribute to the bounding box and need to be inspected, if reached.

        // The way the algorithm belows works is that it enummerates all cusp and
        // critical point events in the order they occur along the trajectory, and
        // adjusts the size of the bounding box to contain the position encountered
        // at the events.

        // Analyze if and when the type a and b cusps would occur. The cusps are
        // the points along the trajectory where either the linear velocity v or
        // the angular velocity w switches sign.
        double cuspTime = -v/a;

        // Determine the initial direction of rotation of the spiral.
        int direction = sgn(w);

        //qDebug() << "dir:" << direction << "cusp:" << cuspTime;

        // Initialize the bounding box with the current position.
        double top = y;
        double left = x;
        double bottom = y;
        double right = x;

        // Initialize the current orbit angle and the current time.
        double currentTime = 0;
        Unicycle u = *this;
        double oa = u.orbitAngle();

        // Step through all encountered events.
        int counter = 0;
        while (currentTime < dt-EPSILON && counter < 10)
        {
            //qDebug() << "  From oa:" << oa << "currentTime:" << currentTime;

            // Determine the orbit angle of the next critical point in line.
            double cp = 0;
            if (direction > 0)
            {
                if (oa >= 0-EPSILON && oa < PI2-EPSILON) cp = PI2;
                else if (oa >= PI2-EPSILON && oa < PI-EPSILON) cp = PI;
                else if (oa < -PI2-EPSILON || oa >= PI-EPSILON) cp = -PI2;
                else if (oa >= -PI2-EPSILON && oa < 0-EPSILON) cp = 0;
            }
            else
            {
                if (oa > 0+EPSILON && oa <= PI2+EPSILON) cp = 0;
                else if (oa > PI2+EPSILON || oa <= -PI+EPSILON) cp = PI2;
                else if (oa <= -PI2+EPSILON && oa > -PI+EPSILON) cp = -PI;
                else if (oa > -PI2+EPSILON && oa <= 0+EPSILON) cp = -PI2;
            }

            // Determine the time at which the next critical point will be reached.
            double cpTime = min(currentTime+(cp-oa)/w, dt);

            // Now check wich event will occur next, cusp type a or critical point, and act accordingly.

            // Cusp type a.
            if (cuspTime > currentTime && cuspTime <= cpTime)
            {
                // Predict the state at the cusp point.
                u = predicted(cuspTime+EPSILON);

                // Adjust the bounding box parameters.
                if (u.y > top) top = u.y;
                else if (u.y < bottom) bottom = u.y;
                if (u.x < left) left = u.x;
                else if (u.x > right) right = u.x;

                // Update the orbit angle.
                oa = u.orbitAngle();

                // Update the the current time.
                currentTime = cuspTime;

                //qDebug() << "  Cusp a oa:" << oa << "cp:" << cp << "cpTime:" << cpTime << "currentTime:" << currentTime;

                continue;
            }

            // Handle the critical point.
            u = predicted(cpTime);

            // Adjust the bounding box parameters.
            if (u.y > top) top = u.y;
            else if (u.y < bottom) bottom = u.y;
            if (u.x < left) left = u.x;
            else if (u.x > right) right = u.x;

            // Advance to the next critical point, or the end of the section.
            oa = cp;
            if (oa == PI && direction == 1)
                oa = -PI;
            else if (oa == -PI && direction == -1)
                oa = PI;
            currentTime = cpTime;

            //qDebug() << "  To oa:" << oa << "cp:" << cp << "cpTime:" << cpTime << "currentTime:" << currentTime;

            counter++;
        }


        aabb.set(x, y, top-y, left-x, bottom-y, right-x);
        return aabb;
    }

    // Full Fresnel spiral.
    else
    {
        //qDebug() << "FRESNEL case" << a << b;

        // A Fresnel type bounding box is the most difficult, because due to
        // both the acceleration a and the angular acceleration b, special cases
        // occur. When due to the acceleration a the linear velocity flips sign,
        // we have a cusp in the trajectory that can contribute to the bounding
        // box. When due to the angular acceleration b the angular velocity w
        // flips sign, we change the direction of rotation and the critical points
        // occur in reversed order. The critical points at orbit angle 0, PI2,
        // -PI2, and PI can all contribute to the bounding box and need to be
        // inspected, if reached.

        // The way the algorithm belows works is that it enummerates all cusp and
        // critical point events in the order they occur along the trajectory, and
        // adjusts the size of the bounding box to contain the position encountered
        // at the events.

        // Analyze if and when the type a and b cusps would occur. The cusps are
        // the points along the trajectory where either the linear velocity v or
        // the angular velocity w switches sign.
        double cuspTime_a = -v/a;
        double cuspTime_b = -w/b;

        // Determine the initial direction of rotation of the spiral.
        int direction = sgn(w);
        if (fabs(w) < 0.001)
            direction = sgn(b);

        //qDebug() << "dir:" << direction << "cusp a:" << cuspTime_a << "cusp b:" << cuspTime_b;

        // Initialize the bounding box with the current position.
        double top = y;
        double left = x;
        double bottom = y;
        double right = x;

        // Initialize the current orbit angle and the current time.
        double currentTime = 0;
        Unicycle u = *this;
        double oa = u.orbitAngle();

        // Step through all encountered events.
        while (currentTime < dt-EPSILON)
        {
            //qDebug() << "  From oa:" << oa << "currentTime:" << currentTime;

            // Determine the orbit angle of the next critical point in line.
            double cp = 0;
            if (direction > 0)
            {
                if (oa >= 0-EPSILON && oa < PI2-EPSILON) cp = PI2;
                else if (oa >= PI2-EPSILON && oa < PI-EPSILON) cp = PI;
                else if (oa < -PI2-EPSILON || oa >= PI-EPSILON) cp = -PI2;
                else if (oa >= -PI2-EPSILON && oa < 0-EPSILON) cp = 0;
            }
            else
            {
                if (oa > 0+EPSILON && oa <= PI2+EPSILON) cp = 0;
                else if (oa > PI2+EPSILON || oa <= -PI+EPSILON) cp = PI2;
                else if (oa <= -PI2+EPSILON && oa > -PI+EPSILON) cp = -PI;
                else if (oa > -PI2+EPSILON && oa <= 0+EPSILON) cp = -PI2;
            }

            // Determine the time at which the next critical point will be reached.
            double sq = sqrt((u.w*u.w)/(b*b)-2.0*(oa-cp)/b);
            double cpTime1 = -u.w/b-sq;
            double cpTime2 = -u.w/b+sq;
            double cpTime = dt;
            if (cpTime1 > 0)
                cpTime = min(currentTime+cpTime1, dt);
            else if (cpTime2 > 0)
                cpTime = min(currentTime+cpTime2, dt);

            // Now check wich event will occur next, cusp a, cusp b, or critical point, and act accordingly.


            // Cusp type b.
            if (cuspTime_b > currentTime && !(cuspTime_a > currentTime && cuspTime_a < cuspTime_b) && cuspTime_b <= cpTime)
            {
                // Flip the direction!
                direction = -direction;

                // Predict the state at the cusp point.
                u = predicted(cuspTime_b+EPSILON);

                // Adjust the bounding box parameters.
                if (u.y > top) top = u.y;
                else if (u.y < bottom) bottom = u.y;
                if (u.x < left) left = u.x;
                else if (u.x > right) right = u.x;

                // Update the orbit angle.
                oa = u.orbitAngle();

                // Update the current time.
                currentTime = cuspTime_b;

                //qDebug() << "  Cusp b oa:" << oa << "cp:" << cp << "cpTime:" << cpTime << cpTime1 << cpTime2 << "currentTime:" << currentTime;

                continue;
            }

            // Cusp type a.
            if (cuspTime_a > currentTime && cuspTime_a <= cpTime)
            {
                // Predict the state at the cusp point.
                u = predicted(cuspTime_a+EPSILON);

                // Adjust the bounding box parameters.
                if (u.y > top) top = u.y;
                else if (u.y < bottom) bottom = u.y;
                if (u.x < left) left = u.x;
                else if (u.x > right) right = u.x;

                // Update the orbit angle.
                oa = u.orbitAngle();

                // Update the the current time.
                currentTime = cuspTime_a;

                //qDebug() << "  Cusp a oa:" << oa << "cp:" << cp << "cpTime:" << cpTime << cpTime1 << cpTime2 << "currentTime:" << currentTime;

                continue;
            }

            // Handle the critical point.
            u = predicted(cpTime);

            // Adjust the bounding box parameters.
            if (u.y > top) top = u.y;
            else if (u.y < bottom) bottom = u.y;
            if (u.x < left) left = u.x;
            else if (u.x > right) right = u.x;

            // Advance to the next critical point, or the end of the section.
            oa = cp;
            if (oa == PI && direction == 1)
                oa = -PI;
            else if (oa == -PI && direction == -1)
                oa = PI;
            currentTime = cpTime;

            //qDebug() << "  To oa:" << oa << "cp:" << cp << "cpTime:" << cpTime << cpTime1 << cpTime2 << "currentTime:" << currentTime;
        }

        // Set and return the bounding box.
        aabb.set(x, y, top-y, left-x, bottom-y, right-x);
        return aabb;
    }
}

// Intersects the trajectory of the unicycle with the (static) line.
// It returns the time of the collision, if a collision occurs between
// time zero and dt. Otherwise it returns -1.
double Unicycle::intersects(const Line &line) const
{
//    qDebug() << "Unicycle::intersects(): line:" << line;

    // Bounding box check.
    if (!boundingBox().intersects(line))
        return -1;

    // Linear motion only.
    if (fabs(b) < 0.001 && fabs(w) < 0.001)
    {
        // Holonomic collision check.
        Hpm2D h = *this;
        return h.intersects(line);
    }

    // a = b = 0 case -> motion on an arc.
    else if (fabs(a) < 0.001 && fabs(b) < 0.001)
    {
        // Center everything around the icc.
        Vec2 c = icc();
        Line l = line;
        l.translate(-c);
        double r = v/w;
        double phi = orbitAngle();

        // Compute the angle of the intersection point.
        double alpha1 = -100;
        double alpha2 = -100;
        if (l.isVertical())
        {
            if (fabs(l.left()) > fabs(r))
                return -1;

            double sq = sqrt(r*r-l.left()*l.left());
            double y1 = -sq;
            double y2 = sq;

            if (y1 > l.bottom() && y1 < l.top())
                alpha1 = atan2(y1, l.left()) - phi;

            if (y2 > l.bottom() && y2 < l.top())
                alpha2 = atan2(y2, l.left()) - phi;
        }
        else if (l.isHorizontal())
        {
            if (fabs(l.y1()) > fabs(r))
                return -1;

            double sq = sqrt(r*r-l.y1()*l.y1());
            double x1 = -sq;
            double x2 = sq;

            if (x1 > l.left() && x1 < l.right())
                alpha1 = atan2(l.y1(), x1) - phi;

            if (x2 > l.left() && x2 < l.right())
                alpha2 = atan2(l.y1(), x2) - phi;
        }
        else
        {
            double la = l.a();
            double lb = l.b();
            double ab = la*lb;
            double aap1 = la*la+1.0;
            double sq = (ab*ab)/(aap1*aap1) - (lb*lb-r*r)/aap1;
            if (sq < 0)
                return -1;

            sq = sqrt(sq);
            double x1 = -ab/aap1 - sq;
            double x2 = -ab/aap1 + sq;

            if (x1 > l.left() && x1 < l.right())
                alpha1 = atan2(la*x1+lb, x1) - phi;

            if (x2 > l.left() && x2 < l.right())
                alpha2 = atan2(la*x2+lb, x2) - phi;
        }

        // Convert the angle to time.
        double ct1 = -1;
        double ct2 = -1;
        if (alpha1 > -100)
        {
            if (sgn(w) != sgn(alpha1))
                alpha1 = alpha1 + sgn(w)*PII;
            ct1 = alpha1/w;
        }
        if (alpha2 > -100)
        {
            if (sgn(w) != sgn(alpha2))
                alpha2 = alpha2 + sgn(w)*PII;
            ct2 = alpha2/w;
        }

        // Determine the smaller time.
        double ct = ct1;
        if (ct < 0 || (ct2 > -1 && ct2 < ct))
            ct = ct2;
        if (ct > dt)
            ct = -1;

        return ct;
    }

    // b = 0 case -> fresnel formula singularity
    else if (fabs(b) < 0.001)
    {
        return intersects(line, Vec2());
    }

    // Full Fresnel collision check.
    else
    {
        return intersects(line, Vec2());
    }
}

// Intersects the trajectory of the unicycle with the moving line.
// The motion of the line is defined by the lineVelocity vector.
// It returns the time of the collision, if a collision occurs between
// time 0 and dt. Otherwise it returns -1.
double Unicycle::intersects(const Line &line, const Vec2& lineVelocity) const
{
//    if (config.debugLevel > 0)
//    {
//        qDebug() << "            Unicycle::intersects(const Line &line, Vec2 lv):";
//        qDebug() << "              Line:" << line << "lv:" << lineVelocity;
//        qDebug() << "              unicycle from:" << *this;
//        qDebug() << "              unicycle to:  " << predicted(dt);
//    }

    // Bounding box check.
    // We use the bounding box of the unicycle to check against the swept
    // volume of the moving line. This doesn't cost much.
    const Box &bb = boundingBox();
    if ((line.left() > bb.right() && line.left()+dt*lineVelocity.x > bb.right())
        || (line.right() < bb.left() && line.right()+dt*lineVelocity.x < bb.left())
        || (line.bottom() > bb.top() && line.bottom()+dt*lineVelocity.y > bb.top())
        || (line.top() < bb.bottom() && line.top()+dt*lineVelocity.y < bb.bottom()))
            return -1;

    // Linear motion only.
    if (fabs(b) < 0.001 && fabs(w) < 0.001)
    {
        // The linear case is easily dealt with with a holonomic collision check.
        Hpm2D h = *this;
        h.vx -= lineVelocity.x;
        h.vy -= lineVelocity.y;
        return h.intersects(line);
    }

    // First we rotate the line, the unicycle, and their velocities around the origin
    // such that the line becomes vertical.
    double langle = line.angle();
    double rangle = sgn(langle)*PI2-langle;
    Line rotatedLine = line.rotated(rangle);
    Vec2 rotatedLineVelocity = lineVelocity.rotated(rangle);
    Unicycle rotatedUnicycle = rotated(rangle);

//    if (config.debugLevel > 0)
//    {
//        qDebug() << "              Line angle:" << line.angle() << rangle << line.angle()+rangle;
//        qDebug() << "              Rotated line:" << rotatedLine << "lv:" << rotatedLineVelocity;
//        qDebug() << "              Rotated Unicycle:" << rotatedUnicycle;
//    }

    // In the nonlinear case, we want to solve the equation line(x(t)) - y(t) = 0 for t.
    // Unfortunately, we can't do this analytically, not even for an arc,
    // and so we use the Regula Falsi (Illinois) method to find the root numerically.
    // For this, we need to split up the time interval dt into monotonic
    // sequences (brackets) between the "zenith" points 0, pi/2, pi, and -pi/2.
    // Also cusps where v or w flip sign are bracket boundaries.
    double zenith[] = {0, PI2, -PI2, PI, -PI, PI32, -PI32, PII, -PII};

    // Initialize the brackets with 0.
    brackets.clear();
    brackets << 0;

    // The v cusp. (The omega cusp does not influence monotony.
    if (v*a < 0 && -v/a < dt)
        brackets << -v/a;

    // a = b = 0 case -> motion on an arc.
    if (fabs(a) < 0.001 && fabs(b) < 0.001)
    {
        // Zenith times.
        for (int i = 0; i < 9; i++)
        {
            double alpha = zenith[i]-rotatedUnicycle.theta;
            if (sgn(w) != sgn(alpha))
                alpha = alpha + sgn(w)*PII;
            double tt = alpha/w;
            if (tt > 0 && tt < dt)
                brackets << tt;
        }
    }

    // b = 0 case -> fresnel formula singularity
    else if (fabs(b) < 0.001)
    {
        // Zenith times.
        for (int i = 0; i < 9; i++)
        {
            double alpha = zenith[i]-rotatedUnicycle.theta;
            if (sgn(w) != sgn(alpha))
                alpha = alpha + sgn(w)*PII;
            double tt = alpha/w;
            if (tt > 0 && tt < dt)
                brackets << tt;
        }
    }

    // Full Fresnel collision check.
    else
    {
        // Zenith times.
        double w0b = w/b;
        double w00bb = (w*w)/(b*b);
        double b2 = 2.0/b;
        for (int i = 0; i < 9; i++)
        {
            double sq = sqrt(w00bb-b2*(rotatedUnicycle.theta-zenith[i]));
            double t1 = -w0b-sq;
            double t2 = -w0b+sq;
            if (t1 > 0 && t1 < dt)
                brackets << t1;
            if (t2 > 0 && t2 < dt)
                brackets << t2;
        }
    }

    // And dt.
    brackets << dt;
    brackets.sort();

    //if (config.debugLevel > 0)
    //qDebug() << "                Brackets:" << brackets;

    Unicycle cur = rotatedUnicycle;
    Unicycle prev;
    for (uint i = 1; i < brackets.size(); i++)
    {
        // Test if the bracket is "hugging" the line, i.e. whether at the lower time the
        // unicycle is on the one side of the line and at the higher time on the other.
        prev = cur;
        cur = rotatedUnicycle.predicted(brackets[i]);

        // Scalar product-based line distance function with the predicted lines,
        // except since we rotate the line to be vertical, we only need the distance in x direction.
        Line l1 = rotatedLine.translated(rotatedLineVelocity*brackets[i-1]);
        Line l2 = rotatedLine.translated(rotatedLineVelocity*brackets[i]);
        double ld1 = (prev.x-l1.x1());
        double ld2 = (cur.x-l2.x1());

        // Is it a hugging bracket?
        // If it is not, there cannot be a collision and we can safely advance to the next bracket.
//        if (config.debugLevel > 0)
//        {
//            qDebug() << "                It it a hugging bracket? dl:" << ld1 << ld2 << "t:" << brackets[i-1] << brackets[i];
//            qDebug() << "                   u1 :" << prev;
//            qDebug() << "                   l1 :" << l1;
//            qDebug() << "                   u2 :" << cur;
//            qDebug() << "                   l2 :" << l2;
//        }
        if (ld1*ld2 > 0)
            continue;

        // So there must be an intersection inside the bracket.
        // Use regula falsi (Illinois) to solve the equation y(t)-(line.a*x(t)+line.b+lv.y) = 0.
        double z, f;
        double bz, bf = 1.0E6;
        double bracketHigh = brackets[i];
        double bracketLow = brackets[i-1];
        for (int i = 0; i < 3; i++)
        {
            // Compute the function value with a linear interpolation inside the bracket.
            z = bracketLow+ld1*(bracketLow-bracketHigh)/(ld2-ld1);
            Unicycle uu = rotatedUnicycle.predicted(z);
            Line ll = rotatedLine.translated(rotatedLineVelocity*z);
            f = (uu.x-ll.x1());

            // Remember the best solution so far.
            if (fabs(f) < bf)
            {
                bz = z;
                bf = fabs(f);
            }

            if (sgn(ld2)*f < 0)
            {
                bracketLow = bracketHigh;
                ld1 = ld2;
                bracketHigh = z;
                ld2 = f;
            }
            else
            {
                bracketHigh = z;
                ld1 *= 0.5;
                ld2 = f;
            }

//            if (config.debugLevel > 0)
//            qDebug() << "                 " << i << "low:" << bracketLow << "high:" << bracketHigh << "cur:" << z << "f:" << f << "best:" << bf;
        }

        // The solution of the bisection should be on the line.
        // Let's check if it's inside the boundaries of the line with a projection technique.
        Unicycle bu = rotatedUnicycle.predicted(bz);
        Line ll = rotatedLine.translated(rotatedLineVelocity*bz);
//        double lineMissedBy = qMin(fabs(bu.y-ll.y2), fabs(bu.y-ll.y1));
//        if (config.debugLevel > 0)
//        {
//            qDebug() << "                  Checking line boundaries. t:" << bz << "d:" << lineMissedBy;
//            qDebug() << "                     u:" << bu;
//            qDebug() << "                     l:" << ll;
//        }
        if ((bu.y < ll.top() && bu.y > ll.bottom()))
        {
//            if (config.debugLevel > 0)
//            qDebug() << "                Collision detected at t:" << bz;

            // Collision detected.
            return bz;
        }
    }

    // No collision found.
    return -1;
}


// Updates the state of the car by time t using Euler integration.
// This is a slow method that is best used only to test the analytic method below.
void Unicycle::simulate(double dt)
{
    double timestep = 1.0E-6; // simulation time step

    double time = 0;
    while (time <= dt)
    {
        x += timestep * v * fcos(theta);
        y += timestep * v * fsin(theta);
        theta += timestep * w;
        v += timestep * a;
        w += timestep * b;

        time += timestep;
    }

    boundingBoxValid = false;
}

void Unicycle::draw(QPainter *painter) const
{
    if (dt <= EPSILON)
        return;

    // Draw the trajectory backwards. It gets rid of an unwanted visual effect.
    double ddt = 0.05;
    double ct = dt;
    Unicycle u = predicted(ct);
    while (ct > ddt)
    {
        ct -= ddt;
        Unicycle uold = u;
        u = predicted(ct);
        if (uold.pos() != u.pos())
            painter->drawLine(QLineF(uold.x, uold.y, u.x, u.y));
    }
    if (u.pos() != pos())
        painter->drawLine(QLineF(u.x, u.y, x, y));

    // The switching points.
    double s = 0.01;
    painter->drawEllipse(pos(), s, s);
    painter->drawEllipse(predicted(dt).pos(), s, s);

    // The icc.
//    painter->drawEllipse(icc(), 2*s, 2*s);
//    double r=fabs(v/w);
//    if (r == r)
//    {
//        painter->save();
//        painter->setBrush(Qt::NoBrush);
//        painter->drawEllipse(icc(), fabs(v/w), fabs(v/w));
//        painter->restore();
//    }
}

// OpenGL drawing.
void Unicycle::draw() const
{
    if (dt <= EPSILON)
        return;

    // Draw the trajectory backwards. It gets rid of an unwanted visual effect.
    double ddt = 0.05;
    double ct = dt;
    Unicycle u = predicted(ct);
    while (ct > ddt)
    {
        ct -= ddt;
        Unicycle uold = u;
        u = predicted(ct);
        if (uold.pos() != u.pos())
            GLlib::drawLine(uold.pos(), u.pos());
    }
    if (u.pos() != pos())
        GLlib::drawLine(u.pos(), pos());

    // The switching points.
    double s = 0.01;
    GLlib::drawEllipse(pos(), s, s);
    GLlib::drawEllipse(predicted(dt).pos(), s, s);
}

// Writes the Unicycle into a data stream.
void Unicycle::streamOut(QDataStream &out) const
{
    out << dt;
    out << x;
    out << y;
    out << theta;
    out << v;
    out << w;
    out << a;
    out << b;
}

// Reads the Unicycle from a data stream.
void Unicycle::streamIn(QDataStream &in)
{
    in >> dt;
    in >> x;
    in >> y;
    in >> theta;
    in >> v;
    in >> w;
    in >> a;
    in >> b;

    boundingBoxValid = false;
}

QDataStream& operator<<(QDataStream& out, const Unicycle &o)
{
    o.streamOut(out);
    return out;
}

QDataStream& operator>>(QDataStream& in, Unicycle &o)
{
    o.streamIn(in);
    return in;
}

QDebug operator<<(QDebug dbg, const Unicycle &o)
{
    dbg << "dt:" << o.dt << "pose:" << o.pose() << "vel:" << o.vel() << "acc:" << o.acc();
    return dbg;
}
