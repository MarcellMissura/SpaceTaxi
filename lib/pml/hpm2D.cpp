#include "hpm2D.h"

static const double sixth = 1.0/6;

Hpm2D::Hpm2D()
{
    t = 0;
    dt = 0;
    x = 0;
    y = 0;
    vx = 0;
    vy = 0;
    ax = 0;
    ay = 0;
    jx = 0;
    jy = 0;

    boundingBoxValid = false;
}

void Hpm2D::set(double x, double y, double vx, double vy, double ax, double ay, double jx, double jy)
{
    this->x = x;
    this->y = y;
    this->vx = vx;
    this->vy = vy;
    this->ax = ax;
    this->ay = ay;
    this->jx = jx;
    this->jy = jy;

    boundingBoxValid = false;
}

void Hpm2D::reset()
{
    t = 0;
    dt = 0;
    x = 0;
    vx = 0;
    ax = 0;
    jx = 0;
    y = 0;
    vy = 0;
    ay = 0;
    jy = 0;

    boundingBoxValid = false;
}

void Hpm2D::predict()
{
    double sixth = 1.0/6;
    t += dt;
    x = dt*(dt*(sixth*dt*jx+0.5*ax)+vx)+x;
    y = dt*(dt*(sixth*dt*jy+0.5*ay)+vy)+y;
    vx = dt*(0.5*dt*jx+ax)+vx;
    vy = dt*(0.5*dt*jy+ay)+vy;
    ax = dt*jx+ax;
    ay = dt*jy+ay;

    boundingBoxValid = false;
}

Hpm2D Hpm2D::predicted() const
{
    Hpm2D v = *this;
    v.predict();
    return v;
}

void Hpm2D::predict(double dt)
{
    this->dt = dt;
    predict();
}

Hpm2D Hpm2D::predicted(double dt) const
{
    Hpm2D v = *this;
    v.predict(dt);
    return v;
}

// Returns the bounding box of this trajectory.
const Box &Hpm2D::boundingBox() const
{
    // We are going to compute a bounding box for the holonomic trajectory
    // described by jerk j, acceleration a, velocity v, and position x in
    // x and y directions. Starting from the current position of the hpm2d,
    // we search for the maximum in left, right, up, and down directions
    // among the critical points where the velocity is zero, and the final
    // point of the trajectory. Whether j=0 or not decides if the velocity
    // is a linear function or a 2nd order polynomial.

    if (boundingBoxValid)
        return aabb;
    boundingBoxValid = true;

// X

    // Initialize with the current point.
    double left = x;
    double right = x;

    if (fabs(jx) < EPSILON)
    {
        // Easier case, j = 0, v is linear.
        // There is only one turning point.

        // The final point.
        double xnew = x+(vx+0.5*ax*dt)*dt;
        if (xnew < left)
            left = xnew;
        else if (xnew > right)
            right = xnew;

        // Critical point where the velocity is zero.
        double tzx = -vx/ax;
        if (tzx > 0 && tzx < dt)
        {
            double xnew = x+(vx+0.5*ax*tzx)*tzx;
            if (xnew < left)
                left = xnew;
            if (xnew > right)
                right = xnew;
        }
    }
    else
    {
        // Harder case, jerk is set and v is a quadratic function.
        // There can be up to two turning points.

        // The final point.
        double xnew = dt*(dt*(sixth*dt*jx+0.5*ax)+vx)+x;
        if (xnew < left)
            left = xnew;
        else if (xnew > right)
            right = xnew;

        // Critical points where the velocity is zero.
        double s = sqrt((ax*ax)/(jx*jx) - 2*vx/jx);
        double tz1 = -ax/jx + s;
        double tz2 = -ax/jx - s;

        if (tz1 > 0 && tz1 < dt)
        {
            double xnew = tz1*(tz1*(sixth*tz1*jx+0.5*ax)+vx)+x;
            if (xnew < left)
                left = xnew;
            if (xnew > right)
                right = xnew;
        }
        if (tz2 > 0 && tz2 < dt)
        {
            double xnew = tz2*(tz2*(sixth*tz2*jx+0.5*ax)+vx)+x;
            if (xnew < left)
                left = xnew;
            if (xnew > right)
                right = xnew;
        }
    }

// Y

    // Initialize with the current point.
    double top = y;
    double bottom = y;

    if (fabs(jy) < EPSILON)
    {
        // Easier case, j = 0, v is linear.
        // There is only one turning point.

        // The final point.
        double ynew = y+(vy+0.5*ay*dt)*dt;
        if (ynew < bottom)
            bottom = ynew;
        else if (ynew > top)
            top = ynew;

        double tzy = -vy/ay;
        if (tzy > 0 && tzy < dt)
        {
            double ynew = y + (vy+0.5*ay*tzy)*tzy;
            if (ynew < bottom)
                bottom = ynew;
            else if (ynew > top)
                top = ynew;
        }
    }
    else
    {
        // Harder case, v is a quadratic function.
        // There can be up to two turning points.

        // The final point.
        double ynew = dt*(dt*(sixth*dt*jy+0.5*ay)+vy)+y;
        if (ynew < bottom)
            bottom = ynew;
        else if (ynew > top)
            top = ynew;

        double s = sqrt((ay*ay)/(jy*jy) - 2*vy/jy);
        double tz1 = -ay/jy + s;
        double tz2 = -ay/jy - s;

        if (tz1 > 0 && tz1 < dt)
        {
            double ynew = tz1*(tz1*(sixth*tz1*jy+0.5*ay)+vy)+y;
            if (ynew < bottom)
                bottom = ynew;
            else if (ynew > top)
                top = ynew;
        }
        if (tz2 > 0 && tz2 < dt)
        {
            double ynew = tz2*(tz2*(sixth*tz2*jy+0.5*ay)+vy)+y;
            if (ynew < bottom)
                bottom = ynew;
            else if (ynew > top)
                top = ynew;
        }
    }

    // Construct the box from the left, right, top, bottom parameters.
    aabb.set(x, y, top-y, left-x, bottom-y, right-x);

    return aabb;
}

// Computes the time of the future intersection between the line
// and the trajectoy described by this hpm2d. If there is no
// intersection, or the intersection time is greater than dt,
// -1 is returned.
double Hpm2D::intersects(const Line &line) const
{
//    qDebug() << "Hpm2D::intersects() Line:" << line << *this;

    // Bounding box check with line.
    const Box& aabb = boundingBox();
    if (!aabb.intersects(line))
        return -1;

    double ct1 = -1;
    double ct2 = -1;
    double ct3 = -1;
    if (line.isVertical())
    {
//        qDebug() << "   line is vertical";

        if (fabs(jx) > EPSILON)
        {
            // Cubic case.
            // We are using the GSL library to solve for the roots of the polynomial equation.
            // x + vt + 1/2 at² + 1/6 jt³ = line x reformulated to
            // t³ + 3a/j t² + 6v/j t + 6(x-line x)/j = 0
            // https://www.gnu.org/software/gsl/doc/html/poly.html#cubic-equations

//            qDebug() << "   cubic case x";

            double p0 = jx/6;
            double p1 = 0.5*ax/p0;
            double p2 = vx/p0;
            double p3 = (x-line.x1())/p0;
            //gsl_poly_solve_cubic(p1, p2, p3, &ct1, &ct2, &ct3);
        }
        else if (fabs(ax) > EPSILON)
        {
            // Quadratic case.
            // x + vt + 1/2 at² = line x reformulated to
            // t² + 6v/j t + 6(x - line x)/j = 0

//            qDebug() << "   quadratic case x";

            double p = vx/ax;
            double q = 2*(x-line.x1())/ax;
            double root = sqrt(p*p-q);
            ct1 = -p-root;
            ct2 = -p+root;
        }
        else
        {
//            qDebug() << "   linear case x";

            // Linear case.
            ct1 = (line.x1()-x)/vx;
        }

//        qDebug() << "c:" << ct1 << ct2 << ct3;

        // Validate the crossing times and check if the y coordinate is on the edge.
        if (ct1 >= 0 && ct1 <= dt)
        {
            double yct = ct1*(ct1*((1.0/6)*ct1*jy+0.5*ay)+vy)+y;
            if (yct >= line.bottom() && yct <= line.top())
                return ct1;
        }
        if (ct2 >= 0 && ct2 <= dt)
        {
            double yct = ct2*(ct2*((1.0/6)*ct2*jy+0.5*ay)+vy)+y;
            if (yct >= line.bottom() && yct <= line.top())
                return ct2;
        }
        if (ct3 >= 0 && ct3 <= dt)
        {
            double yct = ct3*(ct3*((1.0/6)*ct3*jy+0.5*ay)+vy)+y;
            if (yct >= line.bottom() && yct <= line.top())
                return ct3;
        }
    }
    else if (line.isHorizontal())
    {
//        qDebug() << "   line is horizontal";

        if (fabs(jy) > EPSILON)
        {
            // Cubic case.
            // We are using the GSL library to solve for the roots of the polynomial equation.
            // x + vt + 1/2 at² + 1/6 jt³ = line x reformulated to
            // t³ + 3a/j t² + 6v/j t + 6(x-line x)/j = 0
            // https://www.gnu.org/software/gsl/doc/html/poly.html#cubic-equations

//            qDebug() << "   cubic case y";

            double p0 = jy/6;
            double p1 = 0.5*ay/p0;
            double p2 = vy/p0;
            double p3 = (y-line.y1())/p0;
            //gsl_poly_solve_cubic(p1, p2, p3, &ct1, &ct2, &ct3);
        }
        else if (fabs(ay) > EPSILON)
        {
            // Quadratic case.
            // x + vt + 1/2 at² = line x reformulated to
            // t² + 6v/j t + 6(x - line x)/j = 0

//            qDebug() << "   quadratic case y";

            double p = vy/ay;
            double q = 2*(y-line.y1())/ay;
            double root = sqrt(p*p-q);
            ct1 = -p-root;
            ct2 = -p+root;
        }
        else
        {
//            qDebug() << "   linear case y";

            // Linear case.
            ct1 = (line.y1()-y)/vy;
        }

//        qDebug() << "c:" << ct1 << ct2 << ct3;

        // Validate the crossing times and check if the x coordinate is on the edge.
        if (ct1 >= 0 && ct1 <= dt)
        {
            double xct = ct1*(ct1*((1.0/6)*ct1*jx+0.5*ax)+vx)+x;
            if (xct >= line.left() && xct <= line.right())
            {
//                qDebug() << "ct1 is valid";
                return ct1;
            }
        }
        if (ct2 >= 0 && ct2 <= dt)
        {
            double xct = ct2*(ct2*((1.0/6)*ct2*jx+0.5*ax)+vx)+x;
            if (xct >= line.left() && xct <= line.right())
            {
//                qDebug() << "ct2 is valid";
                return ct2;
            }
        }
        if (ct3 >= 0 && ct3 <= dt)
        {
            double xct = ct3*(ct3*((1.0/6)*ct3*jx+0.5*ax)+vx)+x;
            if (xct >= line.left() && xct <= line.right())
            {
//                qDebug() << "ct3 is valid";
                return ct3;
            }
        }
    }
    else // The line is in a general orientation.
    {
//        qDebug() << "   line is not vertical";

        if (!jerk().isNull())
        {
//            qDebug() << "   cubic case";

            // Cubic case.
            // We are using the GSL library to solve for the roots of the polynomial equation.
            // la*(x + vxt + 1/2 axt² + 1/6 jxt³) + lb = y + vyt + 1/2 ayt² + 1/6 jyt³ reformulated to
            // t³ + 3(la*ax-ay)/(la*jx-jy) t² + 6(la*vx-vy)/(la*jx-jy) t + 6(la*x-y+lb)/(la*jx-jy) = 0
            // https://www.gnu.org/software/gsl/doc/html/poly.html#cubic-equations
            double la = line.a();
            double p0 = (la*jx-jy)/6;
            double p1 = (1./2)*(la*ax-ay)/p0;
            double p2 = (la*vx-vy)/p0;
            double p3 = (la*x-y+line.b())/p0;
            //gsl_poly_solve_cubic(p1, p2, p3, &ct1, &ct2, &ct3);
        }
        else if (!acc().isNull())
        {
            // Quadratic case.
            // la*(x+vxt+1/2axt²)+lb = y+vyt+1/2ay² reformulated to
            // 1/2 (la*ax-ay) t² + (la*vx-vy) t + (la*x-y+lb) = 0

            double la = line.a();
            double p0 = (la*ax-ay);
            double p = (la*vx-vy)/p0;
            double q = 2*(la*x-y+line.b())/p0;
            double root = sqrt(p*p-q);
            ct1 = -p-root;
            ct2 = -p+root;

//            qDebug() << "   quadratic case" <<  ct1 << ct2;
        }
        else
        {
            // Linear case.
            // la*(x + vx*t)+lb = y+vy*t
            // (la*vx-vy) t + la*x-y+lb = 0
            ct1 = (y-line.a()*x-line.b())/(line.a()*vx-vy);

//            qDebug() << "   linear case ct1:" << ct1 << "la:" << line.a;
        }

//        qDebug() << "c:" << ct1 << ct2 << ct3;

        // Validate the crossing times and check if the y coordinate is on the edge.
        if (ct1 >= 0 && ct1 <= dt)
        {
            double xct = ct1*(ct1*((1.0/6)*ct1*jx+0.5*ax)+vx)+x;
            if (xct >= line.left() && xct <= line.right())
            {
//                qDebug() << "ct1 is valid";
                return ct1;
            }
        }
        if (ct2 >= 0 && ct2 <= dt)
        {
            double xct = ct2*(ct2*((1.0/6)*ct2*jx+0.5*ax)+vx)+x;
            if (xct >= line.left() && xct <= line.right())
            {
//                qDebug() << "ct2 is valid";
                return ct2;
            }
        }
        if (ct3 >= 0 && ct3 <= dt)
        {
            double xct = ct3*(ct3*((1.0/6)*ct3*jx+0.5*ax)+vx)+x;
            if (xct >= line.left() && xct <= line.right())
            {
//                qDebug() << "ct3 is valid";
                return ct3;
            }
        }
    }

    return -1;
}

bool Hpm2D::operator==(const Hpm2D &o)
{
    return (fabs(x-o.x) < EPSILON && fabs(y-o.y) < EPSILON && fabs(vx-o.vx) < EPSILON && fabs(vy-o.vy) < EPSILON);
}

bool Hpm2D::operator!=(const Hpm2D &o)
{
    return (fabs(x-o.x) >= EPSILON || fabs(y-o.y) >= EPSILON || fabs(vx-o.vx) >= EPSILON || fabs(vy-o.vy) >= EPSILON);
}

// Returns the position of the Hpm2D.
Vec2 Hpm2D::pos() const
{
    return Vec2(x,y);
}

// Returns the velocity of the Hpm2D.
Vec2 Hpm2D::vel() const
{
    return Vec2(vx,vy);
}

// Returns the acceleration of the Hpm2D.
Vec2 Hpm2D::acc() const
{
    return Vec2(ax,ay);
}

// Returns the jerk of the Hpm2D.
Vec2 Hpm2D::jerk() const
{
    return Vec2(jx,jy);
}

// Returns the angle of the velocity vector, mimicing a heading of a holonomic vehicle.
double Hpm2D::heading() const
{
    return atan2(vy, vx);
}

// Sets the position of the Hpm2D.
void Hpm2D::setPos(const Vec2 &p)
{
    x = p.x;
    y = p.y;
}

// Sets the position of the Hpm2D.
void Hpm2D::setPos(double x, double y)
{
    this->x = x;
    this->y = y;
}

// Sets the velocity vector of the Hpm2D.
void Hpm2D::setVel(const Vec2 &v)
{
    vx = v.x;
    vy = v.y;
}

// Sets the velocity vector of the Hpm2D.
void Hpm2D::setVel(double vx, double vy)
{
    this->vx = vx;
    this->vy = vy;
}

// Sets the acceleration of the Hpm2D.
void Hpm2D::setAcc(const Vec2 &c)
{
    ax = c.x;
    ay = c.y;
}

// Sets the acceleration of the Hpm2D.
void Hpm2D::setAcc(double ax, double ay)
{
    this->ax = ax;
    this->ay = ay;
}

// Sets the jerk of the Hpm2D.
void Hpm2D::setJerk(const Vec2 &j)
{
    jx = j.x;
    jy = j.y;
}

// Sets the jerk of the Hpm2D.
void Hpm2D::setJerk(double jx, double jy)
{
    this->jx = jx;
    this->jy = jy;
}

// Translates the Hpm2D by (dx,dy).
void Hpm2D::translate(double dx, double dy)
{
    setPos(this->x+dx, this->y+dy);
}

// Translates the Hpm2D by d.
void Hpm2D::translate(const Vec2 &d)
{
    setPos(this->x+d.x, this->y+d.y);
}

// Rotates the Hpm2D around the origin, i.e., applies a rotation
// by the given angle to the coordinates and velocity.
void Hpm2D::rotate(double angle)
{
    if (fabs(angle) < EPSILON)
        return;

    double c = fcos(angle);
    double s = fsin(angle);

    if (fabs(s) < EPSILON)
        return;

    // Rotate x,y.
    double x_ = x;
    double y_ = y;
    x = x_*c + y_*-s;
    y = x_*s + y_*c;

    // Rotate vx,vy.
    x_ = vx;
    y_ = vy;
    vx = x_*c + y_*-s;
    vy = x_*s + y_*c;

    // Rotate ax,ay.
    x_ = ax;
    y_ = ay;
    ax = x_*c + y_*-s;
    ay = x_*s + y_*c;

    // And the jerk?

}

// Draws the trajectory described by the ctrl sequence onto the QPainter.
void Hpm2D::draw(QPainter *painter) const
{
    // Draw the trajectory backwards. I forgot why it's better this way.
    double ddt = 0.025;
    double ct = dt;
    Hpm2D kf = predicted(ct);
    while (ct > ddt)
    {
        ct -= ddt;
        Hpm2D kfold = kf;
        kf = predicted(ct);
        if (kfold.pos() != kf.pos())
            painter->drawLine(QLineF(kfold.x, kfold.y, kf.x, kf.y));
    }
    if (kf.pos() != pos())
        painter->drawLine(QLineF(kf.x, kf.y, x, y));

    // The points.
    double s = 0.03;
    painter->drawEllipse(pos(), s, s);
    painter->drawEllipse(predicted(dt).pos(), s, s);
}

// Maps the Line l from the coordinate frame of Pose p to world coordinates.
Hpm2D operator+(const Hpm2D& l, const Pose2D& p)
{
    Hpm2D c = l;
    c += p;
    return c;
}

// Maps the Line l into the coordinate frame of the Pose p.
Hpm2D operator-(const Hpm2D& l, const Pose2D& p)
{
    Hpm2D c = l;
    c -= p;
    return c;
}

// Maps the Line l from the coordinate frame of Pose p to world coordinates.
void operator+=(Hpm2D& l, const Pose2D& p)
{
    l.rotate(p.z);
    l.translate(p.x, p.y);
}

// Maps the Line l into the coordinate frame of the Pose p.
void operator-=(Hpm2D& l, const Pose2D& p)
{
    l.translate(-p.x, -p.y);
    l.rotate(-p.z);
}



QDebug operator<<(QDebug dbg, const Hpm2D &v)
{
    dbg.nospace();
    //dbg << "dt:" << v.dt << " x:" << v.pos() << " v:" << v.vel() << " a:" << v.acc() << " j:" << v.jerk();
    dbg << "dt:" << v.dt << " x:" << v.pos() << " v:" << v.vel() << " a:" << v.acc();
    dbg.setAutoInsertSpaces(true);
    return dbg;
}

QDataStream& operator<<(QDataStream& out, const Hpm2D &o)
{
    out << o.t;
    out << o.dt;
    out << o.x;
    out << o.y;
    out << o.vx;
    out << o.vy;
    out << o.ax;
    out << o.ay;
    out << o.jx;
    out << o.jy;
    return out;
}

QDataStream& operator>>(QDataStream& in, Hpm2D &o)
{
    in >> o.t;
    in >> o.dt;
    in >> o.x;
    in >> o.y;
    in >> o.vx;
    in >> o.vy;
    in >> o.ax;
    in >> o.ay;
    in >> o.jx;
    in >> o.jy;
    return in;
}
