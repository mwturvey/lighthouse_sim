#define _USE_MATH_DEFINES
#include <math.h>

typedef struct
{
    double x;
    double y;
    double z;

    double phi;
    double theta;
} Point;


// subtraction
static Point PointSub(Point a, Point b)
{
    Point newPoint;

    newPoint.x = a.x - b.x;
    newPoint.y = a.y - b.y;
    newPoint.z = a.z - b.z;

    return newPoint;
}

double angleFromPoints(Point p1, Point p2, Point center)
{
    Point v1, v2, v1norm, v2norm;
    v1.x = p1.x - center.x;
    v1.y = p1.y - center.y;
    v1.z = p1.z - center.z;

    v2.x = p2.x - center.x;
    v2.y = p2.y - center.y;
    v2.z = p2.z - center.z;

    double a = v1.x*v1.x;
    double b = v1.y*v1.y;
    double c = v1.z*v1.z;
    double d = a + b + c;
    double e = sqrt(d);
    double v1mag = sqrt(v1.x * v1.x + v1.y * v1.y + v1.z * v1.z);
    v1norm.x = v1.x / v1mag;
    v1norm.y = v1.y / v1mag;
    v1norm.z = v1.z / v1mag;

    double v2mag = sqrt(v2.x * v2.x + v2.y * v2.y + v2.z * v2.z);
    v2norm.x = v2.x / v2mag;
    v2norm.y = v2.y / v2mag;
    v2norm.z = v2.z / v2mag;

    double res = v1norm.x * v2norm.x + v1norm.y * v2norm.y + v1norm.z * v2norm.z;

    double angle = acos(res);

    return angle;
}

double angleBetweenSensors(Point *a, Point *b)
{
    double angle = acos(cos(a->phi - b->phi)*cos(a->theta - b->theta));
    double angle2 = acos(cos(b->phi - a->phi)*cos(b->theta - a->theta));


    return angle;
}
double angleBetweenSensors2(Point *a, Point *b)
{
    double p = (a->phi - b->phi);
    double d = (a->theta - b->theta);

    double adjd = sin((a->phi + b->phi) / 2);
    double adjP = sin((a->theta + b->theta) / 2);

    double angle = acos(cos((a->phi - b->phi)*adjP)*cos((a->theta - b->theta)*adjd));
    double angle2 = acos(cos((b->phi - a->phi)*adjP)*cos((b->theta - a->theta)*adjd));


    return angle;
}

#define SQUARED(x) ((x)*(x))

double pythAngleBetweenSensors(Point *a, Point *b)
{
    double pythAngle = sqrt(SQUARED(a->phi - b->phi) + SQUARED(a->theta - b->theta));

    return pythAngle;

}
double pythAngleBetweenSensors2(Point *a, Point *b)
{
    double p = (a->phi - b->phi);
    double d = (a->theta - b->theta);

    double adjd = sin((a->phi + b->phi)/2);
    double adjP = sin((a->theta + b->theta) / 2);
    double pythAngle = sqrt(SQUARED(p*adjP) + SQUARED(d*adjd));
    return pythAngle;
}

static double getPhi(Point p)
{
    //    double phi = acos(p.z / (sqrt(p.x*p.x + p.y*p.y + p.z*p.z)));
    //    double phi = atan(sqrt(p.x*p.x + p.y*p.y)/p.z);
    double phi = atan(p.x / p.z);
    return phi;
}

static double getSphericalPhi(Point p)
{
    double phi = acos(p.z / (sqrt(p.x*p.x + p.y*p.y + p.z*p.z)));
    return phi;
}
static double getTheta(Point p)
{
    //double theta = atan(p.y / p.x);
    double theta = atan(p.x / p.y);
    return theta;
}
int main()
{
    Point p1;
    Point p2;
    Point lh;

    lh.x = 0;
    lh.y = 0;
    lh.z = 0;
    //lh.z = 000;

    p1.x = 54.00001;
    p1.y = 1.00000;
    p1.z = 11.00001;
    p1.phi = getPhi(PointSub(p1,lh));
    p1.theta = getTheta(PointSub(p1, lh));
    double p1phi = p1.phi * 180 / M_PI;
    double p1theta = p1.theta * 180 / M_PI;
    double p1SPhi = getSphericalPhi(p1) * 180 / M_PI;

    p2.x = 52.00001;
    p2.y = 0.00000;
    p2.z = 10.00001;
    p2.phi = getPhi(PointSub(p2, lh));
    p2.theta = getTheta(PointSub(p2, lh));
    double p2phi = p2.phi * 180 / M_PI;
    double p2theta = p2.theta * 180 / M_PI;
    double p2SPhi = getSphericalPhi(p2) * 180 / M_PI;

    double deltaTheta = p2.theta - p1.theta;
    double deltaPhi = p2.phi - p1.phi;
    double deltaThetaD = deltaTheta * 180 / M_PI;
    double deltaPhiD = deltaPhi * 180 / M_PI;

    double angle = 180/M_PI*angleFromPoints(p1, p2, lh);
    double angle2 = 180 / M_PI*angleBetweenSensors(&p1, &p2);
    double angle3 = 180 / M_PI*angleBetweenSensors2(&p1, &p2);
    double angle4 = 180 / M_PI*pythAngleBetweenSensors(&p1, &p2);
    double angle5 = 180 / M_PI*pythAngleBetweenSensors2(&p1, &p2);


    return 0;
}