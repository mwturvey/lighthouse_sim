#include "includes.h"
#include <memory.h>
#include <stdlib.h>
#include <assert.h>
#include <stdio.h>

static int pointsWritten = 0;

#define MAX_COLORS 18
static unsigned int COLORS[] = {
    0x00FFFF,
    0xFF00FF,
    0xFFFF00,
    0xFF0000,
    0x00FF00,
    0x0000FF,
    0x0080FF,
    0x8000FF,
    0x80FF00,
    0x00FF80,
    0xFF0080,
    0xFF8000,
    0x008080,
    0x800080,
    0x808000,
    0x000080,
    0x008000,
    0x800000
};
Matrix3x3 GetRotationMatrixForTorus(Point a, Point b)
{
    Matrix3x3 result;
    float v1[3] = { 0, 0, 1 };
    float v2[3] = { a.x - b.x, a.y - b.y, a.z - b.z };

    normalize_v3(v2);

    rotation_between_vecs_to_mat3(result.val, v1, v2);

    return result;
}

Point RotateAndTranslatePoint(Point p, Matrix3x3 rot, Point newOrigin)
{
    Point q;

    float pf[3] = { p.x, p.y, p.z };
    //float pq[3];

    //q.x = rot.val[0][0] * p.x + rot.val[0][1] * p.y + rot.val[0][2] * p.z + newOrigin.x;
    //q.y = rot.val[1][0] * p.x + rot.val[1][1] * p.y + rot.val[1][2] * p.z + newOrigin.y;
    //q.z = rot.val[2][0] * p.x + rot.val[2][1] * p.y + rot.val[2][2] * p.z + newOrigin.z;
    q.x = rot.val[0][0] * p.x + rot.val[1][0] * p.y + rot.val[2][0] * p.z + newOrigin.x;
    q.y = rot.val[0][1] * p.x + rot.val[1][1] * p.y + rot.val[2][1] * p.z + newOrigin.y;
    q.z = rot.val[0][2] * p.x + rot.val[1][2] * p.y + rot.val[2][2] * p.z + newOrigin.z;

    return q;
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

Point midpoint(Point a, Point b)
{
    Point m;
    m.x = (a.x + b.x) / 2;
    m.y = (a.y + b.y) / 2;
    m.z = (a.z + b.z) / 2;

    return m;
}


void torusGenerator(Point p1, Point p2, double lighthouseAngle, Point **pointCloud)
{
    double poloidalRadius = 0;
    double toroidalRadius = 0;

    Point m = midpoint(p1, p2);
    double distanceBetweenPoints = distance(p1, p2);

    // ideally should only need to be lighthouseAngle, but increasing it here keeps us from accidentally
    // thinking the tori converge at the location of the tracked object instead of at the lighthouse.
    double centralAngleToIgnore = lighthouseAngle * 3; 

    Matrix3x3 rot = GetRotationMatrixForTorus(p1, p2);

    toroidalRadius = distanceBetweenPoints / (2 * tan(lighthouseAngle));

    poloidalRadius = sqrt(pow(toroidalRadius, 2) + pow(distanceBetweenPoints / 2, 2));

    unsigned int pointCount = (unsigned int)((M_PI * 2 / TOROIDAL_PRECISON + 1) *  ((M_PI - lighthouseAngle) * 2 / POLOIDAL_PRECISON + 1) +1);

    *pointCloud = malloc((pointCount+ 1000) * sizeof(Point));
    (*pointCloud)[pointCount - 1].x = -1000;
    (*pointCloud)[pointCount - 1].y = -1000;
    (*pointCloud)[pointCount - 1].z = -1000; // need a better magic number or flag, but this'll do for now.

    size_t currentPoint = 0;
    for (double toroidalStep = 0; toroidalStep < M_PI * 2; toroidalStep += TOROIDAL_PRECISON)
    {
        for (double poloidalStep = centralAngleToIgnore + M_PI; poloidalStep < M_PI * 3 - centralAngleToIgnore; poloidalStep += POLOIDAL_PRECISON)
        {
            if (currentPoint >= pointCount - 1)
            {
                int a = 0;
            }
            assert(currentPoint < pointCount - 1);
            (*pointCloud)[currentPoint].x = (toroidalRadius + poloidalRadius*cos(poloidalStep))*cos(toroidalStep);
            (*pointCloud)[currentPoint].y = (toroidalRadius + poloidalRadius*cos(poloidalStep))*sin(toroidalStep);
            (*pointCloud)[currentPoint].z = poloidalRadius*sin(poloidalStep);
            (*pointCloud)[currentPoint] = RotateAndTranslatePoint((*pointCloud)[currentPoint], rot, m);

            // TODO: HACK!!! Instead of doing anything with normals, we're "assuming" that all sensors point directly up
            // and hence we know that nothing with a negative z value is a possible lightouse location.
            // Before this code can go live, we'll have to take the normals into account and remove this hack.
            if ((*pointCloud)[currentPoint].z > 0) 
            {
                currentPoint++;
            }
        }
    }
    (*pointCloud)[currentPoint].x = -1000;
    (*pointCloud)[currentPoint].y = -1000;
    (*pointCloud)[currentPoint].z = -1000;
}

void writePoint(FILE *file, double x, double y, double z, unsigned int rgb)
{
    fprintf(file, "%f %f %f %u\n", x, y, z, rgb);
    pointsWritten++;
}

void updateHeader(FILE * file)
{
    fseek(file, 0x4C, SEEK_SET);
    fprintf(file, "%d", pointsWritten);
    fseek(file, 0x7C, SEEK_SET);
    fprintf(file, "%d", pointsWritten);
}
void writeAxes(FILE * file)
{
    for (double i = 0; i < 1000; i++)
    {
        writePoint(file, i / 10, 0, 0, 255);
    }
    for (double i = 0; i < 1000; i++)
    {
        if ((int)(i / 50) % 2 == 1)
        {
            writePoint(file, 0, i / 10, 0, 255<<8);
        }
    }
    for (double i = 0; i < 1000; i++)
    {
        if ((int)(i / 100) % 2 == 1)
        {
            writePoint(file, 0, 0, i / 10, 255<<16);
        }
    }
}

void drawLineBetweenPoints(FILE *file, Point a, Point b, unsigned int color)
{
    int max = 50;
    for (int i = 0; i < max; i++)
    {
        writePoint(file, 
            (a.x*i + b.x*(max - i)) / max,
            (a.y*i + b.y*(max - i)) / max,
            (a.z*i + b.z*(max - i)) / max,
            color);
    }
}

void writePcdHeader(FILE * file)
{
    fprintf(file, "VERSION 0.7\n");
    fprintf(file, "FIELDS  x y z rgb\n");
    fprintf(file, "SIZE 4 4 4 4\n");
    fprintf(file, "TYPE F F F U\n");
    fprintf(file, "COUNT 1 1 1 1\n");
    fprintf(file, "WIDTH        \n");
    fprintf(file, "HEIGHT 1\n");
    fprintf(file, "VIEWPOINT 0 0 0 1 0 0 0\n");
    fprintf(file, "POINTS        \n");
    fprintf(file, "DATA ascii\n");

    //fprintf(file, "100000.0, 100000.0, 100000\n");

}

void writePointCloud(FILE *f, Point *pointCloud, unsigned int Color)
{
    Point *currentPoint = pointCloud;

    while (currentPoint->x != -1000 || currentPoint->y != -1000 || currentPoint->z != -1000)
    {
        writePoint(f, currentPoint->x, currentPoint->y, currentPoint->z, Color);
        currentPoint++;
    }
}

void markPointWithStar(FILE *file, Point point, unsigned int color)
{
    float i;
    for ( i = -5; i <= 5; i=i +0.2)
    {
        writePoint(file, point.x + i, point.y, point.z, color);
        writePoint(file, point.x, point.y + i, point.z, color);
        writePoint(file, point.x, point.y, point.z + i, color);
    }

}

float FindSmallestDistance(Point p, Point* cloud)
{
    Point *cp = cloud;
    float smallestDistance = 10000000000000.0;

    while (cp->x != -1000 || cp->y != -1000 || cp->z != -1000)
    {
        float distance = (SQUARED(cp->x - p.x) + SQUARED(cp->y - p.y) + SQUARED(cp->z - p.z));
        if (distance < smallestDistance)
        {
            smallestDistance = distance;
        }
        cp++;
    }
    smallestDistance = sqrt(smallestDistance);
    return smallestDistance;
}

// Given a cloud and a list of clouds, find the point on masterCloud that best matches clouds.
Point findBestPointMatch(Point *masterCloud, Point** clouds, int numClouds)
{

    Point bestMatch;
    float bestDistance = 10000000000000.0;
    Point *cp = masterCloud;
    int point = 0;
    while (cp->x != -1000 || cp->y != -1000 || cp->z != -1000)
    {
        point++;
        if (point % 10000)
        {
            printf(".");
        }
        float currentDistance = 0;
        for (int i = 0; i < numClouds; i++)
        {
            if (clouds[i] == masterCloud)
            {
                continue;
            }
            Point* cloud = clouds[i];
            currentDistance += FindSmallestDistance(*cp, cloud);
        }

        if (currentDistance < bestDistance)
        {
            bestDistance = currentDistance;
            bestMatch = *cp;
        }
        cp++;
    }

    return bestMatch;
}


#define MAX_POINT_PAIRS 100

typedef struct
{
    Point a;
    Point b;
    double angle;
} PointsAndAngle;

double angleBetweenSensors(TrackedSensor *a, TrackedSensor *b)
{
    double angle = acos(cos(a->phi - b->phi)*cos(a->theta - b->theta));
    double angle2 = acos(cos(b->phi - a->phi)*cos(b->theta - a->theta));

    return angle;
}

Point SolveForLighthouse(TrackedObject *obj)
{
    PointsAndAngle pna[MAX_POINT_PAIRS];
    Point lh = { 100, 20, 5 };

    size_t pnaCount = 0;
    // TODO: Need a better way of picking the pairs to use that more does a better job
    // picking each sensor a more even amount of time (and maybe also looking for point pairs 
    // that would result in a more diverse set of vectors to better calculate position.
    for (unsigned int i = 0; i < obj->numSensors; i++)
    {
        for (unsigned int j = 0; j < i; j++)
        {
            if ( pnaCount < MAX_POINT_PAIRS)
            {
                pna[pnaCount].a = obj->sensor[i].point;
                pna[pnaCount].b = obj->sensor[j].point;

                pna[pnaCount].angle = angleBetweenSensors(&obj->sensor[i], &obj->sensor[j]);

                float pythAngle = sqrt(SQUARED(obj->sensor[i].phi - obj->sensor[j].phi) + SQUARED(obj->sensor[i].theta - obj->sensor[j].theta));

                double tmp = angleFromPoints(pna[pnaCount].a, pna[pnaCount].b, lh);

                int a=4;

                pnaCount++;
            }
        }
    }

    Point **pointCloud = malloc(sizeof(Point*)* pnaCount);

    FILE *f = fopen("pointcloud2.pcd", "wb");
    writePcdHeader(f);
    writeAxes(f);

    for (int i = 0; i < pnaCount; i++)
    {
        torusGenerator(pna[i].a, pna[i].b, pna[i].angle, &(pointCloud[i]));
        writePointCloud(f, pointCloud[i], COLORS[i%MAX_COLORS]);

    }
    //Point *pointCloud_ab = NULL;
    //Point *pointCloud_ac = NULL;
    //Point *pointCloud_bc = NULL;
    //Point *pointCloud_ad = NULL;
    //Point *pointCloud_bd = NULL;
    //Point *pointCloud_cd = NULL;
    //torusGenerator(a, b, angleFromPoints(a, b, lh), &pointCloud_ab);
    //torusGenerator(a, c, angleFromPoints(a, c, lh), &pointCloud_ac);
    //torusGenerator(b, c, angleFromPoints(b, c, lh), &pointCloud_bc);
    //torusGenerator(a, d, angleFromPoints(a, d, lh), &pointCloud_ad);
    //torusGenerator(b, d, angleFromPoints(b, d, lh), &pointCloud_bd);
    //torusGenerator(c, d, angleFromPoints(c, d, lh), &pointCloud_cd);


    markPointWithStar(f, lh, 0xFF0000);

    //drawLineBetweenPoints(f, a, b, 255);
    //drawLineBetweenPoints(f, b, c, 255);
    //drawLineBetweenPoints(f, a, c, 255);
    //drawLineBetweenPoints(f, a, d, 255);
    //drawLineBetweenPoints(f, b, d, 255);
    //drawLineBetweenPoints(f, c, d, 255);

    Point bestMatchA = findBestPointMatch(pointCloud[0], pointCloud, pnaCount);

    markPointWithStar(f, bestMatchA, 0xFFFFFF);

    updateHeader(f);

    fclose(f);

}

static Point makeUnitPoint(Point *p)
{
    Point newP;
    float r = sqrt(p->x*p->x + p->y*p->y + p->z*p->z);
    newP.x = p->x / r;
    newP.y = p->y / r;
    newP.z = p->z / r;

    return newP;
}

static float getPhi(Point p)
{
    float phi = acos(p.z / (sqrt(p.x*p.x + p.y*p.y + p.z*p.z)));
//    float phi = atan(sqrt(p.x*p.x + p.y*p.y)/p.z);
    return phi;
}

static float getTheta(Point p)
{
    float theta = atan(p.y / p.x);
    return theta;
}

// subtraction
static Point PointSub(Point a, Point b)
{
    Point newPoint;

    newPoint.x = a.x - b.x;
    newPoint.y = a.y - b.y;
    newPoint.z = a.z - b.z;

    return newPoint;
}

int main()
{

    //Point lh = { 0, 0, 0 };
    Point lh = { 100, 20, 5 };

    TrackedObject *to;

    to = malloc(sizeof(TrackedObject)+4 * sizeof(TrackedSensor));

    to->numSensors = 4;
    to->sensor[0].point.x = 5.0;
    to->sensor[0].point.y = 0.00001;
    to->sensor[0].point.z = 0.00001;

    to->sensor[1].point.x = 0.00001;
    to->sensor[1].point.y = 5.00001;
    to->sensor[1].point.z = 0.00001;

    to->sensor[2].point.x = 0.00001;
    to->sensor[2].point.y = 0.00001;
    to->sensor[2].point.z = 5.00001;

    to->sensor[3].point.x = 0.00001;
    to->sensor[3].point.y = 5.00001;
    to->sensor[3].point.z = 5.00001;

    for (int i = 0; i < to->numSensors; i++)
    {
        float tmp = getTheta(PointSub(to->sensor[i].point, lh));
        float tmp2 = getPhi(PointSub(to->sensor[i].point, lh));

        float tmpD = tmp * 180 / M_PI;
        float tmp2D = tmp2 * 180 / M_PI;

        to->sensor[i].theta = getTheta(PointSub(to->sensor[i].point, lh));
        to->sensor[i].phi = getPhi(PointSub(to->sensor[i].point, lh));
        int a = 0;
    }


    Point foundLh = SolveForLighthouse(to);



    FILE *f = fopen("pointcloud.pcd", "wb");

    Point a = { 5.0, 0.0 , 0};
    Point b = { 0.0, 5.0 , 0};
    //Point c = { 3.54, 3.54, 5.0 };
    Point c = { 0, 0, 5.0 };
    Point d = { 0, 5, 5.0 };
    //Point lh = { 50, 50, 50 };


    Point *pointCloud_ab = NULL;
    Point *pointCloud_ac = NULL;
    Point *pointCloud_bc = NULL;
    Point *pointCloud_ad = NULL;
    Point *pointCloud_bd = NULL;
    Point *pointCloud_cd = NULL;
    torusGenerator(a, b, angleFromPoints(a, b, lh), &pointCloud_ab);
    torusGenerator(a, c, angleFromPoints(a, c, lh), &pointCloud_ac);
    torusGenerator(b, c, angleFromPoints(b, c, lh), &pointCloud_bc);
    torusGenerator(a, d, angleFromPoints(a, d, lh), &pointCloud_ad);
    torusGenerator(b, d, angleFromPoints(b, d, lh), &pointCloud_bd);
    torusGenerator(c, d, angleFromPoints(c, d, lh), &pointCloud_cd);

    writePcdHeader(f);

    writeAxes(f);

    writePointCloud(f, pointCloud_ab, 0x00FFFF);
    writePointCloud(f, pointCloud_ac, 0xFFFFFF);
    writePointCloud(f, pointCloud_bc, 0xFF00FF);
    writePointCloud(f, pointCloud_ad, 0x000080);
    writePointCloud(f, pointCloud_bd, 0x008000);
    writePointCloud(f, pointCloud_cd, 0x800000);

    markPointWithStar(f, lh, 0xFF0000);

    drawLineBetweenPoints(f, a, b, 255);
    drawLineBetweenPoints(f, b, c, 255);
    drawLineBetweenPoints(f, a, c, 255);
    drawLineBetweenPoints(f, a, d, 255);
    drawLineBetweenPoints(f, b, d, 255);
    drawLineBetweenPoints(f, c, d, 255);

    Point *clouds[6];

    clouds[0] = pointCloud_ab;
    clouds[1] = pointCloud_ac;
    clouds[2] = pointCloud_bc;
    clouds[3] = pointCloud_ad;
    clouds[4] = pointCloud_bd;
    clouds[5] = pointCloud_cd;

    Point bestMatchA = findBestPointMatch(pointCloud_ab, clouds, 6);

    markPointWithStar(f, bestMatchA, 0x00FFFF);

    updateHeader(f);


    fclose(f);

    return 0;
}