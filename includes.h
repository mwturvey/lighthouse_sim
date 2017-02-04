
#ifndef __INCLUDES_H
#define __INCLUDES_H

#include <stddef.h>
#include <math.h>
#include <stdint.h>


typedef struct
{
    double x;
    double y;
    double z;
} Point;

typedef struct
{
    Point point; // location of the sensor on the tracked object;
    Point normal; // unit vector indicating the normal for the sensor
    double theta; // "horizontal" angular measurement from lighthouse radians
    double phi; // "vertical" angular measurement from lighthouse in radians.
} TrackedSensor;

typedef struct
{
    size_t numSensors;
    TrackedSensor sensor[0];
} TrackedObject;


#ifndef M_PI
#define M_PI 3.14159265358979323846264338327
#endif

#define SQUARED(x) ((x)*(x))

typedef union
{
    struct
    {
        unsigned char Blue;
        unsigned char Green;
        unsigned char Red;
        unsigned char Alpha;
    };
//    float float_value;
    uint32_t long_value;
} RGBValue;

static RGBValue RED = { .Red = 255, .Green = 0, .Blue = 0, .Alpha = 125 };
static RGBValue GREEN = { .Red = 0, .Green = 255, .Blue = 0, .Alpha = 125 };
static RGBValue BLUE = { .Red = 0, .Green = 0, .Blue = 255, .Alpha = 125 };

static const double WORLD_BOUNDS = 100;
#define MAX_TRACKED_POINTS 40

static const float DefaultPointsPerOuterDiameter = 60;
//static const double DefaultPointsPerOuterDiameter = 30;



typedef struct
{
    // row, column, (0,0) in upper left
    double val[3][3];
} Matrix3x3;




#endif