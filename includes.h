#include <stddef.h>
#include <math.h>
#include <stdint.h>

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
static const double POLOIDAL_PRECISON = M_PI / 90; // i.e. 2 degrees
static const double TOROIDAL_PRECISON = M_PI / 90; // i.e. 2 degrees
//static const double POLOIDAL_PRECISON = M_PI / 90 * 6; // i.e. 12 degrees
//static const double TOROIDAL_PRECISON = M_PI / 90 * 6; // i.e. 12 degrees

typedef struct
{
    double x;
    double y;
    double z;
} Point;

typedef struct
{
    size_t numPoints;
    Point point[MAX_TRACKED_POINTS]; // lazy, but simple.
} TrackedObject;

double distance(Point a, Point b)
{
    float x = a.x - b.x;
    float y = a.y - b.y;
    float z = a.z - b.z;
    return sqrt(x*x + y*y + z*z);
}

typedef struct
{
    // row, column, (0,0) in upper left
    float val[3][3];
} Matrix3x3;



//###################################
// The following code came from http://stackoverflow.com/questions/23166898/efficient-way-to-calculate-a-3x3-rotation-matrix-from-the-rotation-defined-by-tw
// Need to check up on license terms and give proper attribution
// I think we'll be good with proper attribution, but don't want to assume without checking.

#include <math.h>
#include <float.h>


      /* -------------------------------------------------------------------- */
      /* Math Lib declarations */

      static void unit_m3(float m[3][3]);
static float dot_v3v3(const float a[3], const float b[3]);
static float normalize_v3(float n[3]);
static void cross_v3_v3v3(float r[3], const float a[3], const float b[3]);
static void mul_v3_v3fl(float r[3], const float a[3], float f);
static void ortho_v3_v3(float p[3], const float v[3]);
static void axis_angle_normalized_to_mat3_ex(
    float mat[3][3], const float axis[3],
    const float angle_sin, const float angle_cos);


/* -------------------------------------------------------------------- */
/* Main function */
void rotation_between_vecs_to_mat3(float m[3][3], const float v1[3], const float v2[3]);

/**
* Calculate a rotation matrix from 2 normalized vectors.
*
* v1 and v2 must be unit length.
*/
void rotation_between_vecs_to_mat3(float m[3][3], const float v1[3], const float v2[3])
{
    float axis[3];
    /* avoid calculating the angle */
    float angle_sin;
    float angle_cos;

    cross_v3_v3v3(axis, v1, v2);

    angle_sin = normalize_v3(axis);
    angle_cos = dot_v3v3(v1, v2);

    if (angle_sin > FLT_EPSILON) {
    axis_calc:
        axis_angle_normalized_to_mat3_ex(m, axis, angle_sin, angle_cos);
    }
    else {
        /* Degenerate (co-linear) vectors */
        if (angle_cos > 0.0f) {
            /* Same vectors, zero rotation... */
            unit_m3(m);
        }
        else {
            /* Colinear but opposed vectors, 180 rotation... */
            ortho_v3_v3(axis, v1);
            normalize_v3(axis);
            angle_sin = 0.0f;  /* sin(M_PI) */
            angle_cos = -1.0f;  /* cos(M_PI) */
            goto axis_calc;
        }
    }
}


/* -------------------------------------------------------------------- */
/* Math Lib */

static void unit_m3(float m[3][3])
{
    m[0][0] = m[1][1] = m[2][2] = 1.0;
    m[0][1] = m[0][2] = 0.0;
    m[1][0] = m[1][2] = 0.0;
    m[2][0] = m[2][1] = 0.0;
}

static float dot_v3v3(const float a[3], const float b[3])
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

static void cross_v3_v3v3(float r[3], const float a[3], const float b[3])
{
    r[0] = a[1] * b[2] - a[2] * b[1];
    r[1] = a[2] * b[0] - a[0] * b[2];
    r[2] = a[0] * b[1] - a[1] * b[0];
}

static void mul_v3_v3fl(float r[3], const float a[3], float f)
{
    r[0] = a[0] * f;
    r[1] = a[1] * f;
    r[2] = a[2] * f;
}

static float normalize_v3_v3(float r[3], const float a[3])
{
    float d = dot_v3v3(a, a);

    if (d > 1.0e-35f) {
        d = sqrtf(d);
        mul_v3_v3fl(r, a, 1.0f / d);
    }
    else {
        d = r[0] = r[1] = r[2] = 0.0f;
    }

    return d;
}

static float normalize_v3(float n[3])
{
    return normalize_v3_v3(n, n);
}

static int axis_dominant_v3_single(const float vec[3])
{
    const float x = fabsf(vec[0]);
    const float y = fabsf(vec[1]);
    const float z = fabsf(vec[2]);
    return ((x > y) ?
        ((x > z) ? 0 : 2) :
        ((y > z) ? 1 : 2));
}

static void ortho_v3_v3(float p[3], const float v[3])
{
    const int axis = axis_dominant_v3_single(v);

    switch (axis) {
    case 0:
        p[0] = -v[1] - v[2];
        p[1] = v[0];
        p[2] = v[0];
        break;
    case 1:
        p[0] = v[1];
        p[1] = -v[0] - v[2];
        p[2] = v[1];
        break;
    case 2:
        p[0] = v[2];
        p[1] = v[2];
        p[2] = -v[0] - v[1];
        break;
    }
}

/* axis must be unit length */
static void axis_angle_normalized_to_mat3_ex(
    float mat[3][3], const float axis[3],
    const float angle_sin, const float angle_cos)
{
    float nsi[3], ico;
    float n_00, n_01, n_11, n_02, n_12, n_22;

    ico = (1.0f - angle_cos);
    nsi[0] = axis[0] * angle_sin;
    nsi[1] = axis[1] * angle_sin;
    nsi[2] = axis[2] * angle_sin;

    n_00 = (axis[0] * axis[0]) * ico;
    n_01 = (axis[0] * axis[1]) * ico;
    n_11 = (axis[1] * axis[1]) * ico;
    n_02 = (axis[0] * axis[2]) * ico;
    n_12 = (axis[1] * axis[2]) * ico;
    n_22 = (axis[2] * axis[2]) * ico;

    mat[0][0] = n_00 + angle_cos;
    mat[0][1] = n_01 + nsi[2];
    mat[0][2] = n_02 - nsi[1];
    mat[1][0] = n_01 - nsi[2];
    mat[1][1] = n_11 + angle_cos;
    mat[1][2] = n_12 + nsi[0];
    mat[2][0] = n_02 + nsi[1];
    mat[2][1] = n_12 - nsi[0];
    mat[2][2] = n_22 + angle_cos;
}