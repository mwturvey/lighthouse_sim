
#ifndef __VISUALIZATION_H
#define __VISUALIZATION_H

#include <stdio.h>
#include "includes.h"
#include "common.h"

extern int pointsWritten;

void writePoint(FILE *file, double x, double y, double z, unsigned int rgb);

void updateHeader(FILE * file);

void writeAxes(FILE * file);

void drawLineBetweenPoints(FILE *file, Point a, Point b, unsigned int color);

void writePcdHeader(FILE * file);

void writePointCloud(FILE *f, Point *pointCloud, unsigned int Color);

void markPointWithStar(FILE *file, Point point, unsigned int color);


#endif