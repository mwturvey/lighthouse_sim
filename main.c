
#include <stdio.h>
#include <assert.h>
#include "TorusLocalizer.h"
#include <memory.h>
#include <time.h>

void testRealData()
{

    TrackedObject *to;

    to = malloc(sizeof(TrackedObject) + (14 * sizeof(TrackedSensor)));

    to->numSensors = 14;

    //0	190648.3331	158215.2782
    to->sensor[0].point.x = 0.0851814;
    to->sensor[0].point.y = 0.0170621;
    to->sensor[0].point.z = 0.0464036;
    to->sensor[0].theta = 190648.3331;
    to->sensor[0].phi = 158215.2782;

    //4	190442.1726	160237.0121
    to->sensor[1].point.x = 0.0799671;
    to->sensor[1].point.y = 0.0452252;
    to->sensor[1].point.z = 0.0347871;
    to->sensor[1].theta = 190442.1726;
    to->sensor[1].phi = 160237.0121;

    //6	194906.0781	159445.4121
    to->sensor[2].point.x = 0.0243163;
    to->sensor[2].point.y = 0.0200039;
    to->sensor[2].point.z = 0.0594331;
    to->sensor[2].theta = 194906.0781;
    to->sensor[2].phi = 159445.4121;

    //7	193054.9208	159914.2366
    to->sensor[3].point.x = 0.047366;
    to->sensor[3].point.y = 0.0335892;
    to->sensor[3].point.z = 0.0535793;
    to->sensor[3].theta = 193054.9208;
    to->sensor[3].phi = 159914.2366;

    //8	194056.9867	155690.5243
    to->sensor[4].point.x = 0.0477814;
    to->sensor[4].point.y = -0.0340002;
    to->sensor[4].point.z = 0.0534839;
    to->sensor[4].theta = 194056.9867;
    to->sensor[4].phi = 155690.5243;

    //9	192905	157564.8921
    to->sensor[5].point.x = 0.0579574;
    to->sensor[5].point.y = -3.65E-05;
    to->sensor[5].point.z = 0.056517;
    to->sensor[5].theta = 192905;
    to->sensor[5].phi = 157564.8921;

    //15	191133.3968	156099.2932
    to->sensor[6].point.x = 0.0852848;
    to->sensor[6].point.y = -0.0171755;
    to->sensor[6].point.z = 0.0464536;
    to->sensor[6].theta = 191133.3968;
    to->sensor[6].phi = 156099.2932;

    //16	199287.4782	161832.5252
    to->sensor[7].point.x = -0.047897;
    to->sensor[7].point.y = 0.0336478;
    to->sensor[7].point.z = 0.053597;
    to->sensor[7].theta = 199287.4782;
    to->sensor[7].phi = 161832.5252;

    //17	198090.2071	160430.2143
    to->sensor[8].point.x = -0.0245155;
    to->sensor[8].point.y = 0.0200543;
    to->sensor[8].point.z = 0.0593949;
    to->sensor[8].theta = 198090.2071;
    to->sensor[8].phi = 160430.2143;

    //23	201748.4484	161651.8288
    to->sensor[9].point.x = -0.0853892;
    to->sensor[9].point.y = 0.0173502;
    to->sensor[9].point.z = 0.0463133;
    to->sensor[9].theta = 201748.4484;
    to->sensor[9].phi = 161651.8288;

    //24	202248.2187	159553.2756
    to->sensor[10].point.x = -0.0852688;
    to->sensor[10].point.y = -0.0171002;
    to->sensor[10].point.z = 0.0462514;
    to->sensor[10].theta = 202248.2187;
    to->sensor[10].phi = 159553.2756;

    //29	199112.6078	156248.6293
    to->sensor[11].point.x = -0.0272585;
    to->sensor[11].point.y = -0.0516152;
    to->sensor[11].point.z = 0.0468868;
    to->sensor[11].theta = 199112.6078;
    to->sensor[11].phi = 156248.6293;

    //30	200482.5997	159912.9083
    to->sensor[12].point.x = -0.0580757;
    to->sensor[12].point.y = 6.80E-06;
    to->sensor[12].point.z = 0.0565004;
    to->sensor[12].theta = 200482.5997;
    to->sensor[12].phi = 159912.9083;

    //31	200340.6642	157639.705
    to->sensor[13].point.x = -0.0475571;
    to->sensor[13].point.y = -0.0339427;
    to->sensor[13].point.z = 0.0535213;
    to->sensor[13].theta = 200340.6642;
    to->sensor[13].phi = 157639.705;



    for (int i = 0; i < 14; i++)
    {
        // convert from clock cycles to degrees
        to->sensor[i].theta = to->sensor[i].theta / 48000000 * 60 * M_PI * 2;
        to->sensor[i].phi = to->sensor[i].phi / 48000000 * 60 * M_PI * 2;
    }

    Point foundLh = SolveForLighthouse(to, 0);

    free(to);
}


void testRealData2()
{

    TrackedObject *to;

    to = malloc(sizeof(TrackedObject) + (50 * sizeof(TrackedSensor)));


    int i = 0;

    // 0
    to->sensor[i].point.x = 0.0851814;
    to->sensor[i].point.y = 0.0170621;
    to->sensor[i].point.z = 0.0464036;
    to->sensor[i].theta = 173656.227498;
    to->sensor[i].phi = 204736.018055;
    i++;

    //// 1
    //to->sensor[i].point.x = 0.0929987;
    //to->sensor[i].point.y = -9.77111e-05;
    //to->sensor[i].point.z = 0.034903;
    //to->sensor[i].theta = ;
    //to->sensor[i].phi = ;
    //i++;

    //// 2
    //to->sensor[i].point.x = 0.0866358;
    //to->sensor[i].point.y = 0.01655;
    //to->sensor[i].point.z = 0.0205866;
    //to->sensor[i].theta = ;
    //to->sensor[i].phi = ;
    //i++;

    //// 3
    //to->sensor[i].point.x = 0.0896136;
    //to->sensor[i].point.y = 0.0291564;
    //to->sensor[i].point.z = 0.0296088;
    //to->sensor[i].theta = ;
    //to->sensor[i].phi = ;
    //i++;

    // 4
    to->sensor[i].point.x = 0.0799671;
    to->sensor[i].point.y = 0.0452252;
    to->sensor[i].point.z = 0.0347871;
    to->sensor[i].theta = 172643.620157;
    to->sensor[i].phi = 205062.361095;
    i++;

    // 5
    to->sensor[i].point.x = 0.050822;
    to->sensor[i].point.y = 0.0525379;
    to->sensor[i].point.z = 0.0332851;
    to->sensor[i].theta = 172551.577338;
    to->sensor[i].phi = 206242.149097;
    i++;

    // 6
    to->sensor[i].point.x = 0.0243163;
    to->sensor[i].point.y = 0.0200039;
    to->sensor[i].point.z = 0.0594331;
    to->sensor[i].theta = 174201.746577;
    to->sensor[i].phi = 207033.149388;
    i++;

    // 7
    to->sensor[i].point.x = 0.047366;
    to->sensor[i].point.y = 0.0335892;
    to->sensor[i].point.z = 0.0535793;
    to->sensor[i].theta = 173608.039324;
    to->sensor[i].phi = 206211.706756;
    i++;

    //// 8
    //to->sensor[i].point.x = 0.0477814;
    //to->sensor[i].point.y = -0.0340002;
    //to->sensor[i].point.z = 0.0534839;
    //to->sensor[i].theta = ;
    //to->sensor[i].phi = ;
    //i++;

    // 9
    to->sensor[i].point.x = 0.0579574;
    to->sensor[i].point.y = -3.65101e-05;
    to->sensor[i].point.z = 0.056517;
    to->sensor[i].theta = 174465.920186;
    to->sensor[i].phi = 205663.607746;
    i++;

    //// 10
    //to->sensor[i].point.x = 0.027572;
    //to->sensor[i].point.y = -0.051707;
    //to->sensor[i].point.z = 0.046649;
    //to->sensor[i].theta = ;
    //to->sensor[i].phi = ;
    //i++;

    //// 11
    //to->sensor[i].point.x = 0.0514582;
    //to->sensor[i].point.y = -0.0529347;
    //to->sensor[i].point.z = 0.0331235;
    //to->sensor[i].theta = ;
    //to->sensor[i].phi = ;
    //i++;

    //// 12
    //to->sensor[i].point.x = 0.0805458;
    //to->sensor[i].point.y = -0.0452235;
    //to->sensor[i].point.z = 0.0346787;
    //to->sensor[i].theta = ;
    //to->sensor[i].phi = ;
    //i++;

    //// 13
    //to->sensor[i].point.x = 0.0899552;
    //to->sensor[i].point.y = -0.0293091;
    //to->sensor[i].point.z = 0.0296856;
    //to->sensor[i].theta = ;
    //to->sensor[i].phi = ;
    //i++;

    //// 14
    //to->sensor[i].point.x = 0.0868583;
    //to->sensor[i].point.y = -0.0166452;
    //to->sensor[i].point.z = 0.0205461;
    //to->sensor[i].theta = ;
    //to->sensor[i].phi = ;
    //i++;

    //// 15
    //to->sensor[i].point.x = 0.0852848;
    //to->sensor[i].point.y = -0.0171755;
    //to->sensor[i].point.z = 0.0464536;
    //to->sensor[i].theta = ;
    //to->sensor[i].phi = ;
    //i++;

    // 16
    to->sensor[i].point.x = -0.047897;
    to->sensor[i].point.y = 0.0336478;
    to->sensor[i].point.z = 0.053597;
    to->sensor[i].theta = 174012.703466;
    to->sensor[i].phi = 209888.311881;
    i++;

    // 17
    to->sensor[i].point.x = -0.0245155;
    to->sensor[i].point.y = 0.0200543;
    to->sensor[i].point.z = 0.0593949;
    to->sensor[i].theta = 174408.549956;
    to->sensor[i].phi = 208907.731508;
    i++;

    //// 18
    //to->sensor[i].point.x = -0.0512064;
    //to->sensor[i].point.y = 0.0528224;
    //to->sensor[i].point.z = 0.0331849;
    //to->sensor[i].theta = ;
    //to->sensor[i].phi = ;
    //i++;

    // 19
    to->sensor[i].point.x = -0.0802602;
    to->sensor[i].point.y = 0.0452913;
    to->sensor[i].point.z = 0.0348134;
    to->sensor[i].theta = 173321.909991;
    to->sensor[i].phi = 211270.631625;
    i++;

    //// 20
    //to->sensor[i].point.x = -0.0897543;
    //to->sensor[i].point.y = 0.0293923;
    //to->sensor[i].point.z = 0.0296238;
    //to->sensor[i].theta = ;
    //to->sensor[i].phi = ;
    //i++;

    //// 21
    //to->sensor[i].point.x = -0.0867611;
    //to->sensor[i].point.y = 0.0166726;
    //to->sensor[i].point.z = 0.020728;
    //to->sensor[i].theta = ;
    //to->sensor[i].phi = ;
    //i++;

    //// 22
    //to->sensor[i].point.x = -0.0929696;
    //to->sensor[i].point.y = 0.000195598;
    //to->sensor[i].point.z = 0.0349093;
    //to->sensor[i].theta = ;
    //to->sensor[i].phi = ;
    //i++;

    //// 23
    //to->sensor[i].point.x = -0.0853892;
    //to->sensor[i].point.y = 0.0173502;
    //to->sensor[i].point.z = 0.0463133;
    //to->sensor[i].theta = ;
    //to->sensor[i].phi = ;
    //i++;

    //// 24
    //to->sensor[i].point.x = -0.0852688;
    //to->sensor[i].point.y = -0.0171002;
    //to->sensor[i].point.z = 0.0462514;
    //to->sensor[i].theta = ;
    //to->sensor[i].phi = ;
    //i++;

    //// 25
    //to->sensor[i].point.x = -0.086695;
    //to->sensor[i].point.y = -0.0164564;
    //to->sensor[i].point.z = 0.0207053;
    //to->sensor[i].theta = ;
    //to->sensor[i].phi = ;
    //i++;

    //// 26
    //to->sensor[i].point.x = -0.0895888;
    //to->sensor[i].point.y = -0.0292942;
    //to->sensor[i].point.z = 0.0297272;
    //to->sensor[i].theta = ;
    //to->sensor[i].phi = ;
    //i++;

    //// 27
    //to->sensor[i].point.x = -0.0801986;
    //to->sensor[i].point.y = -0.0452252;
    //to->sensor[i].point.z = 0.0346869;
    //to->sensor[i].theta = ;
    //to->sensor[i].phi = ;
    //i++;

    //// 28
    //to->sensor[i].point.x = -0.0509185;
    //to->sensor[i].point.y = -0.0527843;
    //to->sensor[i].point.z = 0.0331621;
    //to->sensor[i].theta = ;
    //to->sensor[i].phi = ;
    //i++;

    //// 29
    //to->sensor[i].point.x = -0.0272585;
    //to->sensor[i].point.y = -0.0516152;
    //to->sensor[i].point.z = 0.0468868;
    //to->sensor[i].theta = ;
    //to->sensor[i].phi = ;
    //i++;

    // 30
    to->sensor[i].point.x = -0.0580757;
    to->sensor[i].point.y = 6.80145e-06;
    to->sensor[i].point.z = 0.0565004;
    to->sensor[i].theta = 174963.711040;
    to->sensor[i].phi = 210130.630751;
    //i++;

    //// 31
    //to->sensor[i].point.x = -0.0475571;
    //to->sensor[i].point.y = -0.0339427;
    //to->sensor[i].point.z = 0.0535213;
    //to->sensor[i].theta = ;
    //to->sensor[i].phi = ;

    to->numSensors = i;


    to->numSensors = 4;


    for (int j = 0; j < i; j++)
    {
        // convert from clock cycles to degrees
        to->sensor[j].theta = to->sensor[j].theta / 48000000 * 60 * M_PI * 2;
        to->sensor[j].phi = to->sensor[j].phi / 48000000 * 60 * M_PI * 2;
    }

    Point foundLh = SolveForLighthouse(to, 1);

    printf("(%f,%f,%f)\n", foundLh.x, foundLh.y, foundLh.z);

    getchar();
    free(to);
}

int main()
{
    testRealData2();

    exit(1);
    //Point lh = { 0, 0, 0 };
    Point lh = { 0, 200, 10 };

    TrackedObject *to;

    to = malloc(sizeof(TrackedObject) + 6 * sizeof(TrackedSensor));

    to->numSensors = 6;
    to->sensor[0].point.x = 18.0;
    to->sensor[0].point.y = 4.0;
    to->sensor[0].point.z = 3.0;

    to->sensor[1].point.x = 0.00001;
    to->sensor[1].point.y = 0.00001;
    to->sensor[1].point.z = 0.00001;

    to->sensor[2].point.x = 5.0;
    to->sensor[2].point.y = 10.0;
    to->sensor[2].point.z = 3.0;

    to->sensor[3].point.x = 0.00001;
    to->sensor[3].point.y = 21.0;
    to->sensor[3].point.z = 0.00001;

    to->sensor[4].point.x = 13.0;
    to->sensor[4].point.y = 18.0;
    to->sensor[4].point.z = 3.0;

    to->sensor[5].point.x = 23.0;
    to->sensor[5].point.y = 21.0;
    to->sensor[5].point.z = 0.00001;

    //for (int i = 0; i < to->numSensors; i++)
    //{
    //    double tmp = getTheta(PointSub(to->sensor[i].point, lh));
    //    double tmp2 = getPhi(PointSub(to->sensor[i].point, lh));

    //    double tmpD = tmp * 180 / M_PI;
    //    double tmp2D = tmp2 * 180 / M_PI;

    //    to->sensor[i].theta = getTheta(PointSub(to->sensor[i].point, lh));
    //    to->sensor[i].phi = getPhi(PointSub(to->sensor[i].point, lh));
    //    int a = 0;
    //}
#define RADIANS(x) (x / 180 * M_PI)

    //    0:(79.138, 91.254) 1 : (83.977, 92.572) 2 : (82.497, 89.744) 3 : (84.135, 86.627) 4 : (80.580, 87.252) 5 : (77.591, 86.291)
    //to->sensor[0].theta = RADIANS(79.138);
    //to->sensor[0].phi = RADIANS(91.254);
    //to->sensor[1].theta = RADIANS(83.977);
    //to->sensor[1].phi = RADIANS(92.572);
    //to->sensor[2].theta = RADIANS(82.497);
    //to->sensor[2].phi = RADIANS(89.744);
    //to->sensor[3].theta = RADIANS(84.135);
    //to->sensor[3].phi = RADIANS(86.627);
    //to->sensor[4].theta = RADIANS(80.580);
    //to->sensor[4].phi = RADIANS(87.252);
    //to->sensor[5].theta = RADIANS(77.591);
    //to->sensor[5].phi = RADIANS(86.291);

    //    0:(79.821, 87.904) 1 : (78.427, 93.046) 2 : (81.371, 91.689) 3 : (84.387, 93.179) 4 : (83.767, 89.468) 5 : (84.603, 86.709)
    //to->sensor[0].theta = RADIANS(79.821);
    //to->sensor[0].phi = RADIANS(87.904);
    //to->sensor[1].theta = RADIANS(78.427);
    //to->sensor[1].phi = RADIANS(93.046);
    //to->sensor[2].theta = RADIANS(81.371);
    //to->sensor[2].phi = RADIANS(91.689);
    //to->sensor[3].theta = RADIANS(84.387);
    //to->sensor[3].phi = RADIANS(93.179);
    //to->sensor[4].theta = RADIANS(83.767);
    //to->sensor[4].phi = RADIANS(89.468);
    //to->sensor[5].theta = RADIANS(84.603);
    //to->sensor[5].phi = RADIANS(86.709);

    //0:(83.614, 88.358) 1 : (78.345, 86.965) 2 : (79.828, 89.899) 3 : (78.185, 93.070) 4 : (80.894, 91.510) 5 : (84.688, 93.237)
    //to->sensor[0].theta = RADIANS(83.614);
    //to->sensor[0].phi = RADIANS(88.358);
    //to->sensor[1].theta = RADIANS(78.345);
    //to->sensor[1].phi = RADIANS(86.965);
    //to->sensor[2].theta = RADIANS(79.828);
    //to->sensor[2].phi = RADIANS(89.899);
    //to->sensor[3].theta = RADIANS(78.185);
    //to->sensor[3].phi = RADIANS(93.070);
    //to->sensor[4].theta = RADIANS(80.894);
    //to->sensor[4].phi = RADIANS(91.510);
    //to->sensor[5].theta = RADIANS(84.688);
    //to->sensor[5].phi = RADIANS(93.237);

    //0:(88.350, 135.574) 1 : (95.278, 136.934) 2 : (93.468, 134.097) 3 : (95.171, 131.030) 4 : (90.511, 131.695) 5 : (86.909, 130.729)
    //to->sensor[0].theta = RADIANS(88.350);
    //to->sensor[0].phi = RADIANS(135.574);
    //to->sensor[1].theta = RADIANS(95.278);
    //to->sensor[1].phi = RADIANS(136.934);
    //to->sensor[2].theta = RADIANS(93.468);
    //to->sensor[2].phi = RADIANS(134.097);
    //to->sensor[3].theta = RADIANS(95.171);
    //to->sensor[3].phi = RADIANS(131.030);
    //to->sensor[4].theta = RADIANS(90.511);
    //to->sensor[4].phi = RADIANS(131.695);
    //to->sensor[5].theta = RADIANS(86.909);
    //to->sensor[5].phi = RADIANS(130.729);

    //0:(84.606, 90.598) 1 : (87.230, 91.840) 2 : (87.040, 89.078) 3 : (87.361, 86.069) 4 : (85.603, 86.617) 5 : (82.989, 85.470)
    to->sensor[0].theta = RADIANS(84.606);
    to->sensor[0].phi = RADIANS(90.598);
    to->sensor[1].theta = RADIANS(87.230);
    to->sensor[1].phi = RADIANS(91.840);
    to->sensor[2].theta = RADIANS(87.040);
    to->sensor[2].phi = RADIANS(89.078);
    to->sensor[3].theta = RADIANS(87.361);
    to->sensor[3].phi = RADIANS(86.069);
    to->sensor[4].theta = RADIANS(85.603);
    to->sensor[4].phi = RADIANS(86.617);
    to->sensor[5].theta = RADIANS(82.989);
    to->sensor[5].phi = RADIANS(85.470);

    to->numSensors = 3;

    time_t startTime = time(NULL);

    for (int i = 0; i < 500; i++)
    {
        Point foundLh = SolveForLighthouse(to, 0);
    }

    time_t endTime = time(NULL);

    time_t duration = difftime(endTime, startTime);




    exit(1);

    //FILE *f = fopen("pointcloud.pcd", "wb");

    //Point a = { 5.0, 0.0, 0 };
    //Point b = { 0.0, 5.0, 0 };
    ////Point c = { 3.54, 3.54, 5.0 };
    //Point c = { 0, 0, 5.0 };
    //Point d = { 0, 5, 5.0 };
    ////Point lh = { 50, 50, 50 };


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

    //writePcdHeader(f);

    //writeAxes(f);

    //writePointCloud(f, pointCloud_ab, 0x00FFFF);
    //writePointCloud(f, pointCloud_ac, 0xFFFFFF);
    //writePointCloud(f, pointCloud_bc, 0xFF00FF);
    //writePointCloud(f, pointCloud_ad, 0x000080);
    //writePointCloud(f, pointCloud_bd, 0x008000);
    //writePointCloud(f, pointCloud_cd, 0x800000);

    //markPointWithStar(f, lh, 0xFF0000);

    //drawLineBetweenPoints(f, a, b, 255);
    //drawLineBetweenPoints(f, b, c, 255);
    //drawLineBetweenPoints(f, a, c, 255);
    //drawLineBetweenPoints(f, a, d, 255);
    //drawLineBetweenPoints(f, b, d, 255);
    //drawLineBetweenPoints(f, c, d, 255);

    //Point *clouds[6];

    //clouds[0] = pointCloud_ab;
    //clouds[1] = pointCloud_ac;
    //clouds[2] = pointCloud_bc;
    //clouds[3] = pointCloud_ad;
    //clouds[4] = pointCloud_bd;
    //clouds[5] = pointCloud_cd;

    //Point bestMatchA = findBestPointMatch(pointCloud_ab, clouds, 6);

    //markPointWithStar(f, bestMatchA, 0x00FFFF);

    //updateHeader(f);


    //fclose(f);

    return 0;
}