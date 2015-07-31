#include <math.h>
#include <stddef.h>
#include <iostream>

void deg2rad(float *angles,float *sinAngles, int numAngles)
{
    int step = 180/numAngles;

    for(int i=0;i<numAngles;i++)
    {
        angles[i] = ((float)(-90+(i*step))*M_PI)/180;
        sinAngles[i] = sin(angles[i]);
    }
}
