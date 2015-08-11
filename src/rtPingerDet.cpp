//Standard headers
#include <math.h>
#include <stddef.h>
#include <iostream>
#include <cstring>
#include <tuple>
#include <fstream>
#include <sstream>
#include <unistd.h>
#include <sys/time.h>

//Epiphany headers
#include <e-hal.h>
#include <e-loader.h>

//fftwf library
#include <fftw3.h>

//beamforming headers
#include "genFilt.hpp"
#include "filtSignal.hpp"
#include "findSigFreq.hpp"
#include "settings.h"
#include "rtPingerDet.hpp"
#include "deg2rad.hpp"
#include "hilbertTrans.hpp"

using namespace std;

float sinValsAngles[nFFT][NUMCHANNELS][NUMANGLES];
float cosValsAngles[nFFT][NUMCHANNELS][NUMANGLES];
float sinValsSamples[nFFT][NUMCHANNELS][WINDOWPERCORE];
float cosValsSamples[nFFT][NUMCHANNELS][WINDOWPERCORE];
float angles[NUMANGLES];
float sinAngles[NUMANGLES];

struct timespec timer[2];
float tdiff = 0;
int runCount = 0;

e_epiphany_t Epiphany, *pEpiphany;
e_mem_t      ERAM,     *pERAM;
extern e_platform_t e_platform;


/*************************************************************************************************************************************/

rtPingerDet::rtPingerDet()
{
    numCalls = 0;
    lastFreqUsed = 0;
}
/*************************************************************************************************************************************/


float rad2deg(float angle)
{
    return angle * 180 / M_PI;
}

/*************************************************************************************************************************************/

void initBFCoeffs()
{
    float step = Fs / nFFT;
    float temp, temp1;

    deg2rad(angles, sinAngles, NUMANGLES);

    temp = 2 * pi * step;

    for (int i = 0; i < nFFT; i++)
    {
        for (int j = 0; j < NUMCHANNELS; j++)
        {

            for (int k = 0; k < NUMANGLES; k++)
            {
                temp1 = i * temp * SPACING * ((float)j / SPEED) * sinAngles[k];
                sinValsAngles[i][j][k] = sin(temp1);
                cosValsAngles[i][j][k] = cos(temp1);
            }

            for (int k = 0; k < WINDOWPERCORE; k++)
            {
                temp1 = -i * temp * ((((float)k + 1) / Fs) + SLEW * j);
                sinValsSamples[i][j][k] = sin(temp1);
                cosValsSamples[i][j][k] = cos(temp1);
            }

        }
    }
}
/*************************************************************************************************************************************/


/*************************************************************************************************************************************/

int eInit(void)
{
    pEpiphany = &Epiphany;
  pERAM     = &ERAM;

  if ( E_OK != e_init(NULL) ) {
    printf("\nERROR: epiphinay initialization failed!\n\n");
    exit(1);
  }

  if (E_OK != e_reset_system() ) {
    printf("\nWARNING: epiphinay system rest failed!\n\n");
  }

  // prepare ERAM
  if (E_OK != e_alloc(pERAM, 0x00000000, e_platform.emem[0].size))
  {
    printf("\nERROR: Can't allocate Epiphany DRAM!\n\n");
    exit(1);
  }

  if (E_OK != e_open(pEpiphany, 0, 0, e_platform.rows, e_platform.cols))
  {
    printf("\nERROR: Can't establish connection to Epiphany device!\n\n");
    exit(1);
  }

  int loops = 100;
  int num = CORES * NUMANGLES;
  float bufRead[loops * num];
  
  int i;
  for (i = 0; i < loops; i++)
  {
    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &timer[0]);
    e_read(pERAM, 0, 0, (off_t) 0, (void *)&bufRead[num * i], CORES * NUMANGLES * sizeof(float));
    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &timer[1]);
    tdiff += ((float) (timer[1].tv_sec - timer[0].tv_sec)) + ((float) (timer[1].tv_nsec - timer[0].tv_nsec) / 1000.0);
    runCount++;
  }
  printf("Time to read data to ARM from ERAM in microsecs : %f\n", tdiff / runCount);

  tdiff = 0;
  runCount = 0;
  for (i = 0; i < loops; i++)
  {
    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &timer[0]);
    e_read(pERAM, 0, 0, (off_t) 0x01000000, (void *)&bufRead[num * i], CORES * NUMANGLES * sizeof(float));
    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &timer[1]);
    tdiff += ((float) (timer[1].tv_sec - timer[0].tv_sec)) + ((float) (timer[1].tv_nsec - timer[0].tv_nsec) / 1000.0);
    runCount++;
  }
  printf("Time to read data to ARM from ERAM in microsecs : %f\n", tdiff / runCount);


    return 0;
}
/*************************************************************************************************************************************/

/*************************************************************************************************************************************/

int eFree(void)
{
    e_free(pERAM);

    // Close connection to device
    if (e_close(pEpiphany))
    {
        printf("\nERROR: Can't close connection to Epiphany device!\n\n");
        return 1;
    }

    e_finalize();

    return 0;
}
/*************************************************************************************************************************************/


int main(int argc, char *argv[])
{
  initBFCoeffs();

    if (eInit())
    {
        cout << "Cannot initialize the epiphany cluster.... \n";
    }

    if (eFree()) {
        cout << "Cannot finalize the epiphany cluster.... \n";
    }

  return (EXIT_SUCCESS);
}



