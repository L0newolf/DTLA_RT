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

/*************************************************************************************************************************************/
static struct timeval t0;

static inline void tick(void) { gettimeofday(&t0, NULL); }

static inline float tock(void)
{
  struct timeval t1;
  gettimeofday(&t1, NULL);
  return (t1.tv_sec - t0.tv_sec) * 1000 + ((float)t1.tv_usec - t0.tv_usec) / 1000 ;
}

float tdiff = 0;
float tdiff1 = 0;
float tdiff2 = 0;

int runCount = 0;
int runCount1 = 0;
int runCount2 = 0;
/*************************************************************************************************************************************/

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

  /*************************************************************************************************************************************/
  float bufRead[CORES * WINDOWPERCORE * NUMANGLES];
  tdiff = 0.0;
  runCount = 0;
  for (int i = 0; i < 100; i++)
  {
    tick();
    e_read(pERAM, 0, 0, (off_t) 0, (void *)bufRead, CORES * WINDOWPERCORE * NUMANGLES * sizeof(float));
    e_read(pERAM, 0, 0, (off_t) 0, (void *)bufRead, CORES * WINDOWPERCORE * NUMANGLES * sizeof(float));
    tdiff += tock();
    runCount++;
  }
  printf("\n\n\nTime to read data to ARM from ERAM in millisecs : %f\n\n\n", tdiff / runCount);
  /*************************************************************************************************************************************/


  return 0;
}
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

int eDataWrite(float *dataReal, float *dataImag, float *bfoReal, float *bfoImag, float *sinValsAngles, float *cosValsAngles, float *sinValsSamples, float *cosValsSamples, int numSamples)
{

  int coreId, offset;

  unsigned int done[2];
  done[0] = 0x00000000;
  done[1] = 0x00000000;

  for (int r = 0; r < NROWS; r++)
  {
    for (int c = 0; c < NCOLS; c++)
    {

      coreId = r * NCOLS + c;
      offset = 0x2000;

      if (e_write(pEpiphany, r, c, offset, (const void *)&dataReal[WINDOWPERCORE * coreId], WINDOWPERCORE * sizeof(float)) == E_ERR) {
        cout << "ERROR : Failed to write data to device memory" << endl;
        return 1;
      }
      offset += WINDOWPERCORE * sizeof(float);

      if (e_write(pEpiphany, r, c, offset, (const void *)&dataImag[WINDOWPERCORE * coreId], WINDOWPERCORE * sizeof(float)) == E_ERR) {
        cout << "ERROR : Failed to write data to device memory" << endl;
        return 1;
      }
      offset += WINDOWPERCORE * sizeof(float);

      if (e_write(pEpiphany, r, c, offset, (const void *)sinValsAngles, NUMANGLES * sizeof(float)) == E_ERR) {
        cout << "ERROR : Failed to write data to device memory" << endl;
        return 1;
      }
      offset += NUMANGLES * sizeof(float);

      if (e_write(pEpiphany, r, c, offset, (const void *)cosValsAngles, NUMANGLES * sizeof(float)) == E_ERR) {
        cout << "ERROR : Failed to write data to device memory" << endl;
        return 1;
      }
      offset += NUMANGLES * sizeof(float);

      if (e_write(pEpiphany, r, c, offset, (const void *)sinValsSamples, WINDOWPERCORE * sizeof(float)) == E_ERR) {
        cout << "ERROR : Failed to write data to device memory" << endl;
        return 1;
      }
      offset += WINDOWPERCORE * sizeof(float);

      if (e_write(pEpiphany, r, c, offset, (const void *)cosValsSamples, WINDOWPERCORE * sizeof(float)) == E_ERR) {
        cout << "ERROR : Failed to write data to device memory" << endl;
        return 1;
      }
      offset += WINDOWPERCORE * sizeof(float);

      if (e_write(pEpiphany, r, c, offset, (const void *)&numSamples, sizeof(int)) == E_ERR) {
        cout << "ERROR : Failed to write data to device∏ memory" << endl;
        return 1;
      }


      if (e_write(pEpiphany, r, c, 0x7500, (const void *)&done[0], sizeof(done)) == E_ERR)
      {
        cout << "ERROR : Failed to write data to eCPU memory" << endl;
      }


      if (e_write(pEpiphany, r, c, 0x2500 , (const void *)&bfoReal[(r * NCOLS + c)*WINDOWPERCORE * NUMANGLES], WINDOWPERCORE * NUMANGLES * sizeof(float)) == E_ERR)
      {
        cout << "ERROR : Failed to write data to device∏ memory" << endl;
        return 1;
      }

      if (e_write(pEpiphany, r, c, 0x5000, (const void *)&bfoImag[(r * NCOLS + c)*WINDOWPERCORE * NUMANGLES], WINDOWPERCORE * NUMANGLES * sizeof(float)) == E_ERR)
      {
        cout << "ERROR : Failed to write data to device∏ memory" << endl;
        return 1;
      }





    }
  }
  return 0;
}

/*************************************************************************************************************************************/
void beamform(float *dataReal, float *dataImag, float *bfoReal, float *bfoImag, float *sinValsAngles, float *cosValsAngles, float *sinValsSamples, float *cosValsSamples, int numSamples)
{
  int winLen = WINDOWLEN;
  int numLoops = numSamples / winLen;
  int offset = 0;

  //unsigned int done[CORES], allDone = 0;

  //cout<<"Loops per channel : "<<numLoops<<endl;

  for (int i = 0; i < numLoops; i++)
  {

    //cout << "Processing data block : " << i << endl;

    /*
    for (int c = 0; c < CORES; c++)
      done[c] = 0;
    */

    offset = WINDOWLEN * i;


    // write to memeory
    tick();
    if (eDataWrite(&dataReal[offset], &dataImag[offset], &bfoReal[offset * NUMANGLES ], &bfoImag[offset * NUMANGLES], sinValsAngles, cosValsAngles, sinValsSamples, cosValsSamples, numSamples))
    {
      cout << "ERROR : Failed to write data to shared memory" << endl;
    }
    tdiff2 += tock();
    runCount2++;

    /*
    // Run program on cores
    for (int r = 0; r < NROWS; r++)
    {
        for (int c = 0; c < NCOLS; c++)
        {
            if (e_signal(pEpiphany, r, c) != E_OK)
            {
                std::cout << "ERROR : Failed to start program on epipahny core " << std::endl;
            }
        }
    }


    allDone = 0;


    while (1)
    {
        for (int r = 0; r < NROWS; r++)
        {
            for (int c = 0; c < NCOLS; c++)
            {
                if (!done[r * NCOLS + c])
                {
                    if (e_read(pEpiphany, r, c, 0x7500, &done[r * NCOLS + c], sizeof(int)) == E_ERR)
                    {
                        cout << "ERROR : Failed to read data from device memory" << endl;
                    }
                    else
                    {
                        //cout << "Status from core " << r * NCOLS + c << " : " << done[r * NCOLS + c] << endl;
                        allDone += done[r * NCOLS + c];
                    }
                }
            }
        }

        if (allDone == CORES)
        {
            allDone = 0;
            break;
        }
        else
        {
            //cout << "all Done : " << allDone << endl ;
            usleep(10);
        }
    }
    */

    /********************************************************************************************************************************/

    tick();
    if (e_read(pERAM, 0, 0, (off_t) 0x00000000, (void *)&bfoReal[offset * NUMANGLES], CORES * WINDOWPERCORE * NUMANGLES * sizeof(float)) == E_ERR)
    {
      cout << "ERROR : Failed to read data from shared memory" << endl;
    }

    if (e_read(pERAM, 0, 0, (off_t) 0x01000000, (void *)&bfoImag[offset * NUMANGLES], CORES * WINDOWPERCORE * NUMANGLES * sizeof(float)) == E_ERR)
    {
      cout << "ERROR : Failed to read data from shared memory" << endl;
    }

    tdiff1 += tock();
    runCount1++;

    /********************************************************************************************************************************/



  }



  //cout << "Time to copy outputs : " << (timeKeep / runCount)*numLoops << endl << endl;
}

/*************************************************************************************************************************************/

void rtPingerDet::detectPingerPos(float *data, int numSamples, float *firCoeff, float *bfoFinalReal, float *bfoFinalImag, fftwf_complex *analyticData)
{
  //Objects
  filtSignal filt;
  findSigFreq freqDetector;
  hilbertTrans hib;

  //Working varialbes
  float curData[numSamples];

  //


  int freqBF = 0.0;
  int freqDet = 0;
  float maxPow = 0.0;
  int freqIdx = 0;
  float filtData[numSamples];

  for (int j = 0; j < numSamples * NUMANGLES; j++)
  {
    bfoFinalImag[j] = 1.0;
    bfoFinalReal[j] = 1.0;
  }



  //std::fstream bfoFile("bfo_opt_cpp.txt", std::ios_base::out);

  //BandPass the signal and find the frequency to be used for beamforming

  for (int i = 0; i < NUMCHANNELS; i++)
  {

    cout << "Processing data from channel : " << i << endl;
    for (int j = 0; j < numSamples; j++)
      curData[j] = data[i + j * NUMCHANNELS];


    filt.filter(filtData, firCoeff, numTaps + 1, curData, numSamples);


    if (i == 0)
    {

      std::tie(freqDet, maxPow) = freqDetector.detectSigFreq(filtData, Fs, nFFT, numSamples, numOverlap);

      numCalls++;

      if (numCalls == 1)
      {
        freqBF = freqDet;
        lastFreqUsed = freqBF;
      }
      else
      {
        freqBF = (1 - lambda) * lastFreqUsed + lambda * freqDet;
        lastFreqUsed = freqBF;
      }

      freqIdx = (float)((float)freqBF / Fs) * nFFT;
      cout << "Freq: " << freqDet << " Pow : " << maxPow << " Call: " << numCalls << " Freq BF : " << freqBF << " freq index : " << freqIdx << endl;
    }


    /// Convert the incoming signal to its complex baseband form

    int samplesToUse = (int)(numSamples / SKIP_RATE);
    float filtDataToUse[samplesToUse];
    int counter = 0;
    for (int j = 0; j < numSamples; j++)
    {
      if (SKIP_RATE == 1)
      {
        filtDataToUse[j] = filtData[j];
      }
      else
      {
        if (j % SKIP_RATE)
        {
          filtDataToUse[counter] = filtData[j];
          counter++;
        }
      }
    }

    cout << "Creating analytic signal" << endl;

    hib.sigAnalytic(filtDataToUse, analyticData, samplesToUse);

    cout << "Created analytic signal" << endl;

    float dataReal[samplesToUse];
    float dataImag[samplesToUse];

    for (int i = 0; i < samplesToUse; i++)
    {
      dataReal[i] = analyticData[i][0];
      dataImag[i] = analyticData[i][1];
    }

    //Beamform the resultant data
    beamform(dataReal, dataImag, bfoFinalReal, bfoFinalImag, &sinValsAngles[freqIdx][i][0], &cosValsAngles[freqIdx][i][0], &sinValsSamples[freqIdx][i][0], &cosValsSamples[freqIdx][i][0], samplesToUse);

  }

  /*
  float maxVal = 0.0;
  int angleIdx = 0;
  int timeIdx = 0;
  float tempVal = 0.0;

  for (int a = 0; a < Fs; a++) {
      for (int b = 0; b < NUMANGLES; b++) {
          tempVal = 20 * log10(sqrt((bfoFinalImag[NUMANGLES * a + b] * bfoFinalImag[NUMANGLES * a + b]) + (bfoFinalReal[NUMANGLES * a + b] * bfoFinalReal[NUMANGLES * a + b])));
          //bfoFile << tempVal << endl;
          if (tempVal > maxVal) {
              maxVal = tempVal;
              angleIdx = b;
              timeIdx = a;
          }
      }
  }

  cout << "Max power detected at : " << rad2deg(angles[angleIdx]) << " degrees at time : " << numCalls + ((float)(SKIP_RATE*timeIdx) / Fs) << " secs for frequency  : " << freqBF << endl;

  //bfoFile.close();
  */

}
/*************************************************************************************************************************************/


int main(int argc, char *argv[])
{
  /*
  int a = 24038 * 1; //2163461;
  float durPerBlock = 1.0;
  int samplesPerBlock = floor(Fs * durPerBlock);
  int numLoops = floor(a / samplesPerBlock);

  float curSig[NUMCHANNELS * samplesPerBlock];

  float *bfoFinalReal = new float[samplesPerBlock * NUMANGLES];
  float *bfoFinalImag = new float[samplesPerBlock * NUMANGLES];
  fftwf_complex *analyticData = new fftwf_complex[samplesPerBlock];

  rtPingerDet detectPinger;
  genFilt filter;

  //create the bandpass filter coeffs
  float firCoeff[numTaps + 1];
  float omega, bandwidth;
  omega = (float)(freqUpper + freqLower) / Fs;
  bandwidth = 2 * (float)(freqUpper - freqLower) / Fs;
  filter.BasicFIR(firCoeff, numTaps + 1, BPF, omega, bandwidth, wtKAISER_BESSEL, beta);

  initBFCoeffs();
  */

  if (eInit())
  {
    cout << "Cannot initialize the epiphany cluster.... \n";
  }

  /*
  std::fstream sigFile("sig.txt", std::ios_base::in);

  for (int i = 0; i < numLoops; i++)
  {

    //cout << "Processing time stamp : " << i << " secs " << endl;

    for (int j = 0; j < NUMCHANNELS * samplesPerBlock; j++)
      sigFile >> curSig[j];

   detectPinger.detectPingerPos(curSig, samplesPerBlock, firCoeff, bfoFinalReal, bfoFinalImag, analyticData);
  }

  sigFile.close();

  printf("Time to write data to ERAM in millisecs Avg per call : %f Total time : %f  Num of call : %d \n", tdiff2 / runCount2, tdiff2, runCount2);
  printf("Time to read data from ERAM in millisecs Avg per call : %f Total time : %f  Num of call : %d \n", tdiff1 / runCount1, tdiff1, runCount1);
  */

  if (eFree())
  {
    cout << "Cannot finalize the epiphany cluster.... \n";
  }

  /*
  delete(bfoFinalReal);
  delete(bfoFinalImag);
  fftwf_free(analyticData);
  */

  return (EXIT_SUCCESS);
}



