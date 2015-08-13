//Standar headers
#include <math.h>
#include <stddef.h>
#include <iostream>
#include <cstring>
#include <tuple>
#include <iomanip>

#include <fstream>
#include <sstream>


#include <unistd.h>
#include <sys/time.h>

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

//epiphany specific headers
#include <e-hal.h>
#include <e-loader.h>
#include <a_trace.h>


using namespace std;

e_epiphany_t dev;
e_mem_t      bfoRealERAM,     *pbfoRealERAM;
e_mem_t      bfoImagERAM,     *pbfoImagERAM;
extern e_platform_t e_platform;

float angles[NUMANGLES];
float sinAngles[NUMANGLES];
float sinValsAngles[nFFT][NUMCHANNELS][NUMANGLES];
float cosValsAngles[nFFT][NUMCHANNELS][NUMANGLES];
float sinValsSamples[nFFT][NUMCHANNELS][WINDOWPERCORE];
float cosValsSamples[nFFT][NUMCHANNELS][WINDOWPERCORE];

/*************************************************************************************************************************************/

float timeKeep = 0.0;
float runCount = 0;
float timeKeep1 = 0.0;
float runCount1 = 0;
float timeKeep2 = 0.0;
float runCount2 = 0;
float timeKeep3 = 0.0;
float runCount3 = 0;
float timeKeep4 = 0.0;
float runCount4 = 0;
float timeKeep5 = 0.0;
float runCount5 = 0;
float timeKeep6 = 0.0;
float runCount6 = 0;
float timeKeep7 = 0.0;
float runCount7 = 0;
float timeKeep8 = 0.0;
float runCount8 = 0;

static struct timeval t0;
static struct timeval t2, t3, t4, t5;

static inline void tick(void) { gettimeofday(&t0, NULL); }

static inline float tock(void) {
    struct timeval t1;
    gettimeofday(&t1, NULL);
    return (t1.tv_sec - t0.tv_sec) * 1000 + ((float)t1.tv_usec - t0.tv_usec) / 1000 ;
}

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
    pbfoRealERAM     = &bfoRealERAM;
    pbfoImagERAM     = &bfoImagERAM;

    if (e_init(NULL) != E_OK)
    {
        printf("\nERROR: Can't initialize Epiphany device!\n\n");
        return 1;
    }

    if (E_OK != e_reset_system() )
    {
        fprintf(stderr, "\nWARNING: epiphinay system rest failed!\n\n");
    }

    if (E_OK != e_alloc(pbfoRealERAM, 0x00000000, Fs * NUMANGLES * sizeof(float)))
    {
        fprintf(stderr, "\nERROR: Can't allocate Epiphany DRAM!\n\n");
        return 1;
    }

    if (E_OK != e_alloc(pbfoImagERAM, 0x00010000, Fs * NUMANGLES * sizeof(float)))
    {
        fprintf(stderr, "\nERROR: Can't allocate Epiphany DRAM!\n\n");
        return 1;
    }

    if (e_open(&dev, 0, 0, NROWS, NCOLS)) {
        printf("\nERROR: Can't establish connection to Epiphany device!\n\n");
        return 1;
    }

    if (e_load_group("bin/eBF.srec", &dev, 0, 0, NROWS, NCOLS, E_TRUE) != E_OK)
    {
        printf("\nERROR: Can't load program file to core!\n\n");
        return 1;
    }

    return 0;
}
/*************************************************************************************************************************************/


/*************************************************************************************************************************************/

int eFree(void)
{
    e_free(pbfoRealERAM);
    e_free(pbfoImagERAM);

    // Close connection to device
    if (e_close(&dev))
    {
        printf("\nERROR: Can't close connection to Epiphany device!\n\n");
        return 1;
    }

    e_finalize();

    return 0;
}
/*************************************************************************************************************************************/

void beamFormer(float *bfInReal,float *bfInImag,float *bfOutReal,float *bfOutImag, int numSamples, float freqBF)
{
    float temp_real;
    float temp_imag;

    float temp_real_1;
    float temp_imag_1;

    float temp_real_2;
    float temp_imag_2;

    for(int k=0;k<numSamples;k++)
    {
        for(int j=0;j<NUMCHANNELS;j++)
        {
            temp_real = cos(-2*pi*freqBF*( k/numSamples + SLEW*j));
            temp_imag = sin(-2*pi*freqBF*( k/numSamples + SLEW*j));

            temp_real_1 = bfInReal[NUMCHANNELS*j+k]*temp_real - bfInImag[NUMCHANNELS*j+k]*temp_imag;
            temp_imag_1 = bfInReal[NUMCHANNELS*j+k]*temp_imag + bfInImag[NUMCHANNELS*j+k]*temp_real;

            for(int a=0;a<NUMANGLES;a++)
            {
                temp_real_2 = cos(2*pi*freqBF*SPACING*j/SPEED*sin(angles[a]));
                temp_imag_2 = sin(2*pi*freqBF*SPACING*j/SPEED*sin(angles[a]));

                temp_real = temp_real_1*temp_real_2 + temp_imag_1*temp_imag_2;
                temp_imag = temp_real_2*temp_imag_1 - temp_imag_2*temp_real_1;

                bfOutReal[NUMANGLES*k+a] = bfOutReal[NUMANGLES*k+a] + temp_real;
                bfOutImag[NUMANGLES*k+a] = bfOutImag[NUMANGLES*k+a] + temp_imag;

            }
        }
    }
}

/*************************************************************************************************************************************/

void rtPingerDet::detectPingerPos(float *data, int numSamples, float *firCoeff, fftwf_complex *analyticData)
{

    //Objects
    filtSignal filt;
    findSigFreq freqDetector;
    hilbertTrans hib;

    //Working varialbes
    int samplesToUse = (int)(numSamples / SKIP_RATE);
    float curData[NUMCHANNELS][numSamples];

    cout<<"Got here "<<endl;
    float bfOutReal[samplesToUse * NUMANGLES]; 
    float bfOutImag[samplesToUse * NUMANGLES];

    float bfInReal[samplesToUse * NUMANGLES]; 
    float bfInImag[samplesToUse * NUMANGLES];

    cout<<"Got here1 "<<endl;    
    //


    int freqBF = 0.0;
    int freqDet = 0;
    float maxPow = 0.0;
    int freqIdx = 0;
    float filtData[numSamples];

    

    for (int j = 0; j < samplesToUse * NUMANGLES; j++)
    {
        bfOut[j] = float(1,1);
    }



    //std::fstream bfoFile("bfo_opt_cpp.txt", std::ios_base::out);

    //BandPass the signal and find the frequency to be used for beamforming

    for (int i = 0; i < NUMCHANNELS; i++)
    {

        cout << "Processing data from channel : " << i << endl;
        for (int j = 0; j < numSamples; j++)
            curData[i][j] = data[i + j * NUMCHANNELS];

        tick();
        filt.filter(filtData, firCoeff, numTaps + 1, &curData[i][0], numSamples);
        timeKeep1 += tock();
        runCount1++;


        if (i == 0)
        {
            tick();
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

            timeKeep2 += tock();
            runCount2++;
        }


        /// Convert the incoming signal to its complex baseband form


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

        tick();
        hib.sigAnalytic(filtDataToUse, analyticData, samplesToUse);
        timeKeep3 += tock();
        runCount3++;

        for (int k = 0; k < samplesToUse; k++)
        {
            bfInReal[NUMCHANNELS*k + i] = (float)analyticData[k][0];
            bfInImag[NUMCHANNELS*k + i] = (float)analyticData[k][1]);
        }

    }

    

    //Beamform the resultant data
    /* CALL BF FUNCTION HERE */
    beamFormer(bfInReal,bfInImag,bfOutReal,bfOutImag,samplesToUse,freqBF);

    float maxVal = 0.0;
    int angleIdx = 0;
    int timeIdx = 0;
    float tempVal = 0.0;

    tick();
    for (int a = 0; a < samplesToUse; a++)
    {
        for (int b = 0; b < NUMANGLES; b++)
        {
            tempVal = 20 * log10(sqrt(bfOutReal[NUMANGLES * a + b]*bfOutReal[NUMANGLES * a + b] + bfOutImag[NUMANGLES * a + b]*bfOutImag[NUMANGLES * a + b]));
            //bfoFile << tempVal << endl;
            if (tempVal > maxVal)
            {
                maxVal = tempVal;
                angleIdx = b;
                timeIdx = a;
            }
        }
    }
    timeKeep8 += tock();
    runCount8++;

    cout << "Max power detected at : " << rad2deg(angles[angleIdx]) << " degrees at time : " << numCalls + ((float)(SKIP_RATE * timeIdx) / Fs) << " secs for frequency  : " << freqBF << endl;

    //bfoFile.close();


}
/*************************************************************************************************************************************/

/*************************************************************************************************************************************/

int main()
{
    /* READ SIGNAL VALUES*/

    int a = 24038 * 1; //2163461;
    float durPerBlock = 1.0;
    int samplesPerBlock = floor(Fs * durPerBlock);
    int numLoops = floor(a / samplesPerBlock);

    float curSig[NUMCHANNELS * samplesPerBlock];

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

    if (eInit())
    {
        cout << "Cannot initialize the epiphany cluster.... \n";
    }

    

    std::fstream sigFile("sig.txt", std::ios_base::in);


    for (int i = 0; i < numLoops; i++)
    {

        cout << "Processing time stamp : " << i << " secs " << endl;

        for (int j = 0; j < NUMCHANNELS * samplesPerBlock; j++)
            sigFile >> curSig[j];
        
        gettimeofday(&t2, NULL);
        detectPinger.detectPingerPos(curSig, samplesPerBlock, firCoeff, analyticData);
        gettimeofday(&t3, NULL);
        timeKeep += (float)((t3.tv_sec - t2.tv_sec) * 1000 + ((float)t3.tv_usec - t2.tv_usec) / 1000);
        runCount++;
    }

    /*************************************************************************************************************************************/
    cout << endl;
    cout << "Time to process single second data block              : " << (timeKeep / runCount)   << "   num of cycles : " << runCount <<  "   Total time : " << timeKeep  << endl;
    cout << "Time to process filter single channel data            : " << (timeKeep1 / runCount1) << "   num of cycles : " << runCount1 << "   Total time : " << timeKeep1 << endl;
    cout << "Time to find beamform freq                            : " << (timeKeep2 / runCount2) << "   num of cycles : " << runCount2 << "   Total time : " << timeKeep2 << endl;
    cout << "Time to process hilbert transform single channel data : " << (timeKeep3 / runCount3) << "   num of cycles : " << runCount3 << "   Total time : " << timeKeep3 << endl;
    cout << "Time to process beamform single channel data          : " << (timeKeep4 / runCount4) << "   num of cycles : " << runCount4 << "   Total time : " << timeKeep4 << endl;
    cout << "Time to find max power and time and angle             : " << (timeKeep8 / runCount8) << "   num of cycles : " << runCount8 << "   Total time : " << timeKeep8 << endl;
    cout << "Time to copy single data block to eCPU                : " << (timeKeep5 / runCount5) << "   num of cycles : " << runCount5 << "   Total time : " << timeKeep5 << endl;
    cout << "Time to process single data block in eCPU             : " << (timeKeep6 / runCount6) << "   num of cycles : " << runCount6 << "   Total time : " << timeKeep6 << endl;
    cout << "Time to copy single data block from shared mem        : " << (timeKeep7 / runCount7) << "   num of cycles : " << runCount7 << "   Total time : " << timeKeep7 << endl;
    cout << endl;
    /*************************************************************************************************************************************/

    fftwf_free(analyticData);

    if (eFree()) {
        cout << "Cannot finalize the epiphany cluster.... \n";
    }

    sigFile.close();

    return 0;
}
/*************************************************************************************************************************************/