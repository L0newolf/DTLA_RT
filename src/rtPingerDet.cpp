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
e_mem_t      filtERAM,     *pfiltERAM;
extern e_platform_t e_platform;

float angles[NUMANGLES];
float sinAngles[NUMANGLES];

float sinValsAngles[nFFT][NUMANGLES * NUMCHANNELS];
float cosValsAngles[nFFT][NUMANGLES * NUMCHANNELS];
float sinValsChannels[nFFT][WINDOWPERCORE * NUMCHANNELS];
float cosValsChannels[nFFT][WINDOWPERCORE * NUMCHANNELS];

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
float timeKeep9 = 0.0;
float runCount9 = 0;
float timeKeep10 = 0.0;
float runCount10 = 0;
float timeKeep11 = 0.0;
float runCount11 = 0;

static struct timeval t0;
static struct timeval t2, t3, t4, t5, t6, t7;

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

    int i, k, j;

    for ( i = 0; i < nFFT; i++)
    {


        for ( j = 0; j < NUMCHANNELS; j++)
        {
            for ( k = 0; k < NUMANGLES; k++)
            {
                temp1 = i * temp * SPACING * j / SPEED * sin(angles[k]);
                sinValsAngles[i][NUMANGLES * j + k] = sin(temp1);
                cosValsAngles[i][NUMANGLES * j + k] = cos(temp1);
            }
        }

        for ( k = 0; k < WINDOWPERCORE; k++)
        {
            for ( j = 0; j < NUMCHANNELS; j++)
            {
                temp1 = -i * temp * (k / WINDOWPERCORE + SLEW * j);
                sinValsChannels[i][NUMCHANNELS * k + j] = sin(temp1);
                cosValsChannels[i][NUMCHANNELS * k + j] = cos(temp1);
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
    pfiltERAM        = &filtERAM;

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

    if (E_OK != e_alloc(pbfoImagERAM, 0x00100000, Fs * NUMANGLES * sizeof(float)))
    {
        fprintf(stderr, "\nERROR: Can't allocate Epiphany DRAM!\n\n");
        return 1;
    }

    if (E_OK != e_alloc(pfiltERAM, 0x00200000, Fs * sizeof(float)))
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
    e_free(pfiltERAM);

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

int writeDataToEpipahnyBF(float *bfInReal, float *bfInImag, int freqBF, int loopCount)
{
    int coreId, offset;

    unsigned int done = PROCESS_BEAMFORM;

    for (int r = 0; r < NROWS; r++)
    {
        for (int c = 0; c < NCOLS; c++)
        {

            coreId = r * NCOLS + c;
            offset = 0x2000;

            if (e_write(&dev, r, c, offset, (const void *)&bfInReal[WINDOWPERCORE * NUMCHANNELS * coreId + CORES * WINDOWPERCORE * NUMCHANNELS * loopCount], WINDOWPERCORE * NUMCHANNELS * sizeof(float)) == E_ERR) {
                cout << "ERROR : Failed to write data to device memory" << endl;
                return 1;
            }
            offset += WINDOWPERCORE * NUMCHANNELS * sizeof(float);


            if (e_write(&dev, r, c, offset, (const void *)&bfInImag[WINDOWPERCORE * NUMCHANNELS * coreId + CORES * WINDOWPERCORE * NUMCHANNELS * loopCount], WINDOWPERCORE * NUMCHANNELS * sizeof(float)) == E_ERR) {
                cout << "ERROR : Failed to write data to device memory" << endl;
                return 1;
            }
            offset += WINDOWPERCORE * NUMCHANNELS * sizeof(float);

            if (e_write(&dev, r, c, offset, (const void *)angles, NUMANGLES * sizeof(float)) == E_ERR) {
                cout << "ERROR : Failed to write data to device memory" << endl;
                return 1;
            }
            offset += NUMANGLES * sizeof(float);

            if (e_write(&dev, r, c, offset, (const void *)&loopCount, sizeof(int)) == E_ERR) {
                cout << "ERROR : Failed to write data to device memory" << endl;
                return 1;
            }
            offset += sizeof(int);



            if (loopCount == 0)
            {
                offset = 0x2500;

                if (e_write(&dev, r, c, offset, (const void *)&sinValsChannels[freqBF][0], NUMCHANNELS * WINDOWPERCORE * sizeof(float)) == E_ERR) {
                    cout << "ERROR : Failed to write data to device memory" << endl;
                    return 1;
                }
                offset += NUMCHANNELS * WINDOWPERCORE * sizeof(float);

                if (e_write(&dev, r, c, offset, (const void *)&cosValsChannels[freqBF][0], NUMCHANNELS * WINDOWPERCORE * sizeof(float)) == E_ERR) {
                    cout << "ERROR : Failed to write data to device memory" << endl;
                    return 1;
                }
                offset += NUMCHANNELS * WINDOWPERCORE * sizeof(float);

                if (e_write(&dev, r, c, offset, (const void *)&sinValsAngles[freqBF][0], NUMCHANNELS * NUMANGLES * sizeof(float)) == E_ERR) {
                    cout << "ERROR : Failed to write data to device memory" << endl;
                    return 1;
                }
                offset += NUMCHANNELS * NUMANGLES * sizeof(float);

                if (e_write(&dev, r, c, offset, (const void *)&cosValsAngles[freqBF][0], NUMCHANNELS * NUMANGLES * sizeof(float)) == E_ERR) {
                    cout << "ERROR : Failed to write data to device memory" << endl;
                    return 1;
                }
                offset += NUMCHANNELS * NUMANGLES * sizeof(float);
            }



            if (e_write(&dev, r, c, 0x7500, (const void *)&done, sizeof(done)) == E_ERR)
            {
                cout << "ERROR : Failed to write data to eCPU memory" << endl;
                return 1;
            }


        }
    }

    return 0;
}

/*************************************************************************************************************************************/

void beamFormer(float *bfInReal, float *bfInImag, float *bfOutReal, float *bfOutImag, int numSamples, int freqBF)
{
    int winLen = (int)CORES * (int) WINDOWPERCORE;
    int numLoops = numSamples / winLen;
    unsigned int done[CORES], allDone = 0;

    for (int i = 0; i < numLoops; i++)
    {

        //cout << "Processing data block : " << i << endl;

        for (int c = 0; c < CORES; c++)
            done[c] = 0;

        // write to memeory
        tick();
        if (writeDataToEpipahnyBF(bfInReal, bfInImag, freqBF, i))
        {
            cout << "ERROR : Failed to write data to shared memory" << endl;
        }
        timeKeep5 += tock();
        runCount5++;

        // Run program on cores
        for (int r = 0; r < NROWS; r++)
        {
            for (int c = 0; c < NCOLS; c++)
            {
                if (e_signal(&dev, r, c) != E_OK)
                {
                    std::cout << "ERROR : Failed to start program on epipahny core " << std::endl;
                }
            }
        }


        allDone = 0;


        tick();
        while (1)
        {
            for (int r = 0; r < NROWS; r++)
            {
                for (int c = 0; c < NCOLS; c++)
                {
                    if (!done[r * NCOLS + c])
                    {
                        if (e_read(&dev, r, c, 0x7500, &done[r * NCOLS + c], sizeof(int)) == E_ERR)
                        {
                            cout << "ERROR : Failed to read data from device memory" << endl;
                        }
                        else
                        {
                            if (done[r * NCOLS + c] == PROCESS_COMPLETE)
                            {
                                allDone += done[r * NCOLS + c];
                                //cout << "Status from core " << r * NCOLS + c << " : " << done[r * NCOLS + c] << endl;
                                //cout << "all Done : " << allDone << endl ;
                            }
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
        timeKeep6 += tock();
        runCount6++;

        tick();
        e_read(pbfoRealERAM , 0, 0, (off_t) 0,  (void *)&bfOutReal[CORES * WINDOWPERCORE * NUMANGLES * i], CORES * WINDOWPERCORE * NUMANGLES * sizeof(float));
        timeKeep7 += tock();
        runCount7++;
    }
}

/*************************************************************************************************************************************/

int writeDataToEpipahnyFILTER(float *inSig, float *filtSig, float *filtCoeff)
{
    int coreId, offset;

    unsigned int done = PROCESS_FILTER;

    for (int r = 0; r < NROWS; r++)
    {
        for (int c = 0; c < NCOLS; c++)
        {

            coreId = r * NCOLS + c;
            offset = 0x2500;

            if (e_write(&dev, r, c, offset, (const void *)&inSig[FILTERWINDOW * coreId], FILTERWINDOW * sizeof(float)) == E_ERR)
            {
                cout << "ERROR : Failed to write data to device memory" << endl;
                return 1;
            }

            offset = 0x7000;

            if (e_write(&dev, r, c, offset, (const void *)filtCoeff, (numTaps + 1) * sizeof(float)) == E_ERR)
            {
                cout << "ERROR : Failed to write data to device memory" << endl;
                return 1;
            }

            if (e_write(&dev, r, c, 0x7500, (const void *)&done, sizeof(done)) == E_ERR)
            {
                cout << "ERROR : Failed to write data to eCPU memory" << endl;
                return 1;
            }


        }
    }

    return 0;
}

/*************************************************************************************************************************************/

void filterSignal(float *inSig, float *filtSig, float *filtCoeff, int numSamples)
{
    unsigned int done[CORES], allDone = 0;

    //cout<<"Starting data write ... "<<endl;
    tick();
    if (writeDataToEpipahnyFILTER(inSig, filtSig, filtCoeff))
    {
        cout << "ERROR : Failed to write data to shared memory" << endl;
    }
    timeKeep9 += tock();
    runCount9++;
    //cout<<"Finished data write ... "<<endl;

    //cout<<"Signalling cores ... "<<endl;
    // Run program on cores
    for (int r = 0; r < NROWS; r++)
    {
        for (int c = 0; c < NCOLS; c++)
        {
            if (e_signal(&dev, r, c) != E_OK)
            {
                std::cout << "ERROR : Failed to start program on epipahny core " << std::endl;
            }
        }
    }

    //cout<<"Waiting on cores ... "<<endl;
    allDone = 0;

    tick();
    while (1)
    {
        for (int r = 0; r < NROWS; r++)
        {
            for (int c = 0; c < NCOLS; c++)
            {
                if (done[r * NCOLS + c] != PROCESS_COMPLETE)
                {
                    if (e_read(&dev, r, c, 0x7500, &done[r * NCOLS + c], sizeof(int)) == E_ERR)
                    {
                        cout << "ERROR : Failed to read data from device memory" << endl;
                    }
                    else
                    {
                        if (done[r * NCOLS + c] == PROCESS_COMPLETE)
                        {
                            allDone += done[r * NCOLS + c];
                            //cout << "Status from core " << r * NCOLS + c << " : " << done[r * NCOLS + c] << endl;
                            //cout << "all Done : " << allDone << endl ;
                        }
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
    timeKeep10 += tock();
    runCount10++;

    tick();
    e_read(pfiltERAM , 0, 0, (off_t) 0,  (void *)filtSig, FILTERWINDOW * CORES * sizeof(float));
    timeKeep11 += tock();
    runCount11++;

}
/*************************************************************************************************************************************/

void rtPingerDet::detectPingerPos(float *data, int numSamples, float *firCoeff, fftwf_complex *analyticData)
{


    //Objects
    //filtSignal filt;
    findSigFreq freqDetector;
    hilbertTrans hib;

    //Working varialbes
    int samplesToUse = (int)(numSamples / SKIP_RATE);
    float curData[NUMCHANNELS][numSamples];



    float *bfOutReal = new float [samplesToUse * NUMANGLES];
    float *bfOutImag = new float [samplesToUse * NUMANGLES];

    float *bfInReal = new float [samplesToUse * NUMANGLES];
    float *bfInImag = new float [samplesToUse * NUMANGLES];
    //



    int freqBF = 0.0;
    int freqDet = 0;
    float maxPow = 0.0;
    int freqIdx = 0;
    float *filtData = new float [numSamples];

    //cout<<"Starting beamforming ... "<<endl;

    for (int j = 0; j < samplesToUse * NUMANGLES; j++)
    {
        bfOutReal[j] = 1;
        bfOutImag[j] = 1;
    }

    //BandPass the signal and find the frequency to be used for beamforming

    for (int i = 0; i < NUMCHANNELS; i++)
    {

        //cout << "Processing data from channel : " << i << endl;
        for (int j = 0; j < numSamples; j++)
            curData[i][j] = data[i + j * NUMCHANNELS];

        //cout<<"Starting filtering process .... "<<endl;
        gettimeofday(&t6, NULL);
        //filt.filter(filtData, firCoeff, numTaps + 1, &curData[i][0], numSamples);
        filterSignal(&curData[i][0], filtData, firCoeff, numSamples);
        gettimeofday(&t7, NULL);
        timeKeep1 += (float)((t7.tv_sec - t6.tv_sec) * 1000 + ((float)t7.tv_usec - t6.tv_usec) / 1000);
        runCount1++;
        //cout<<"Finished filtering process .... "<<endl;


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

        samplesToUse = 8192;

        tick();
        hib.sigAnalytic(filtDataToUse, analyticData, samplesToUse);
        timeKeep3 += tock();
        runCount3++;

        for (int k = 0; k < samplesToUse; k++)
        {
            bfInReal[NUMCHANNELS * k + i] = (float)analyticData[k][0];
            bfInImag[NUMCHANNELS * k + i] = (float)analyticData[k][1];
        }

    }



    //Beamform the resultant data
    gettimeofday(&t4, NULL);
    beamFormer(bfInReal, bfInImag, bfOutReal, bfOutImag, samplesToUse, freqIdx);
    gettimeofday(&t5, NULL);
    timeKeep4 += (float)((t5.tv_sec - t4.tv_sec) * 1000 + ((float)t5.tv_usec - t4.tv_usec) / 1000);
    runCount4++;



    float maxVal = 0.0;
    int timeIdx = 0;
    float tempVal = 0.0;

    //std::fstream bfoFile("bfo_opt_cpp.txt", std::ios_base::out);

    tick();
    for (int a = 0; a < samplesToUse; a++)
    {
        for (int b = 0; b < NUMANGLES; b++)
        {
            tempVal = 25 * log10(sqrt(bfOutReal[NUMANGLES * a + b]));
            //bfoFile << tempVal << endl;
            if (tempVal > maxVal)
            {
                maxVal = tempVal;
                timeIdx = a;
            }
        }
    }

    float maxPowers[3] = {0.0, 0.0, 0.0};
    int angleIdxs[3] = {0, 0, 0};

    for (int b = 0; b < NUMANGLES; b++)
    {
        tempVal = 25 * log10(sqrt(bfOutReal[NUMANGLES * timeIdx + b]));
        if (tempVal > maxPowers[0])
        {
            for(int c=2;c>0;c--)
            {
                maxPowers[c] = maxPowers[c-1];
                angleIdxs[c] = angleIdxs[c-1];
            }

            maxPowers[0] = tempVal;
            angleIdxs[0] = b;
        }
    }

    timeKeep8 += tock();
    runCount8++;


    //cout << "Max power detected at : " << rad2deg(angles[angleIdx]) << " degrees at time : " << numCalls + ((float)(SKIP_RATE * timeIdx) / Fs) << " secs for frequency  : " << freqBF << endl;
    cout << "Max power detected at : " << rad2deg(angles[angleIdxs[0]]) << " degrees at time : " << numCalls + ((float)(SKIP_RATE * timeIdx) / Fs) << " secs for frequency  : " << freqBF << endl;
    cout << "Max power detected at : " << rad2deg(angles[angleIdxs[1]]) << " degrees at time : " << numCalls + ((float)(SKIP_RATE * timeIdx) / Fs) << " secs for frequency  : " << freqBF << endl;
    cout << "Max power detected at : " << rad2deg(angles[angleIdxs[2]]) << " degrees at time : " << numCalls + ((float)(SKIP_RATE * timeIdx) / Fs) << " secs for frequency  : " << freqBF << endl;

    //bfoFile.close();

    delete bfOutReal ;
    delete bfOutImag ;
    delete bfInReal ;
    delete bfInImag ;
    delete filtData;


}
/*************************************************************************************************************************************/

/*************************************************************************************************************************************/

int main()
{
    /* READ SIGNAL VALUES*/

    int numSecs = 10;
    int a = 24038 * numSecs;
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

    for (int j = 0; j < NUMCHANNELS * samplesPerBlock; j++)
        sigFile >> curSig[j];

    for (int i = 0; i < numLoops; i++)
    {
        cout << "Processing time stamp : " << i << " secs " << endl;
        gettimeofday(&t2, NULL);
        detectPinger.detectPingerPos(curSig, samplesPerBlock, firCoeff, analyticData);
        gettimeofday(&t3, NULL);
        timeKeep += (float)((t3.tv_sec - t2.tv_sec) * 1000 + ((float)t3.tv_usec - t2.tv_usec) / 1000);
        runCount++;
    }
    /*************************************************************************************************************************************/
    cout << endl;
    cout << "Time to process single second data block                  : " << (timeKeep / runCount)   << "   num of cycles : " << runCount <<  "   Total time : " << timeKeep  << endl;
    cout << "Time to process filter single channel data                : " << (timeKeep1 / runCount1) << "   num of cycles : " << runCount1 << "   Total time : " << timeKeep1 << endl;
    cout << "Time to find beamform freq                                : " << (timeKeep2 / runCount2) << "   num of cycles : " << runCount2 << "   Total time : " << timeKeep2 << endl;
    cout << "Time to process hilbert transform single channel data     : " << (timeKeep3 / runCount3) << "   num of cycles : " << runCount3 << "   Total time : " << timeKeep3 << endl;
    cout << "Time to process beamform single channel data              : " << (timeKeep4 / runCount4) << "   num of cycles : " << runCount4 << "   Total time : " << timeKeep4 << endl;
    cout << "Time to find max power and time and angle                 : " << (timeKeep8 / runCount8) << "   num of cycles : " << runCount8 << "   Total time : " << timeKeep8 << endl;
    cout << "Time to copy single data block to eCPU (Beamform)         : " << (timeKeep5 / runCount5) << "   num of cycles : " << runCount5 << "   Total time : " << timeKeep5 << endl;
    cout << "Time to process single data block in eCPU (Beamform)      : " << (timeKeep6 / runCount6) << "   num of cycles : " << runCount6 << "   Total time : " << timeKeep6 << endl;
    cout << "Time to copy single data block from shared mem (Beamform) : " << (timeKeep7 / runCount7) << "   num of cycles : " << runCount7 << "   Total time : " << timeKeep7 << endl;
    cout << "Time to copy single data block to eCPU (filter)           : " << (timeKeep9 / runCount9) << "   num of cycles : " << runCount9 << "   Total time : " << timeKeep9 << endl;
    cout << "Time to process single data block in eCPU (filter)        : " << (timeKeep10 / runCount10) << "   num of cycles : " << runCount10 << "   Total time : " << timeKeep10 << endl;
    cout << "Time to copy single data block from shared mem (filter)   : " << (timeKeep11 / runCount11) << "   num of cycles : " << runCount11 << "   Total time : " << timeKeep11 << endl;
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