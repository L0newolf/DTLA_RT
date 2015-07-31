#include <math.h>
#include <stddef.h>
#include <iostream>
#include <algorithm>
#include <utility>
#include <cstring>

#include "findSigFreq.hpp"
#include "genFilt.hpp"
#include <fftw3.h>


#include <fstream>
#include <sstream>

using namespace std;

std::pair<int, float> findSigFreq::detectSigFreq(float *inSig, int Fs, int nFFT, int numSamples, int numOverlap)
{

        int winMov = nFFT - numOverlap;
        int loops = floor((numSamples-numOverlap)/winMov)-1;
        int index = 0;
        int Plength = floor(nFFT/2)+1;
        float temp;
        int freq;
        float freqStep = (float)Fs/nFFT;

        float *power = new float [Plength];
        float *winCoeff = new float[nFFT];

        fftwf_complex *curData, *sigFFT;
        curData = (fftwf_complex*) fftwf_malloc(sizeof(fftwf_complex) * nFFT);
        sigFFT = (fftwf_complex*) fftwf_malloc(sizeof(fftwf_complex) * nFFT);
        fftwf_plan p;

        genFilt filter;
        filter.createWindow(winCoeff, nFFT, wtBLACKMAN, 0.0, 0.0, false);

        std::memset(power,0,Plength*sizeof(float));

        p = fftwf_plan_dft_1d(nFFT, curData, sigFFT, FFTW_FORWARD, FFTW_ESTIMATE);

        for(int i=0; i<loops; i++)
        {
                for(int a=0; a<nFFT; a++)
                {
                        curData[a][0] = inSig[index+a]*winCoeff[a];
                        curData[a][1] = 0.0;
                }

                fftwf_execute(p);
                for (int j=0; j<Plength; j++)
                        power[j] = power[j] + sqrt(pow(sigFFT[j][0],2)+pow(sigFFT[j][1],2));
                index += winMov;
        }

        index = 0;
        temp = power[0];
        for(int i=1; i<Plength; i++)
        {
                if(power[i] > temp)
                {
                        temp = power[i];
                        index = i;
                }
        }

        fftwf_destroy_plan(p);
        fftwf_free(curData);
        fftwf_free(sigFFT);
        delete []power;
        delete []winCoeff;

        freq = floor((index)*freqStep);

        return std::make_pair(freq, temp);
}
