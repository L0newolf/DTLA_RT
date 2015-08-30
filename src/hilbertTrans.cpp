#include <math.h>
#include <stddef.h>
#include <iostream>
#include <fftw3.h>
#include <cstring>
#include "hilbertTrans.hpp"
#include <new>
#include <fstream>
#include <sstream>

#include <unistd.h>
#include <sys/time.h>



using namespace std;

//static struct timeval t2, t3;

void hilbertTrans::hibFunc(float *inSig, fftwf_complex *outSig, int numSamples, float scale)
{
    fftwf_complex *xi = new fftwf_complex[numSamples];
    fftwf_complex *xr = new fftwf_complex[numSamples];
    fftwf_plan p, q;

    for (int i = 0; i < numSamples; i++)
    {   xr[i][0] = inSig[i] / scale;
        xr[i][1] = 0.0;
    }

    std::memset(xi, 0, numSamples * sizeof(fftwf_complex));

    fftw_init_threads();
    fftw_plan_with_nthreads(4);

    p = fftwf_plan_dft_1d(numSamples, xr, xi, FFTW_FORWARD, FFTW_ESTIMATE);
    q = fftwf_plan_dft_1d(numSamples, xi, outSig, FFTW_BACKWARD, FFTW_ESTIMATE);

    fftwf_execute(p);

    /*
    gettimeofday(&t2, NULL);
    fftwf_execute(p);
    gettimeofday(&t3, NULL);
    cout<<"Time to fwd fft : "<<(float)((t3.tv_sec - t2.tv_sec) * 1000 + ((float)t3.tv_usec - t2.tv_usec) / 1000)<<endl;
    */
    for (int i = 0; i < numSamples; i++)
    {
        if(i<numSamples/2)
        {
            xi[i][0] *= 2;
            xi[i][1] *= 2;
        }
        else
        {
            xi[i][0] = 0;
            xi[i][1] = 0;
        }

    }

    fftwf_execute(q);
    
    /*
    gettimeofday(&t2, NULL);
    fftwf_execute(q);
    gettimeofday(&t3, NULL);
    cout<<"Time to bkwd fft : "<<(float)((t3.tv_sec - t2.tv_sec) * 1000 + ((float)t3.tv_usec - t2.tv_usec) / 1000)<<endl;
    */
    fftwf_destroy_plan(p);
    fftwf_destroy_plan(q);
    fftwf_free(xi);
    fftwf_free(xr);
    fftw_cleanup_threads();
    //fftw_cleanup();
}

void hilbertTrans::sigAnalytic(float *inSig, fftwf_complex *outSig, int numSamples)
{
    float scale = 0.0;


    for (int j = 0; j < numSamples; j++)
        scale += (inSig[j] * inSig[j]);
    scale = sqrt(scale / numSamples);

    hibFunc(inSig, outSig, numSamples, scale);
}
