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

/*************************************************************************************************************************************/
float timeKeep10 = 0.0;
static struct timeval t10;
static inline void tick(void) { gettimeofday(&t10, NULL); }
static inline float tock(void) {
    struct timeval t11;
    gettimeofday(&t11, NULL);
    return (t11.tv_sec - t10.tv_sec) * 1000 + ((float)t11.tv_usec - t10.tv_usec) / 1000 ;
}
/*************************************************************************************************************************************/

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

    timeKeep10 = 0.0;
    tick();
    fftwf_execute(p);
    timeKeep10 = tock();
    cout << "Time to coumpute fftw plan p  : " << timeKeep10 << endl;

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

    timeKeep10 = 0.0;
    tick();
    fftwf_execute(q);
    timeKeep10 = tock();
    cout << "Time to coumpute fftw plan q  : " << timeKeep10 << endl;

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

    timeKeep10 = 0.0;
    tick();
    for (int j = 0; j < numSamples; j++)
        scale += (inSig[j] * inSig[j]);
    scale = sqrt(scale / numSamples);
    timeKeep10 = tock();
    cout << "Time to coumpute scale : " << timeKeep10 << endl;

    hibFunc(inSig, outSig, numSamples, scale);
}
