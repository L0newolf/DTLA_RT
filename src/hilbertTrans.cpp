#include <math.h>
#include <stddef.h>
#include <iostream>
#include <fftw3.h>
#include <cstring>
#include "hilbertTrans.hpp"
#include <new>
#include <fstream>
#include <sstream>

using namespace std;

void hilbertTrans::hibFunc(float *inSig, fftwf_complex *outSig, int numSamples,float scale)
{
        fftwf_complex *xi = new fftwf_complex[numSamples];
        fftwf_complex *xr = new fftwf_complex[numSamples];
        fftwf_plan p, q;

        for(int i=0; i<numSamples; i++)
        { xr[i][0] = inSig[i]/scale;
          xr[i][1] = 0.0; }

        std::memset(xi,0,numSamples*sizeof(fftwf_complex));

        p = fftwf_plan_dft_1d(numSamples, xr, xi, FFTW_FORWARD, FFTW_ESTIMATE);
        q = fftwf_plan_dft_1d(numSamples, xi, outSig, FFTW_BACKWARD, FFTW_ESTIMATE);

        fftwf_execute(p);

      //cout<<xi[0][0]<<"  "<<xi[0][1]<<endl;

        for(int i=numSamples/2+1;i<numSamples;i++)
            {
                    xi[i][0] = 0;
                    xi[i][1] = 0;

            }

        fftwf_execute(q);

        /*
        std::fstream analyticSigFile("/home/anshu/Dropbox/DTLA_CPP/hib_Sig_CPP.txt", std::fstream::out | std::fstream::app);
        for(int i=0; i<numSamples; i++)
            {
            analyticSigFile<<(outSig[i][0])<<endl;
            }
        analyticSigFile.close();
        */

        /*
        for(int i=0;i<numSamples;i++)
        {
            outSig[i][1]*=-1.0;
          }
        */

        fftwf_destroy_plan(p);
        fftwf_destroy_plan(q);
        fftwf_free(xi);
        fftwf_free(xr);
}

void hilbertTrans::sigAnalytic(float *inSig, fftwf_complex *outSig, int numSamples)
{
      float scale = 0.0;

      for(int j=0;j<numSamples;j++)
          scale+=(inSig[j]*inSig[j]);

      scale = sqrt(scale/numSamples);

    hibFunc(inSig, outSig, numSamples,scale);

}
