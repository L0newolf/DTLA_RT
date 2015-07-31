#include <math.h>
#include <stddef.h>
#include <iostream>
#include "filtSignal.hpp"

using namespace std;

void filtSignal::filter (float *outSig,float *filtCoeff,int filtLen,float *inSig,int numSamples)
{

  float tmp =0;
  int j;

  for(int k=0;k<numSamples;k++)
  {
      for(int i=0;i<filtLen;i++)
      {
          j=k-i;
          if(j>0)
            tmp += filtCoeff[i]*inSig[j];
      }
      outSig[k] = tmp;
  }

}

