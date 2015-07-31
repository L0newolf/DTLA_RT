#ifndef FILTSIGNAL_HPP
#define FILTSIGNAL_HPP

class filtSignal{

  public :
            void filter (float *outSig,float *filtCoeff,int filtLen,float *inSig,int numSamples);
};
#endif
