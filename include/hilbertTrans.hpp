#ifndef HILBERTTRANS_HPP
#define HILBERTTRANS_HPP

class hilbertTrans
{
    public :
    void hibFunc(float *inSig, fftwf_complex *outSig, int numSamples,float scale);
    void sigAnalytic(float *inSig, fftwf_complex *outSig, int numSamples);
};

#endif
