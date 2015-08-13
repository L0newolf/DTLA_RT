#ifndef RTPINGERDET_HPP
#define RTPINGERDET_HPP

  class rtPingerDet{

    public :
    void detectPingerPos(float *data,int numSamples,float *firCoeff,fftwf_complex *analyticData);
    rtPingerDet();

    private :
    int numCalls;
    int lastFreqUsed;

  };

#endif
