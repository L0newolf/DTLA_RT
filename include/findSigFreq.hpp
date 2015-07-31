#ifndef FINDSIGFREQ_HPP
#define FINDSIGFREQ_HPP

class findSigFreq
{
   public :
   std::pair<int, float> detectSigFreq(float *inSig, int Fs, int nFFT, int numSamples, int numOverlap);
};

#endif
