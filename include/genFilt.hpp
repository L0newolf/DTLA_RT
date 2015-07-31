
/*------------------------------------------------------------------------------*/
/*------------------- start of genFilt.h ---------------------------------------*/

#ifndef GENFILT_HPP
#define GENFILT_HPP

enum TBasicPassType {LPF, HPF, BPF, NOTCH};

enum TWindowType {wtNONE, wtKAISER, wtSINC, wtHANNING, wtHAMMING, wtBLACKMAN,
          wtFLATTOP, wtBLACKMAN_HARRIS, wtBLACKMAN_NUTTALL, wtNUTTALL,
          wtKAISER_BESSEL, wtTRAPEZOID, wtGAUSS, wtSINE, wtTEST };

class genFilt{

  public :
 void BasicFIR(float *FirCoeff, int NumTaps, TBasicPassType PassType, float OmegaC, float BW, TWindowType WindowType, float WinBeta);
 void createWindow(float *WinCoeff, int N, TWindowType WindowType, float Alpha, float Beta, bool UnityGain);

 private :

 float Bessel(float x);
 float Sinc(float x);
 void WindowData(float *Data, int N, TWindowType WindowType, float Alpha, float Beta, bool UnityGain);

 };

#endif

 /*------------------- end of genFilt.h ---------------------------------------*/
 /*------------------------------------------------------------------------------*/
