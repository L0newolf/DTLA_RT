//---------------------------------------------------------------------------

#include <math.h>
#include <stddef.h>
#include <iostream>
#include "genFilt.hpp"
#include <new>

#define M_2PI  6.28318530717958647692
//---------------------------------------------------------------------------

using namespace std;

void genFilt::BasicFIR(float *FirCoeff, int NumTaps, TBasicPassType PassType, float OmegaC, float BW, TWindowType WindowType, float WinBeta)
{
 int j;
 float Arg, OmegaLow, OmegaHigh;

 switch(PassType)
  {
   case LPF:
	for(j=0; j<NumTaps; j++)
     {
      Arg = (float)j - (float)(NumTaps-1) / 2.0;
      FirCoeff[j] = OmegaC * Sinc(OmegaC * Arg * M_PI);
     }
    break;

   case HPF:
    if(NumTaps % 2 == 1) // Odd tap counts
     {
      for(j=0; j<NumTaps; j++)
       {
        Arg = (float)j - (float)(NumTaps-1) / 2.0;
        FirCoeff[j] = Sinc(Arg * M_PI) - OmegaC * Sinc(OmegaC * Arg * M_PI);
       }
     }

    else  // Even tap counts
      {
       for(j=0; j<NumTaps; j++)
        {
         Arg = (float)j - (float)(NumTaps-1) / 2.0;
         if(Arg == 0.0)FirCoeff[j] = 0.0;
         else FirCoeff[j] = cos(OmegaC * Arg * M_PI) / M_PI / Arg  + cos(Arg * M_PI);
        }
      }
   break;

   case BPF:
    OmegaLow  = OmegaC - BW/2.0;
    OmegaHigh = OmegaC + BW/2.0;
	for(j=0; j<NumTaps; j++)
     {
      Arg = (float)j - (float)(NumTaps-1) / 2.0;
      if(Arg == 0.0)FirCoeff[j] = 0.0;
      else FirCoeff[j] =  ( cos(OmegaLow * Arg * M_PI) - cos(OmegaHigh * Arg * M_PI) ) / M_PI / Arg ;
     }
   break;

   case NOTCH:
    OmegaLow  = OmegaC - BW/2.0;
    OmegaHigh = OmegaC + BW/2.0;
	for(j=0; j<NumTaps; j++)
     {
      Arg = (float)j - (float)(NumTaps-1) / 2.0;
      FirCoeff[j] =  Sinc(Arg * M_PI) - OmegaHigh * Sinc(OmegaHigh * Arg * M_PI) - OmegaLow * Sinc(OmegaLow * Arg * M_PI);
     }
   break;
  }

 WindowData(FirCoeff, NumTaps, WindowType, 0.0, WinBeta, false);

}

//---------------------------------------------------------------------------

float genFilt::Bessel(float x)
{
 float Sum=0.0, XtoIpower;
 int i, j, Factorial;
 for(i=1; i<10; i++)
  {
   XtoIpower = pow(x/2.0, (float)i);
   Factorial = 1;
   for(j=1; j<=i; j++)Factorial *= j;
   Sum += pow(XtoIpower / (float)Factorial, 2.0);
  }
 return(1.0 + Sum);
}

//-----------------------------------------------------------------------------

float genFilt::Sinc(float x)
{
 if(x > -1.0E-5 && x < 1.0E-5)return(1.0);
 return(sin(x)/x);
}

//---------------------------------------------------------------------------

void genFilt::WindowData(float *Data, int N, TWindowType WindowType, float Alpha, float Beta, bool UnityGain)
{
 if(WindowType == wtNONE) return;

 int j, M, TopWidth;
 float dM, *WinCoeff;

 if(WindowType == wtKAISER ||  WindowType == wtFLATTOP )Alpha = 0.0;

 if(Alpha < 0.0)Alpha = 0.0;
 if(Alpha > 1.0)Alpha = 1.0;

 if(Beta < 0.0)Beta = 0.0;
 if(Beta > 10.0)Beta = 10.0;

 WinCoeff  = new float[N+2];
 if(WinCoeff == NULL)
  {
   std::cout<<"Failed to allocate memory in FFTFunctions::WindowFFTData() "<<std::endl;
   return;
  }

 TopWidth = (int)( Alpha * (float)N );
 if(TopWidth%2 != 0)TopWidth++;
 if(TopWidth > N)TopWidth = N;
 M = N - TopWidth;
 dM = M + 1;

 if(WindowType == wtKAISER)
  {
   float Arg;
   for(j=0; j<M; j++)
	{
	 Arg = Beta * sqrt(1.0 - pow( ((float)(2*j+2) - dM) / dM, 2.0) );
	 WinCoeff[j] = Bessel(Arg) / Bessel(Beta);
	}
  }

 else if(WindowType == wtSINC)
  {
   for(j=0; j<M; j++)WinCoeff[j] = Sinc((float)(2*j+1-M)/dM * M_PI );
   for(j=0; j<M; j++)WinCoeff[j] = pow(WinCoeff[j], Beta);
  }

 else if(WindowType == wtSINE)
  {
   for(j=0; j<M/2; j++)WinCoeff[j] = sin((float)(j+1) * M_PI / dM);
   for(j=0; j<M/2; j++)WinCoeff[j] = pow(WinCoeff[j], Beta);
  }

 else if(WindowType == wtHANNING)
  {
   for(j=0; j<M/2; j++)WinCoeff[j] = 0.5 - 0.5 * cos((float)(j+1) * M_2PI / dM);
  }

 else if(WindowType == wtHAMMING)
  {
   for(j=0; j<M/2; j++)
   WinCoeff[j] = 0.54 - 0.46 * cos((float)(j+1) * M_2PI / dM);
  }

 else if(WindowType == wtBLACKMAN)
  {
   for(j=0; j<M/2; j++)
	{
	 WinCoeff[j] = 0.42
	 - 0.50 * cos((float)(j+1) * M_2PI / dM)
	 + 0.08 * cos((float)(j+1) * M_2PI * 2.0 / dM);
	}
  }

else if(WindowType == wtFLATTOP)
  {
   for(j=0; j<=M/2; j++)
	{
	 WinCoeff[j] = 1.0
	 - 1.93293488969227 * cos((float)(j+1) * M_2PI / dM)
	 + 1.28349769674027 * cos((float)(j+1) * M_2PI * 2.0 / dM)
	 - 0.38130801681619 * cos((float)(j+1) * M_2PI * 3.0 / dM)
	 + 0.02929730258511 * cos((float)(j+1) * M_2PI * 4.0 / dM);
	}
  }


 else if(WindowType == wtBLACKMAN_HARRIS)
  {
   for(j=0; j<M/2; j++)
	{
	 WinCoeff[j] = 0.35875
	 - 0.48829 * cos((float)(j+1) * M_2PI / dM)
	 + 0.14128 * cos((float)(j+1) * M_2PI * 2.0 / dM)
	 - 0.01168 * cos((float)(j+1) * M_2PI * 3.0 / dM);
	}
  }

 else if(WindowType == wtBLACKMAN_NUTTALL)
  {
   for(j=0; j<M/2; j++)
	{
	 WinCoeff[j] = 0.3535819
	 - 0.4891775 * cos((float)(j+1) * M_2PI / dM)
	 + 0.1365995 * cos((float)(j+1) * M_2PI * 2.0 / dM)
	 - 0.0106411 * cos((float)(j+1) * M_2PI * 3.0 / dM);
	}
  }

 else if(WindowType == wtNUTTALL)
  {
   for(j=0; j<M/2; j++)
	{
	 WinCoeff[j] = 0.355768
	 - 0.487396 * cos((float)(j+1) * M_2PI / dM)
	 + 0.144232 * cos((float)(j+1) * M_2PI * 2.0 / dM)
	 - 0.012604 * cos((float)(j+1) * M_2PI * 3.0 / dM);
	}
  }

 else if(WindowType == wtKAISER_BESSEL)
  {
   for(j=0; j<=M/2; j++)
	{
	 WinCoeff[j] = 0.402
	 - 0.498 * cos(M_2PI * (float)(j+1) / dM)
	 + 0.098 * cos(2.0 * M_2PI * (float)(j+1) / dM)
	 + 0.001 * cos(3.0 * M_2PI * (float)(j+1) / dM);
	}
  }

 else if(WindowType == wtTRAPEZOID) // Rectangle for Alpha = 1  Triangle for Alpha = 0
  {
   int K = M/2;
   if(M%2)K++;
   for(j=0; j<K; j++)WinCoeff[j] = (float)(j+1) / (float)K;
  }

else if(WindowType == wtGAUSS)
  {
   for(j=0; j<M/2; j++)
    {
     WinCoeff[j] = ((float)(j+1) - dM/2.0) / (dM/2.0) * 2.7183;
     WinCoeff[j] *= WinCoeff[j];
     WinCoeff[j] = exp(-WinCoeff[j]);
    }
  }

 else
  {
   std::cout<<"Incorrect window type in WindowFFTData"<<std::endl;
   delete[] WinCoeff;
   return;
  }

 for(j=0; j<M/2; j++)WinCoeff[N-j-1] = WinCoeff[j];

 if(WindowType != wtKAISER &&  WindowType != wtFLATTOP)
  {
   for(j=M/2; j<N-M/2; j++)WinCoeff[j] = 1.0;
  }

 if(UnityGain)
  {
   float Sum = 0.0;
   for(j=0; j<N; j++)Sum += WinCoeff[j];
   Sum /= (float)N;
   if(Sum != 0.0)for(j=0; j<N; j++)WinCoeff[j] /= Sum;
  }

 for(j=0; j<N; j++)Data[j] *= WinCoeff[j];

 delete[] WinCoeff;

}

//---------------------------------------------------------------------------


void genFilt::createWindow(float *WinCoeff, int N, TWindowType WindowType, float Alpha, float Beta, bool UnityGain)
{
 if(WindowType == wtNONE) return;

 int j, M, TopWidth;
 float dM;

 if(WindowType == wtKAISER ||  WindowType == wtFLATTOP )Alpha = 0.0;

 if(Alpha < 0.0)Alpha = 0.0;
 if(Alpha > 1.0)Alpha = 1.0;

 if(Beta < 0.0)Beta = 0.0;
 if(Beta > 10.0)Beta = 10.0;

 TopWidth = (int)( Alpha * (float)N );
 if(TopWidth%2 != 0)TopWidth++;
 if(TopWidth > N)TopWidth = N;
 M = N - TopWidth;
 dM = M + 1;

 if(WindowType == wtKAISER)
  {
   float Arg;
   for(j=0; j<M; j++)
	{
	 Arg = Beta * sqrt(1.0 - pow( ((float)(2*j+2) - dM) / dM, 2.0) );
	 WinCoeff[j] = Bessel(Arg) / Bessel(Beta);
	}
  }

 else if(WindowType == wtSINC)
  {
   for(j=0; j<M; j++)WinCoeff[j] = Sinc((float)(2*j+1-M)/dM * M_PI );
   for(j=0; j<M; j++)WinCoeff[j] = pow(WinCoeff[j], Beta);
  }

 else if(WindowType == wtSINE)
  {
   for(j=0; j<M/2; j++)WinCoeff[j] = sin((float)(j+1) * M_PI / dM);
   for(j=0; j<M/2; j++)WinCoeff[j] = pow(WinCoeff[j], Beta);
  }

 else if(WindowType == wtHANNING)
  {
   for(j=0; j<M/2; j++)WinCoeff[j] = 0.5 - 0.5 * cos((float)(j+1) * M_2PI / dM);
  }

 else if(WindowType == wtHAMMING)
  {
   for(j=0; j<M/2; j++)
   WinCoeff[j] = 0.54 - 0.46 * cos((float)(j+1) * M_2PI / dM);
  }

 else if(WindowType == wtBLACKMAN)
  {
   for(j=0; j<M/2; j++)
	{
	 WinCoeff[j] = 0.42
	 - 0.50 * cos((float)(j+1) * M_2PI / dM)
	 + 0.08 * cos((float)(j+1) * M_2PI * 2.0 / dM);
	}
  }

else if(WindowType == wtFLATTOP)
  {
   for(j=0; j<=M/2; j++)
	{
	 WinCoeff[j] = 1.0
	 - 1.93293488969227 * cos((float)(j+1) * M_2PI / dM)
	 + 1.28349769674027 * cos((float)(j+1) * M_2PI * 2.0 / dM)
	 - 0.38130801681619 * cos((float)(j+1) * M_2PI * 3.0 / dM)
	 + 0.02929730258511 * cos((float)(j+1) * M_2PI * 4.0 / dM);
	}
  }


 else if(WindowType == wtBLACKMAN_HARRIS)
  {
   for(j=0; j<M/2; j++)
	{
	 WinCoeff[j] = 0.35875
	 - 0.48829 * cos((float)(j+1) * M_2PI / dM)
	 + 0.14128 * cos((float)(j+1) * M_2PI * 2.0 / dM)
	 - 0.01168 * cos((float)(j+1) * M_2PI * 3.0 / dM);
	}
  }

 else if(WindowType == wtBLACKMAN_NUTTALL)
  {
   for(j=0; j<M/2; j++)
	{
	 WinCoeff[j] = 0.3535819
	 - 0.4891775 * cos((float)(j+1) * M_2PI / dM)
	 + 0.1365995 * cos((float)(j+1) * M_2PI * 2.0 / dM)
	 - 0.0106411 * cos((float)(j+1) * M_2PI * 3.0 / dM);
	}
  }

 else if(WindowType == wtNUTTALL)
  {
   for(j=0; j<M/2; j++)
	{
	 WinCoeff[j] = 0.355768
	 - 0.487396 * cos((float)(j+1) * M_2PI / dM)
	 + 0.144232 * cos((float)(j+1) * M_2PI * 2.0 / dM)
	 - 0.012604 * cos((float)(j+1) * M_2PI * 3.0 / dM);
	}
  }

 else if(WindowType == wtKAISER_BESSEL)
  {
   for(j=0; j<=M/2; j++)
	{
	 WinCoeff[j] = 0.402
	 - 0.498 * cos(M_2PI * (float)(j+1) / dM)
	 + 0.098 * cos(2.0 * M_2PI * (float)(j+1) / dM)
	 + 0.001 * cos(3.0 * M_2PI * (float)(j+1) / dM);
	}
  }

 else if(WindowType == wtTRAPEZOID) // Rectangle for Alpha = 1  Triangle for Alpha = 0
  {
   int K = M/2;
   if(M%2)K++;
   for(j=0; j<K; j++)WinCoeff[j] = (float)(j+1) / (float)K;
  }

else if(WindowType == wtGAUSS)
  {
   for(j=0; j<M/2; j++)
    {
     WinCoeff[j] = ((float)(j+1) - dM/2.0) / (dM/2.0) * 2.7183;
     WinCoeff[j] *= WinCoeff[j];
     WinCoeff[j] = exp(-WinCoeff[j]);
    }
  }

 else
  {
   std::cout<<"Incorrect window type in WindowFFTData"<<std::endl;
   delete[] WinCoeff;
   return;
  }

 for(j=0; j<M/2; j++)WinCoeff[N-j-1] = WinCoeff[j];

 if(WindowType != wtKAISER &&  WindowType != wtFLATTOP)
  {
   for(j=M/2; j<N-M/2; j++)WinCoeff[j] = 1.0;
  }

 if(UnityGain)
  {
   float Sum = 0.0;
   for(j=0; j<N; j++)Sum += WinCoeff[j];
   Sum /= (float)N;
   if(Sum != 0.0)for(j=0; j<N; j++)WinCoeff[j] /= Sum;
  }

}
//---------------------------------------------------------------------------
