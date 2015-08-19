#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <e-lib.h>

#include "settings.h"

#define e_memcopy(dst, src, size) e_dma_copy(dst, src, size)

volatile float  bfOutReal[WINDOWPERCORE * NUMANGLES] SECTION("section_core");
volatile float  bfOutImag[WINDOWPERCORE * NUMANGLES] SECTION("section_core");

static void __attribute__((interrupt)) irqhandler(int n) {
  // do nothing
}

static inline void epc_wait(void) { __asm__ __volatile__("idle"); }

static inline void epc_signal(e_coreid_wrap_t dir) {
  unsigned row, col;
  e_neighbor_id(dir, E_GROUP_WRAP, &row, &col);
  e_irq_set(row, col, E_USER_INT);
}

static inline void epc_init(void) {
  e_irq_attach(E_USER_INT, &irqhandler);
  e_irq_mask(E_USER_INT, E_FALSE);
  e_irq_global_mask(E_FALSE);
}

static int getCoreNum( e_coreid_t coreid )
{
  int coreNum;
  int row     = e_group_config.core_row;
  int col     = e_group_config.core_col;
  coreNum = row * e_group_config.group_cols + col;

  return coreNum;
}

void filter (float *filtSig,float *filtCoeff,int filtLen,float *inSig)
{

  float tmp =0;
  int j,k,i;

  for( k=0;k<FILTERWINDOW;k++)
  {
      for( i=0;i<filtLen;i++)
      {
          j=k-i;
          if(j>0)
            tmp += filtCoeff[i]*inSig[j];
      }
      filtSig[k] = tmp;
  }

}

int main(void)

{
  float *pmemBfoReal, *pmemBfoImag, *filtOpt;
  void *src, *dst;
  unsigned int ptr;
  int i, j, k, a;

  //Beamforming specific variables
  int *done,*loopCount;
  float *bfInReal,*bfInImag,*freqBF,*angles,*sinValAngles,*cosValAngles,*sinValChannels,*cosValChannels;
  float temp1_real,temp1_imag,temp2_real,temp2_imag,temp3_real,temp3_imag,temp4_real,temp4_imag;

  //Signal filtering specific variables
  float *inSig,*filtCoeff,*filtSig;
  int *numSamples;

  epc_init();

  done = (int *)0x7500;

  int coreNum = getCoreNum(e_get_coreid());
  int sizeBuf = WINDOWPERCORE * NUMANGLES * sizeof(float);

  pmemBfoReal = (float *) (e_emem_config.base + 0x00000000 + coreNum * sizeBuf );
  pmemBfoImag = (float *) (e_emem_config.base + 0x00100000 + coreNum * sizeBuf );
  filtOpt = (float *) (e_emem_config.base + 0x00200000 + coreNum * FILTERWINDOW*sizeof(float) );

  while (1)
  {
    epc_wait();
    e_irq_set(0, 0, E_USER_INT);

    if ((*(done)) == PROCESS_BEAMFORM)
    {
      
      ptr = 0x2000;
      bfInReal = (float *)ptr;
      ptr += WINDOWPERCORE * NUMCHANNELS * sizeof(float);
      bfInImag = (float *)ptr;;
      ptr += WINDOWPERCORE * NUMCHANNELS * sizeof(float);
      angles = (float *)ptr;;
      ptr += NUMANGLES * sizeof(float);
      loopCount = (int *)ptr;
      ptr += sizeof(int);

      ptr = 0x2500;
      sinValChannels = (float *)ptr;
      ptr += NUMCHANNELS * WINDOWPERCORE * sizeof(float);
      cosValChannels = (float *)ptr;
      ptr += NUMCHANNELS * WINDOWPERCORE * sizeof(float);
      sinValAngles = (float *)ptr;
      ptr += NUMCHANNELS * NUMANGLES * sizeof(float);
      cosValAngles = (float *)ptr;
      ptr += NUMCHANNELS * NUMANGLES * sizeof(float);


      for (j = 0; j < WINDOWPERCORE * NUMANGLES; j++)
      {
        bfOutReal[j] = 0.0;
        bfOutImag[j] = 0.0;
      }

      for (k = 0; k < WINDOWPERCORE; k++)
      {
        for (j = 0; j < NUMCHANNELS; j++)
        {

          temp1_real = cosValChannels[NUMCHANNELS * k + j];
          temp1_imag = sinValChannels[NUMCHANNELS * k + j];

          temp2_real = bfInReal[NUMCHANNELS * k + j] * temp1_real - bfInImag[NUMCHANNELS * k + j] * temp1_imag;
          temp2_imag = bfInReal[NUMCHANNELS * k + j] * temp1_imag + bfInImag[NUMCHANNELS * k + j] * temp1_real;

          for (a = 0; a < NUMANGLES; a++)
          {

            temp3_real = sinValAngles[NUMANGLES * j + a];
            temp3_imag = cosValAngles[NUMANGLES * j + a];

            temp4_real = temp2_real * temp3_real + temp2_imag * temp3_imag;
            temp4_imag = -temp2_real * temp3_imag + temp2_imag * temp3_real;

            bfOutReal[NUMANGLES * k + a] += temp4_real;
            bfOutImag[NUMANGLES * k + a] += temp4_imag;
          }
        }
      }


      for (k = 0; k < WINDOWPERCORE * NUMANGLES; k++)
      {
            bfOutReal[k] = (bfOutReal[k]*bfOutReal[k] + bfOutImag[k]*bfOutImag[k]);
      }
      

      dst = (void *)pmemBfoReal;
      src = (void *)bfOutReal;
      e_memcopy(dst, src, sizeBuf);

      /*
      dst = (void *)pmemBfoImag;
      src = (void *)bfOutImag;
      e_memcopy(dst, src, sizeBuf);
      */

      (*(done)) = PROCESS_COMPLETE;
    }
    else //if((*(done)) == PROCESS_FILTER)
    {
      
      ptr = 0x2500;
      inSig = (float *)ptr;

      ptr = 0x5000;
      filtSig = (float *)ptr;

      ptr = 0x7000;
      filtCoeff = (float *)ptr;
      ptr += (numTaps+1)*sizeof(float);

      filter (filtSig,filtCoeff,numTaps+1,inSig);

      dst = (void *)filtOpt;
      src = (void *)filtSig;
      e_memcopy(dst, src, FILTERWINDOW*sizeof(float));

      (*(done)) = PROCESS_COMPLETE;

    }
    


  }

  return EXIT_SUCCESS;
}
