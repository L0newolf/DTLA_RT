#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <e-lib.h>

#include "settings.h"

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

int main(void)

{
  int *done, *numSamples;
  float *dataReal, *dataImag, *sinValsAngles, *cosValsAngles, *sinValsSamples, *cosValsSamples;

  float val, realVal_1, imagVal_1,realVal, imagVal;
  int i, a;
  epc_init();

  e_memseg_t        extMemBuf;

  unsigned int ptr = 0x2000;
  dataReal = (float *)ptr;
  ptr += WINDOWPERCORE * sizeof(float);
  dataImag = (float *)ptr;;
  ptr += WINDOWPERCORE * sizeof(float);
  sinValsAngles = (float *)ptr;
  ptr += (NUMANGLES) * sizeof(float);
  cosValsAngles = (float *)ptr;
  ptr += (NUMANGLES) * sizeof(float);
  sinValsSamples = (float *)ptr;
  ptr += (WINDOWPERCORE) * sizeof(float);
  cosValsSamples = (float *)ptr;
  ptr += (WINDOWPERCORE) * sizeof(float);
  numSamples = (int *)ptr;
  ptr += sizeof(int);

  float *bfoReal = (float *)0x2500;
  float *bfoImag = (float *)0x5000;

  done = (int *)0x7500;

  int idx;

  while (1) 
  {
    epc_wait();
    e_irq_set(0, 0, E_USER_INT);
    

    for (i = 0; i < WINDOWPERCORE; i++) 
    {
      realVal_1 =dataReal[i] * cosValsSamples[i] - dataImag[i] * sinValsSamples[i];
      imagVal_1 =dataReal[i] * sinValsSamples[i] + dataImag[i] * cosValsSamples[i];


      for (a = 0; a < NUMANGLES; a++) 
      {
        realVal = cosValsAngles[a] * realVal_1 + sinValsAngles[a] * imagVal_1;
        imagVal = cosValsAngles[a] * imagVal_1 - sinValsAngles[a] * realVal_1;
        bfoReal[i * NUMANGLES + a ] = bfoReal[i * NUMANGLES + a ] + realVal;
        bfoImag[i * NUMANGLES + a ] = bfoImag[i * NUMANGLES + a ] + imagVal;
      }
       
    }

    (*(done)) = 0x00000001;
  }

  return EXIT_SUCCESS;
}
