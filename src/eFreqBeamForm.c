#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <e-lib.h>

#include <unistd.h>

#include "settings.h"

#define e_memcopy(dst, src, size) e_dma_copy(dst, src, size)

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

int main(void)

{
  float *pmemBfoReal, *pmemBfoImag;
  void *src, *dst;

  int *done, *numSamples;
  float *dataReal, *dataImag, *sinValsAngles, *cosValsAngles, *sinValsSamples, *cosValsSamples;

  float val, realVal_1, imagVal_1, realVal, imagVal;
  int i, a;
  epc_init();

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

  int coreNum = getCoreNum(e_get_coreid());
  int sizeBuf = WINDOWPERCORE * NUMANGLES * sizeof(float);

  pmemBfoReal = (float *) (e_emem_config.base + 0x00000000 + coreNum * sizeBuf );
  pmemBfoImag = (float *) (e_emem_config.base + 0x00010000 + coreNum * sizeBuf );

  (*(done)) == 0x00000001;

  while (1)
  {

    epc_wait();
    e_irq_set(0, 0, E_USER_INT);


    //if((*(done)) == 0x00000000)
    //{

    for (i = 0; i < WINDOWPERCORE; i++)
    {

      realVal_1 = dataReal[i] * cosValsSamples[i] - dataImag[i] * sinValsSamples[i];
      imagVal_1 = dataReal[i] * sinValsSamples[i] + dataImag[i] * cosValsSamples[i];


      for (a = 0; a < NUMANGLES; a++)
      {
        realVal = cosValsAngles[a] * realVal_1 + sinValsAngles[a] * imagVal_1;
        imagVal = cosValsAngles[a] * imagVal_1 - sinValsAngles[a] * realVal_1;
        bfoReal[i * NUMANGLES + a ] = bfoReal[i * NUMANGLES + a ] + realVal;
        bfoImag[i * NUMANGLES + a ] = bfoImag[i * NUMANGLES + a ] + imagVal;
      }

    }

    dst = (void *)pmemBfoReal;
    src = (void *)bfoReal;
    e_memcopy(dst, src, sizeBuf);

    dst = (void *)pmemBfoImag;
    src = (void *)bfoImag;
    e_memcopy(dst, src, sizeBuf);

    (*(done)) = 0x00000001;

    //}
  }

  return EXIT_SUCCESS;
}
