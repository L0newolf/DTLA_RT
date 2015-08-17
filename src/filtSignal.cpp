#include <math.h>
#include <stddef.h>
#include <iostream>
#include <pthread.h>

#include "filtSignal.hpp"

using namespace std;

#define NUM_THREADS 1

struct thread_data {
  float *outSig;
  float *filtCoeff;
  int filtLen;
  float *inSig;
  int numSamples;
};

struct thread_data thread_data_array[NUM_THREADS];

void *doFilter(void *threadarg)
{
  float *filtSig;
  float *coeffs;
  int lenFilt;
  float *unfiltSig;
  int sampleLen;

  struct thread_data *my_data;
  my_data = (struct thread_data *) threadarg;

  filtSig = my_data->outSig;
  coeffs = my_data->filtCoeff;
  lenFilt = my_data->filtLen;
  unfiltSig = my_data->inSig;
  sampleLen = my_data->numSamples;

  float tmp = 0;
  int j;

  for (int k = 0; k < sampleLen; k++)
  {
    for (int i = 0; i < lenFilt; i++)
    {
      j = k - i;
      if (j > 0)
        tmp += coeffs[i] * unfiltSig[j];
    }
    filtSig[k] = tmp;
  }

  return NULL;
}

void filtSignal::filter (float *outSig, float *filtCoeff, int filtLen, float *inSig, int numSamples)
{

  pthread_t threadProcs[NUM_THREADS];
  int step = numSamples / NUM_THREADS;

  for (int i = 0; i < NUM_THREADS; i++)
  {
    thread_data_array[i].outSig = &outSig[step * i];
    thread_data_array[i].filtCoeff = filtCoeff;
    thread_data_array[i].filtLen = filtLen;
    thread_data_array[i].inSig = &inSig[step * i];
    thread_data_array[i].numSamples = numSamples;
  }

  int  iret[NUM_THREADS];

  for (int i = 0; i < NUM_THREADS; i++)
  {
    iret[i] = pthread_create( &threadProcs[i], NULL, doFilter, (void*) &thread_data_array[i]);
    if (iret[i])
    {
      fprintf(stderr, "Error - pthread_create() return code: %d\n", iret[i]);
      exit(EXIT_FAILURE);
    }
  }

  for (int i = 0; i < NUM_THREADS; i++)
  {
    pthread_join( threadProcs[i], NULL);
  }


}

