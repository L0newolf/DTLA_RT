//Standard headers
#include <math.h>
#include <stddef.h>
#include <iostream>
#include <cstring>
#include <tuple>
#include <fstream>
#include <sstream>
#include <unistd.h>
#include <sys/time.h>

#include <e-hal.h>
#include <e-loader.h>

using namespace std;

e_epiphany_t Epiphany, *pEpiphany;
e_mem_t      ERAM,     *pERAM;
extern e_platform_t e_platform;

/*************************************************************************************************************************************/
static struct timeval t0;

static inline void tick(void)
{
  gettimeofday(&t0, NULL);
}

static inline float tock(void)
{
  struct timeval t1;
  gettimeofday(&t1, NULL);
  return (t1.tv_sec - t0.tv_sec) * 1000 + ((float)t1.tv_usec - t0.tv_usec) / 1000 ;
}

float tdiff1 = 0;
/*************************************************************************************************************************************/

/*****************************************************************************/
#define ERAM_BUF_SZ    153600
#define SRAM_BUF_SZ    9600

char    ebuf[ERAM_BUF_SZ];
char    sbuf[SRAM_BUF_SZ];

__suseconds_t d;
int ERAM_speed() {
  int i;

  cout << "Bytes beng copied : " << ERAM_BUF_SZ << endl;

  tdiff1 = 0.0;
  for (i = 0; i < 100; i++)
  {
    tick();
    e_write(pERAM, 0, 0, (off_t) 0, ebuf, ERAM_BUF_SZ);
    tdiff1 += tock();
  }

  printf("\nTime to write to ERAM: %f \n", tdiff1 / 100);

  tdiff1 = 0.0;
  for (i = 0; i < 100; i++)
  {
    tick();
    e_read(pERAM, 0, 0, (off_t) 0, ebuf, ERAM_BUF_SZ);
    tdiff1 += tock();
  }

  printf("\nTime to read from ERAM: %f \n", tdiff1 / 100);

  for (unsigned int r = 0; r < e_platform.rows; r++)
  {
    for (unsigned int c = 0; c < e_platform.cols; c++)
    {
      tdiff1 = 0.0;
      for (i = 0; i < 100; i++)
      {
        tick();
        e_write(pEpiphany, r, c, (off_t) 0, sbuf, SRAM_BUF_SZ);
        tdiff1 += tock();
      }

      printf("\nTime to write to CORE: %f \n", tdiff1 / 100);

      tdiff1 = 0.0;
      for (i = 0; i < 100; i++)
      {
        tick();
        e_read(pEpiphany, r, c, (off_t) 0, sbuf, SRAM_BUF_SZ);
        tdiff1 += tock();
      }

      printf("\nTime to read from CORE: %f \n", tdiff1 / 100);
    }
  }

  return 0;
}
/*******************************************************************************/

int main(int argc, char *argv[])
{

  pEpiphany = &Epiphany;
  pERAM     = &ERAM;

  e_set_host_verbosity(H_D0);

  if ( E_OK != e_init(NULL) ) {
    fprintf(stderr, "\nERROR: epiphinay initialization failed!\n\n");
    exit(1);
  }

  if (E_OK != e_reset_system() ) {
    fprintf(stderr, "\nWARNING: epiphinay system rest failed!\n\n");
  }

  // prepare ERAM
  if (E_OK != e_alloc(pERAM, 0x00000000, e_platform.emem[0].size))
  {
    fprintf(stderr, "\nERROR: Can't allocate Epiphany DRAM!\n\n");
    exit(1);
  }
  e_set_host_verbosity(H_D0);

  if (E_OK != e_open(pEpiphany, 0, 0, e_platform.rows, e_platform.cols))
  {
    fprintf(stderr, "\nERROR: Can't establish connection to Epiphany device!\n\n");
    exit(1);
  }

  ERAM_speed();


  //Finalize
  e_close(pEpiphany);
  e_free(pERAM);
  e_finalize();

  return (EXIT_SUCCESS);
}


