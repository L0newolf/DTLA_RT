#define NROWS 4
#define NCOLS 4
#define CORES NROWS *NCOLS
#define NUMANGLES 30
#define SLEW 0.00000158
#define SPACING 0.65
#define SPEED 1500
#define Fs 24038
#define pi 3.14159265
#define WINDOWPERCORE 5
#define WINDOWLEN CORES*WINDOWPERCORE
#define SHMSIZE 0x00600000
#define SHMOFFSET 0x01000000
#define ANGLERES 360
#define NUMCHANNELS 24

#define nFFT 512
#define numOverlap 128

#define numTaps 128
#define freqUpper 3200
#define freqLower 2800
#define beta 3.6

#define powerThreshold 0.1
#define lambda 0.7

#define SKIP_RATE 2

#define PROCESS_BEAMFORM 0x00000000
#define PROCESS_FILTER	 0x00000002
#define PROCESS_COMPLETE 0x00000001 	 	