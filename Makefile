CC=arm-linux-gnueabihf-g++
ECC=e-gcc

RT = src/rtPingerDet.cpp
FILT_SIG = src/filtSignal.cpp
SIG_FREQ = src/findSigFreq.cpp
GEN_FILT = src/genFilt.cpp
D2R = src/deg2rad.cpp
EBF = src/eFreqBeamForm.c
HIB = src/hilbertTrans.cpp

INC = -I include

CFLAGS = -Wall -fexceptions -std=c++0x -g -O3

LIBS = -le-hal -le-loader -lfftw3f -lm -lpthread

ESDK=/opt/adapteva/esdk
ELIBS=-L ${ESDK}/tools/host/lib
EINCS=-I ${ESDK}/tools/host/include
ELDF=${ESDK}/bsps/current/fast.ldf
EHDF=${ESDK}/bsps/current/platform.hdf

OBJ = bin/DTLA

all: cpu ecpu

cpu:
	rm -f ${OBJ}	
	${CC} ${CFLAGS}  ${INC} ${EINCS} ${ELIBS} ${RT} ${FILT_SIG} ${SIG_FREQ} ${GEN_FILT} ${D2R} ${HIB} -o ${OBJ} ${LIBS}

ecpu:
	rm -f bin/eBF.elf
	rm -f bin/eBF.srec	
	e-gcc -O3  -T ${ELDF} ${INC} ${EBF} -o bin/eBF.elf -le-lib -lm -ffast-math -funroll-loops
	e-objcopy --srec-forceS3 --output-target srec bin/eBF.elf bin/eBF.srec	

clean:
	rm -f ${OBJ}
	rm -f bin/eBF.elf
	rm -f bin/eBF.srec	
