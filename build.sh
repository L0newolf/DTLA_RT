#!/bin/bash

set -e

ESDK=${EPIPHANY_HOME}
ELIBS="-L ${ESDK}/tools/host/lib"
EINCS="-I ${ESDK}/tools/host/include"
ELDF=${ESDK}/bsps/current/internal.ldf

SCRIPT=$(readlink -f "$0")
EXEPATH=$(dirname "$SCRIPT")
cd $EXEPATH

# Create the binaries directory
mkdir -p bin/

CROSS_PREFIX=
case $(uname -p) in
	arm*)
		# Use native arm compiler (no cross prefix)
		CROSS_PREFIX=
		;;
	   *)
		# Use cross compiler
		CROSS_PREFIX="arm-linux-gnueabihf-"
		;;
esac

RT=src/rtPingerDet.cpp
FILT_SIG=src/filtSignal.cpp
SIG_FREQ=src/findSigFreq.cpp
GEN_FILT=src/genFilt.cpp
D2R=src/deg2rad.cpp
EBF=src/eFreqBeamForm.c
HIB=src/hilbertTrans.cpp

OBJ=bin/DTLA

INC="-I include"
LIBS="-le-hal -le-loader -lpthread -lrt -lfftw3f -lm"
CFLAGS="-Wall -fexceptions -std=c++0x -g -O3"

# Build HOST side application
echo Deleting previous binaries
rm -f ${OBJ}
echo Building host binary
${CROSS_PREFIX}g++ ${CFLAGS} ${INC} ${EINCS} ${ELIBS} ${RT} ${FILT_SIG} ${SIG_FREQ} ${GEN_FILT} ${D2R} ${HIB} -o ${OBJ} ${EINCS} ${ELIBS} ${LIBS}
echo Host binary built 
