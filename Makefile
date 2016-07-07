# Copyright (C) 2016 by Dennis Glatting <dg@pki2.com>
#
# Simple makefile for rtl_acars_ng.
#
#
# $Log: Makefile,v $
# Revision 1.1  2016/07/06 19:35:32  dennisg
# Initial revision
#

OPT := -O

all:
	g++ -o rtl_acars_ng rtl_acars_ng.cc Buffer.cc print.cc sin.cc \
	utility.cc crc.cc \
	${OPT} -g -Wall -pthread -finline -fopenmp -std=c++11 \
	-Ddpgdebug -UNDEBUG \
	-lfftw3_omp -lfftw3 -lvolk \
	-I /usr/include/libusb-1.0/ -I /usr/local/include -I . \
	-L /usr/local/lib -lrtlsdr -lm
	size rtl_acars_ng

clean:
	-rm *~ *.o a.out rtl_acars_ng

