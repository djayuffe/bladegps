# Makefile for Linux etc.

.PHONY: all check clean
all: bladegps

SHELL=/bin/bash
CC?=gcc
PKG_CONFIG?=pkg-config

CFLAGS?=-O3 -Wall
BLADERF_CFLAGS:=$(shell $(PKG_CONFIG) --cflags libbladeRF 2>/dev/null)
BLADERF_LIBS:=$(shell $(PKG_CONFIG) --libs libbladeRF 2>/dev/null)

ifeq ($(strip $(BLADERF_LIBS)),)
BLADERF_CFLAGS=-I../bladeRF/host/libraries/libbladeRF/include
BLADERF_LIBS=-L../bladeRF/host/build/output -lbladeRF
endif

CPPFLAGS+=-I. $(BLADERF_CFLAGS)
LDLIBS+=-lm -lpthread $(BLADERF_LIBS)

bladegps: bladegps.o gpssim.o getch.o
	${CC} $^ ${LDFLAGS} ${LDLIBS} -o $@

tests/test_core: tests/test_core.o gpssim.o getch.o
	${CC} $^ ${LDFLAGS} ${LDLIBS} -o $@

check: tests/test_core
	./tests/test_core

clean:
	rm -f *.o tests/*.o tests/test_core bladegps
