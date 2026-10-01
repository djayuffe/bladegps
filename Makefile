# Makefile for Linux etc.

.PHONY: all check clean
all: bladegps

SHELL=/bin/bash
CC?=gcc
PKG_CONFIG?=pkg-config

CFLAGS?=-O3 -Wall
BLADERF_CFLAGS:=$(shell $(PKG_CONFIG) --cflags libbladeRF 2>/dev/null)
BLADERF_LIBS:=$(shell $(PKG_CONFIG) --libs libbladeRF 2>/dev/null)
SDL2_CFLAGS:=$(shell $(PKG_CONFIG) --cflags sdl2 2>/dev/null)
SDL2_LIBS:=$(shell $(PKG_CONFIG) --libs sdl2 2>/dev/null)

ifeq ($(strip $(BLADERF_LIBS)),)
BLADERF_CFLAGS=-I../bladeRF/host/libraries/libbladeRF/include
BLADERF_LIBS=-L../bladeRF/host/build/output -lbladeRF
endif

CPPFLAGS+=-I. $(BLADERF_CFLAGS) $(SDL2_CFLAGS)
ifneq ($(strip $(SDL2_LIBS)),)
CPPFLAGS+=-DHAVE_SDL2
endif
LDLIBS+=-lm -lpthread $(BLADERF_LIBS) $(SDL2_LIBS)

bladegps: bladegps.o blade_hw.o gpssim.o gnss_task.o motion_controller.o gnss.o gnss_time.o gnss_receiver.o gnss_codes.o gnss_fec.o gnss_nav.o gnss_orbit.o gnss_geometry.o gnss_galileo_nav.o gnss_beidou_nav.o gnss_glonass_nav.o gnss_rf.o gnss_schedule.o galileo_e1_codes.o getch.o
	${CC} $^ ${LDFLAGS} ${LDLIBS} -o $@

tests/test_core: tests/test_core.o blade_hw.o gpssim.o gnss_task.o motion_controller.o gnss.o gnss_time.o gnss_receiver.o gnss_codes.o gnss_fec.o gnss_nav.o gnss_orbit.o gnss_geometry.o gnss_galileo_nav.o gnss_beidou_nav.o gnss_glonass_nav.o gnss_rf.o gnss_schedule.o galileo_e1_codes.o getch.o
	${CC} $^ ${LDFLAGS} ${LDLIBS} -o $@

check: tests/test_core
	./tests/test_core

clean:
	rm -f *.o tests/*.o tests/test_core bladegps
