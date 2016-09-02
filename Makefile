#******************************************************************************
#
# Makefile - Rules for building the PMT Control Unit Firmware.
#
# Copyright (c) 2015 Kimon Tsitsikas, Delmic
# 
# This file is part of the PMT Control Unit Firmware.
#
# PMT Control Unit Firmware is free software: you can redistribute it and/or 
# modify it under the terms of the GNU General Public License version 2 as 
# published by the Free Software Foundation.
# 
# PMT Control Unit Firmware is distributed in the hope that it will be useful, 
# but WITHOUT ANY WARRANTY; without even the implied warranty of 
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General 
# Public License for more details.
# 
# You should have received a copy of the GNU General Public License along with 
# PMT Control Unit Firmware. If not, see http://www.gnu.org/licenses/.
#
# Linking PMT Control Unit Firmware statically or dynamically with other 
# modules is making a combined work based on PMT Control Unit Firmware. Thus, 
# the terms and conditions of the GNU General Public License cover the whole 
# combination.
#
# In addition, as a special exception, the copyright holders of PMT Control 
# Unit Firmware give you permission to combine PMT Control Unit Firmware with 
# free software programs or libraries that are released under the GNU LGPL and 
# with code included in the standard release of Tiva Firmware Development 
# Package under its own license (or modified versions of such code, with 
# unchanged license). You may copy and distribute such a system following the 
# terms of the GNU GPL for PMT Control Unit Firmware and the licenses of the 
# other code concerned.
#
# Note that people who make modified versions of PMT Control Unit Firmware are 
# not obligated to grant this special exception for their modified versions; 
# it is their choice whether to do so. The GNU General Public License gives 
# permission to release a modified version without this exception; this 
# exception also makes it possible to release a modified version which carries 
# forward this exception.
#
#******************************************************************************

#Part Declaration
PART=TM4C123GH6PM

# Location of the firmware Makefile
rootdir = $(realpath .)
# This works assuming that TivaWare is just in the same directory with our firmware
# Needs to be modified to make sure it points to the correct directory
TIVAWARE_LIB = $(rootdir)/../tivaware

#Create symbolic links to startup and linker
HACK:=$(shell mkdir -p gcc)

#HACK:=$(shell ln -sf $(rootdir)/main.ld gcc/oslo.ld)
#HACK:=$(shell ln -sf $(rootdir)/main.ld gcc/monash.ld)
HACK:=$(shell ln -sf $(rootdir)/startup_gcc.c gcc/.)

VERSION=$(shell git describe --tags --dirty --always)

all: gcc/oslo.axf gcc/monash.axf

gcc/oslo.ld: $(rootdir)/main.ld
	ln -sf $(rootdir)/main.ld ${@}

gcc/monash.ld: $(rootdir)/main.ld
	ln -sf $(rootdir)/main.ld ${@}

gcc/oslo.o: main.c
	@if [ 'x${VERBOSE}' = x ];                            \
	 then                                                 \
	     echo "  CC    ${<}";                             \
	 else                                                 \
	     echo ${CC} ${CFLAGS} -DDESIGN=\"oslo\" -DCOMPARATOR -DSOFTSWITCH -DUART -D${COMPILER} -o ${@} ${<}; \
	 fi
	@${CC} ${CFLAGS} -DDESIGN=\"oslo\" -DCOMPARATOR -DSOFTSWITCH -DUART -D${COMPILER} -o ${@} ${<}

gcc/monash.o: main.c
	@if [ 'x${VERBOSE}' = x ];                            \
	 then                                                 \
	     echo "  CC    ${<}";                             \
	 else                                                 \
	     echo ${CC} ${CFLAGS} -DDESIGN=\"monash\" -D${COMPILER} -o ${@} ${<}; \
	 fi
	@${CC} ${CFLAGS} -DDESIGN=\"monash\" -D${COMPILER} -o ${@} ${<}

include $(TIVAWARE_LIB)/makedefs

# The generic .axf rule just 
gcc/oslo.axf: gcc/oslo.o gcc/startup_gcc.o gcc/ustdlib.o gcc/uartstdio.o $(TIVAWARE_LIB)/usblib/gcc/libusb.a $(TIVAWARE_LIB)/driverlib/gcc/libdriver.a gcc/oslo.ld

SCATTERgcc_oslo=gcc/oslo.ld
ENTRY_oslo=ResetISR

gcc/monash.axf: gcc/monash.o gcc/startup_gcc.o gcc/ustdlib.o gcc/usb_serial_structs.o $(TIVAWARE_LIB)/usblib/gcc/libusb.a $(TIVAWARE_LIB)/driverlib/gcc/libdriver.a gcc/monash.ld

SCATTERgcc_monash=gcc/monash.ld
ENTRY_monash=ResetISR

VPATH=$(TIVAWARE_LIB)/utils
IPATH=$(TIVAWARE_LIB)/

CFLAGSgcc=-DTARGET_IS_TM4C123_RB1 -std=c99 -U__STRICT_ANSI__ -DVERSION="\"$(VERSION)\""


ifneq (${MAKECMDGOALS},clean)
-include ${wildcard gcc/*.d} __dummy__
endif

clean:
	@rm -vf gcc/*
	@rmdir -v gcc

