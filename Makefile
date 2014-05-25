# I2C tools for Linux
#
# Copyright (C) 2007  Jean Delvare <khali@linux-fr.org>
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.

TOOLS_CFLAGS	= -Wstrict-prototypes -Wshadow -Wpointer-arith -Wcast-qual \
		   -Wcast-align -Wwrite-strings -Wnested-externs -Winline \
		   -W -Wundef -Wmissing-prototypes -g

TOOLS_TARGETS	= my2cset

# This is the Ubuntu-supplied cross compile toolchain - no gdb :-(
#CROSS_COMPILE	= arm-linux-gnueabi-
# This is the Linaro cross compile toolchain with gdb :-)
CROSS_COMPILE	= arm-linux-gnueabihf-

CC_I2CBUSSES	= i2cbusses-arm
CC_ROBOT	= robot-arm
CC_ROBOTD	= robotd-arm

#
# Programs
#
all:	driver_station $(CC_ROBOTD)	

$(CC_ROBOTD).o: robotd.c
	$(CROSS_COMPILE)gcc $(LDFLAGS) $(TOOLS_CFLAGS) -pthread -lm -lrt -c -o $(CC_ROBOTD).o robotd.c

$(CC_I2CBUSSES).o: i2cbusses.c
	$(CROSS_COMPILE)gcc $(LDFLAGS) $(TOOLS_CFLAGS) -pthread -lm -lrt -c -o $(CC_I2CBUSSES).o i2cbusses.c

robotd-arm:  $(CC_ROBOTD).o $(CC_I2CBUSSES).o
	$(CROSS_COMPILE)gcc $(LDFLAGS) -g -pthread -lm -lrt -o $@ $^

driver_station: i2cbusses.o driver_station.o
	$(CC) $(LDFLAGS) -g -pthread -lm -lrt -o $@ $^

#
# Objects
#

%.o: %.c
	$(CC) $(CFLAGS) $(TOOLS_CFLAGS) -c $< -o $@

clean:
	rm -f i2cbusses.o i2cbusses-arm.o robotd-arm robotd-arm.o driver_station driver_station.o
