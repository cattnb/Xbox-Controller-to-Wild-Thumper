# I2C tools for Linux
#
# Copyright (C) 2007  Jean Delvare <khali@linux-fr.org>
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.

TOOLS_CFLAGS	:= -Wstrict-prototypes -Wshadow -Wpointer-arith -Wcast-qual \
		   -Wcast-align -Wwrite-strings -Wnested-externs -Winline \
		   -W -Wundef -Wmissing-prototypes 

TOOLS_TARGETS	:= my2cset

#
# Programs
#
all:	Client Server


Client:  Client.o i2cbusses.o
	$(CC) $(LDFLAGS) -std=c99 -o $@ $^

Server:  Server.o i2cbusses.o
	$(CC) $(LDFLAGS) -std=c99 -o $@ $^

#
# Objects
#

%.o: %.c
	$(CC) $(CFLAGS) $(TOOLS_CFLAGS) -std=c99 -c $< -o $@

clean:
	rm i2cbusses.o Client Client.o Server Server.o
