Xbox-Controller-to-Wild-Thumper - Version 2
===============================

This repository contains code that connects the Dagu Wild Thumper to the BeagleBone Black.

Step 1: Clone into repository.
Step 2: Edit the Makefile to set up your ARM cross compiler path on your host computer including i2cbusses.c (make sure your joystick is represented in the file descriptor)
Step 3: Compile the code on the host (ensure the port numbers are the same, and that your I2C bus and address are correct)
Step 4: Run robotd-arm on the robot (acts as a server running as a daemon -- defaults to port 1929 and I2C bus #2)
Step 5: Run driver-station on the Linux host (pass IP and port # of robotd-arm on command line)
Step 6: Drive
