Xbox-Controller-to-Wild-Thumper
===============================

This repository contains code that connects the Dagu Wild Thumper to the BeagleBone Black.

Step 1: Clone into repository.
Step 2: Compile Server.c on your host computer including i2cbusses.c (make sure your joystick is represented in the file descriptor)
Step 3: Compile Client.c on your BeagleBone Black including i2cbusses.c (ensure the port numbers are the same, and that your I2C bus and address are correct)
Step 4: Run Server
Step 5: Run Client
Step 6: Drive

See http://elinux.org/ECE497_Project:_Xbox_Controller_to_Wild_Thumper for further details
