# rpi_soft_pwm_linux_driver

This is the Raspberry Pi 0/1/2/3/4 software based PWM Linux Driver source code for **Yocto** Meta layer integration

This is primarily intended for controlling Servo motors but can be extended to other PWM applications.

I have tested with **SG90** Servo Motor

### Option 1 -> (No Yocto) Standalone Linux Driver:

>cd  

>make -f Makefile.NoYocto

### Option 2 -> Yocto Linux Driver with OS Integration (Poky Reference):

I have used the reference Yocto meta-jumpnowtek layer as explained on Jumpnowtek Technologies website

The process to building raspberry pi systems with Yocto is explained very well by Jumpnowtek Technologies. They have custom meta layer for Yocto on GitHub.

The following URLs contain all the documentation (barring a few changes listed below)

https://jumpnowtek.com/rpi/Raspberry-Pi-Systems-with-Yocto.html (32-bit OS)

https://jumpnowtek.com/rpi/Raspberry-Pi-Systems-with-Yocto.html (64-bit OS)

### Follow the instructions on the above URL meticulously with the following changes

> source ~/poky-dunfell/oe-init-build-env ~/rpi/build

Change 1: Copy the servo-mod PWM Servo Linux Driver directory inside meta-jumpnow/kernel-receipes

>cd [path to this repo]/rpi_soft_pwm_linux_driver 

>cp -a ./servo-mod ~/poky-dunfell/meta-jumpnow/recipes-kernel/servo-mod

>cd -

Change 2: Copy the bblayers.conf and local.conf to ~/rpi/build

>cp ./conf/{bblayers.conf,local.conf} ~/rpi/build

### Assuming one keeps all the default locations as explained in the above tutorial

>cd ~/rpi/build

>bitbake console-image

### This piece of software has been tested with Raspberry Pi B+. It is expected to work with Rpi 0/1/2/3/4 with one change in the Linux Driver code

>#define SOC_PERI_BASE       RPI1_PERI_BASE      //<------------------- Replace this MACRO with appropriate Raspberry pi prior to compiling or building Yocto

Example for Raspberry Pi 4

>#define SOC_PERI_BASE       RPI4_PERI_BASE     

### To just build the servo kernel module after any code changes to servo.c file (This assumes the above command has been run once)

>cd ~/rpi/build

>bitbake -c cleansstate servo-mod && bitbake servo-mod

The process to load files on boot and rootfs partitions on microSD/SSD card is listed above 

The driver will autoload upon OS boot

To load driver on raspberry pi from

>ls /lib/modules/5.4.83/kernel/servo/servo.ko

>modprobe servo.ko

or from the (~) home directory on raspberry pi

>insmod servo.ko

Ensure the driver is loaded

> lsmod | grep -i servo

### Hardware:

The PWM signal is generated on pin GPIO pin 17 

https://www.raspberrypi.org/documentation/usage/gpio/

To control the Servo follow the wiring diagram where 

http://www.ee.ic.ac.uk/pcheung/teaching/DE1_EE/stores/sg90_datasheet.pdf

>The Red wire on SG90 <-----> 5V power on Raspberry pi (pin 2)

>The Brown wire on SG90 <-----> GND on Raspberry pi (pin 9)

>The Orange wire on SG90 <-----> GPIO pin 17 on Raspberry pi (pin 11)

Copy the script (scp to Raspberry pi)

>scp sg90servo_example.sh root@<raspberry pi ip>:~

on raspberry pi

>chmod a+x sg90servo_example.sh

To turn motor back and forth 3 times run

> ./sg90servo_example.sh 3

### Tip 1 (on microSD card before booting, make the following changes in etc/network/interfaces file)

If using Wifi wlan0 on raspberry pi then 

Add the following to /etc/network/interfaces to raspberry pi file system

>auto wlan0

>allow-hotplug wlan0

>iface wlan0 inet dhcp

>        wireless_mode managed

>        wpa-conf /etc/wpa_supplicant.conf

>        post-up /etc/activate_wlan0.sh

Inside 

>touch /etc/activate_wlan0.sh

>chmod a+x /etc/activate_wlan0.sh

>#!/bin/bash

>ifdown wlan0

>ifup wlan0

### Tip 2 (on microSD card before booting, configure your WiFi SSID and passphrase in etc/wpa_supplicant.conf file)

>ctrl_interface=/var/run/wpa_supplicant

>ap_scan=1

>

>network={

>    key_mgmt=WPA-PSK

>    ssid="<ssid>"

>    psk="<passphrase>"

>}
