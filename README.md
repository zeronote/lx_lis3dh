BBB Pin Mapping Table
======================================
BBB P9 header      | LIS3DH
-------------------|------------------
Pin 1 (DGND)       | GND
Pin 3 (VDD_3V3)    | Vin
Pin 15 (GPIO_48)   | INT
Pin 19 (I2C2_SCL)  | SCL
Pin 20 (I2C2_SDA)  | SDA

# TOOLCHAIN SETUP
## Really important disclaimer
My first really important assumption is that you are working on a Beaglebone Black system with the following:

- `U-Boot 2016.01-00001-g4eb802e (Jan 13 2016 - 11:14:31 -0600)`
- `Linux beaglebone 4.1.15-ti-rt-r43`

otherwise, depending on u-boot and kernel version, some of the followings steps could be erroneous or lead to undefined results.

## Packages
The following packages are needed if you are on a Debian/Ubuntu based host machine:

 - `lib32stdc++6`
 - `lzop`
 - `lib32z1`
 - `device-tree-compiler`

## Setup
Simply executing `setup.sh` should download, configure and cross-compile everything for you except the LIS3DH kernel module that you're supposed to cross-compile by yourself using `make` once the setup.sh script exit. If everything went well, you should have the followings in your current directory:

 - `lxdriver.ko`
 - `BB-LX-ACCEL-00A0.dtbo`

Once gained any kind of access to your BBB, copy the kernel `lxdriver.ko` in `/lib/modules/$(uname -r)`and `BB-LX-ACCEL-00A0.dtbo`to `/lib/firmware`. 

  ## Testing
  Since the `universal_cape` overlay is enabled by default, we need to disable it to avoid race conditions on the `GPIO_48` pin once our overlay is loaded: with `sudo vim /boot/uEnv.txt` change this line:
  
  `cmdline=coherent_pool=1M quiet cape_universal=enable`
  
  to
  
  `cmdline=coherent_pool=1M quiet`

and reboot your BBB.

Finally we can load the dtb overlay

`sudo sh -c 'echo BB-LX-ACCEL-00A0.DTBO > /sys/devices/platform/bone_capemgr/slots`

load the kernel module:

`sudo insmod /lib/firmware/lxdriver.ko`

and check our 3-axis accelerations:

`sudo cat /dev/lxaccell` 

> debian@beaglebone:~$ sudo cat /dev/lxaccell  
787,298,540  
787,298,540  
780,304,541  
780,304,541  
780,304,541  
780,304,541  
780,304,541  
780,304,541  
780,304,541  
780,304,541  
780,304,541  
780,304,541  
780,304,541  
780,304,541  
780,304,541  
780,304,541  
780,304,541  
780,304,541  
780,304,541  
780,304,541  
780,304,541
