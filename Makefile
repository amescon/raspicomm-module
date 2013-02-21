obj-m += raspicommrs485.o

raspicommrs485-objs := module.o queue.o

SRC = /home/mdk/workspace/raspicomm-module
LINUX = /home/mdk/workspace/rpi/linux-rpi-3.2.27/
PREFIX = /home/mdk/workspace/rpi/tools-master/arm-bcm2708/arm-bcm2708-linux-gnueabi/bin/arm-bcm2708-linux-gnueabi-

all:
	make ARCH=arm CROSS_COMPILE=$(PREFIX) -C $(LINUX) M=$(SRC) modules

clean:
	make -C $(LINUX) M=$(SRC) clean