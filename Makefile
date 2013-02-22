obj-m += raspicommrs485.o

raspicommrs485-objs := module.o queue.o

SRC = /home/mdk/workspace/raspicomm-module
LINUX_3_2_27 = /home/mdk/workspace/rpi/linux-rpi-3.2.27/
LINUX_3_6_11 = /home/mdk/workspace/rpi/linux-rpi-3.6.y/

PREFIX = /home/mdk/workspace/rpi/tools-master/arm-bcm2708/arm-bcm2708-linux-gnueabi/bin/arm-bcm2708-linux-gnueabi-

all:
	make ARCH=arm CROSS_COMPILE=$(PREFIX) -C $(LINUX_3_2_27) M=$(SRC) modules

3_2_27:
	make ARCH=arm CROSS_COMPILE=$(PREFIX) -C $(LINUX_3_2_27) M=$(SRC) modules

3_6_11:
	make ARCH=arm CROSS_COMPILE=$(PREFIX) -C $(LINUX_3_6_11) M=$(SRC) modules

clean:
	make -C $(LINUX_3_2_27) M=$(SRC) clean
	make -C $(LINUX_3_6_11) M=$(SRC) clean